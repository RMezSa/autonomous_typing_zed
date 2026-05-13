#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <typing_interfaces/action/execute_key.hpp>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <memory>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <vector>

class ArmNode : public rclcpp::Node {
    public:
        using ExecuteKey = typing_interfaces::action::ExecuteKey;
        using GoalHandleExecuteKey = rclcpp_action::ServerGoalHandle<ExecuteKey>;

        // Internal tuning constants. Were ROS parameters earlier; collapsed because the
        // operator never tunes these and they cluttered the launch surface.
        static constexpr double kInterpRateHz = 50.0;
        static constexpr double kTargetReachedTolDeg = 0.5;
        static constexpr double kJointStateStaleSec = 2.0;
        // Arrival check on real joint feedback is looser than the commanded-pose tol because
        // feedback has lag + quantization. The action will still time out if hardware never
        // catches up, so this only affects how quickly "arrived" trips after the interp
        // finishes ramping.
        static constexpr double kArrivalFeedbackTolDeg = 2.0;

        ArmNode() : Node("arm_node") {
            pub_q1_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint1", 1);
            pub_q2_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint2", 1);
            pub_q3_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint3", 1);
            pub_q4_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint4", 1);
            pub_q5_ = this->create_publisher<std_msgs::msg::Int32>("arm_teleop/joint5", 1);
            debug_status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_ik/debug_status", 10);
            at_target_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arm_ik/at_target", 10);

            this->declare_parameter("publish_on_action", false);
            publish_on_action_ = this->get_parameter("publish_on_action").as_bool();

            this->declare_parameter("max_step_deg_per_tick", 1.5);
            // Default matches the rover arm's physical rest pose (confirmed by the
            // quantum_interface UI). Only used as the interpolator's starting point
            // when /joint_states is missing — if firmware publishes joint feedback,
            // onJointStates() overrides this on first message anyway.
            this->declare_parameter("start_pose_deg",
                std::vector<double>{0.0, 190.0, -140.0, -50.0, 0.0});
            this->declare_parameter("joint_state_topic", std::string("/joint_states"));
            this->declare_parameter("joint_state_names",
                std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5"});
            this->declare_parameter("joint_state_in_degrees", false);

            max_step_deg_per_tick_ = this->get_parameter("max_step_deg_per_tick").as_double();
            joint_state_names_ = this->get_parameter("joint_state_names").as_string_array();
            joint_state_in_degrees_ = this->get_parameter("joint_state_in_degrees").as_bool();

            const auto start_pose = this->get_parameter("start_pose_deg").as_double_array();
            if (start_pose.size() == 5) {
                for (size_t i = 0; i < 5; ++i) last_published_deg_[i] = start_pose[i];
            }

            sub_goal_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/goal", 10,
                std::bind(&ArmNode::onGoal, this, std::placeholders::_1)
            );

            sub_predefined_ = this->create_subscription<std_msgs::msg::String>(
                "/predefined", 10,
                std::bind(&ArmNode::onPredefined, this, std::placeholders::_1)
            );

            const std::string js_topic = this->get_parameter("joint_state_topic").as_string();
            sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
                js_topic, 10,
                std::bind(&ArmNode::onJointStates, this, std::placeholders::_1)
            );

            action_server_ = rclcpp_action::create_server<ExecuteKey>(
                this,
                "/arm_ik/execute_key",
                std::bind(&ArmNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ArmNode::handleCancel, this, std::placeholders::_1),
                std::bind(&ArmNode::handleAccepted, this, std::placeholders::_1)
            );

            debug_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(250),
                std::bind(&ArmNode::publishDebugStatus, this)
            );

            interp_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / kInterpRateHz)),
                std::bind(&ArmNode::tickInterpolator, this)
            );

            RCLCPP_INFO(
                this->get_logger(),
                "arm_node ready. publish_on_action=%s max_step=%.2fdeg/tick @ %.1fHz",
                publish_on_action_ ? "true" : "false",
                max_step_deg_per_tick_, kInterpRateHz);
        }

    private:
        static double radToDeg(double r) {
            return r * (180.0 / M_PI);
        }

        static int myMap(double in_min, double in_max, int out_min, int out_max, double x) {
            return static_cast<int>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
        }

        static bool qlimit(const double lim[2], double v) {
            return (v < lim[0] || v > lim[1]);
        }

        static bool finiteAll(double a, double b, double c, double d, double e) {
            return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) &&
                   std::isfinite(d) && std::isfinite(e);
        }

        bool inverseKinematics(double x, double y, double z,
                               double rollRad, double pitchRad,
                               double &q1d, double &q2d, double &q3d, double &q4d, double &q5d) {

            const double l1 = 0.1;
            const double l2 = 0.43;
            const double l3 = 0.43;
            const double l4 = 0.213;

            if (!finiteAll(x, y, z, rollRad, pitchRad)) return false;

            const double q1 = std::atan2(y, x);
            const double q5 = rollRad;

            const double a = std::sqrt(x*x + y*y) - l4 * std::cos(pitchRad);
            const double b = z - (l4 * std::sin(pitchRad)) - l1;

            const double d_raw = (a*a + b*b - l2*l2 - l3*l3) / (2.0 * l2 * l3);
            constexpr double kReachabilityEpsilon = 1e-6;
            if (!std::isfinite(d_raw) ||
                d_raw < -1.0 - kReachabilityEpsilon ||
                d_raw > 1.0 + kReachabilityEpsilon) {
                return false;
            }

            const double d = std::max(-1.0, std::min(1.0, d_raw));

            const double q3 = -std::atan2(std::sqrt(1.0 - d*d), d);
            const double q2 = std::atan2(b, a) - std::atan2(l3 * std::sin(q3), l2 + l3 * std::cos(q3));
            const double q4 = pitchRad - q2 - q3;

            // Real-rover convention: each motor's value=0 corresponds to a horizontal link, positive
            // values rotate upward (q1: positive = CCW looking from above). This matches the IK's
            // native convention (+ rotation around -Y = upward), so we publish raw rad-to-deg with
            // no sign flips and no offsets. NOTE: this breaks the sim/URDF mapping, which assumed
            // q2 rest = vertical and pitch axes around +Y.
            q1d = radToDeg(q1);
            q2d = radToDeg(q2);
            q3d = radToDeg(q3);
            q4d = radToDeg(q4);
            q5d = radToDeg(q5);

            const double lim_q1[2] = {-90, 90};
            const double lim_q2[2] = {-10, 190};
            const double lim_q3[2] = {-150, 150};
            const double lim_q4[2] = {-150, 150};
            const double lim_q5[2] = {-90, 90};

            if (qlimit(lim_q1, q1d) ||
                qlimit(lim_q2, q2d) ||
                qlimit(lim_q3, q3d) ||
                qlimit(lim_q4, q4d) ||
                qlimit(lim_q5, q5d)) {
                return false;
                }

            return true;

        }

    void publishJointsRaw(const std::array<double, 5> &q) {
        std_msgs::msg::Float64 m;
        m.data = q[0]; pub_q1_->publish(m);
        m.data = q[1]; pub_q2_->publish(m);
        m.data = q[2]; pub_q3_->publish(m);
        m.data = q[3]; pub_q4_->publish(m);

        std_msgs::msg::Int32 s;
        s.data = myMap(-90.0, 90.0, 88, 268, q[4]);
        pub_q5_->publish(s);
    }

    void setTarget(double q1, double q2, double q3, double q4, double q5) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            target_deg_[0] = q1;
            target_deg_[1] = q2;
            target_deg_[2] = q3;
            target_deg_[3] = q4;
            target_deg_[4] = q5;
            target_valid_ = true;
        }
        // New target invalidates the previous arrival latch. The interp tick will set it
        // back to true once last_published_deg_ converges (and joint feedback agrees, when
        // available).
        at_target_.store(false, std::memory_order_release);
        publishAtTarget(false);
    }

    // True when joint feedback is fresh and all fed joints are within
    // kArrivalFeedbackTolDeg of `target`. Stale/missing feedback returns true so we fall
    // back to commanded-pose arrival.
    bool feedbackAgreesWithTarget(const std::array<double, 5>& target) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!joint_state_valid_) return true;
        const double age = this->get_clock()->now().seconds() - last_joint_state_time_;
        if (age < 0.0 || age >= kJointStateStaleSec) return true;
        for (size_t i = 0; i < 5; ++i) {
            if (!joint_state_index_valid_[i]) continue;
            if (!std::isfinite(joint_state_deg_[i]) || !std::isfinite(target[i])) return false;
            if (std::abs(joint_state_deg_[i] - target[i]) > kArrivalFeedbackTolDeg) return false;
        }
        return true;
    }

    void publishAtTarget(bool v) {
        if (!at_target_pub_) return;
        std_msgs::msg::Bool m;
        m.data = v;
        at_target_pub_->publish(m);
    }

    // Steps last_published_deg_ toward target_deg_ with per-tick proportional clamping.
    // Proportional (not per-joint independent) preserves the joint-space straight line.
    void tickInterpolator() {
        std::array<double, 5> target;
        std::array<double, 5> last;
        bool valid;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            target = target_deg_;
            last = last_published_deg_;
            valid = target_valid_;
        }
        if (!valid) {
            publishAtTarget(at_target_.load(std::memory_order_acquire));
            return;
        }

        std::array<double, 5> delta{};
        double max_abs_delta = 0.0;
        for (size_t i = 0; i < 5; ++i) {
            if (!std::isfinite(last[i]) || !std::isfinite(target[i])) {
                publishAtTarget(at_target_.load(std::memory_order_acquire));
                return;
            }
            delta[i] = target[i] - last[i];
            max_abs_delta = std::max(max_abs_delta, std::abs(delta[i]));
        }
        if (max_abs_delta < kTargetReachedTolDeg) {
            // Commanded pose converged. Promote to "at_target" only if joint feedback
            // confirms (or is unavailable/stale, in which case we trust the commanded
            // pose).
            if (feedbackAgreesWithTarget(target)) {
                at_target_.store(true, std::memory_order_release);
            }
            publishAtTarget(at_target_.load(std::memory_order_acquire));
            return;
        }

        double ratio = 1.0;
        if (max_abs_delta > max_step_deg_per_tick_) {
            ratio = max_step_deg_per_tick_ / max_abs_delta;
        }

        std::array<double, 5> next_q{};
        for (size_t i = 0; i < 5; ++i) {
            next_q[i] = last[i] + delta[i] * ratio;
        }

        publishJointsRaw(next_q);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_published_deg_ = next_q;
        }
        publishAtTarget(false);
    }

    void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Partial joint sets are allowed: the bridge publishes joints 1..4 only
        // (no feedback for the gripper servo). Indices missing from the message
        // keep their previous cached value.
        std::array<bool, 5> updated{false, false, false, false, false};
        std::array<double, 5> tmp{};

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            tmp = joint_state_deg_;
        }

        for (size_t i = 0; i < 5; ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_state_names_[i]);
            if (it == msg->name.end()) continue;
            const size_t idx = static_cast<size_t>(it - msg->name.begin());
            if (idx >= msg->position.size()) continue;
            double v = msg->position[idx];
            if (!std::isfinite(v)) continue;
            if (!joint_state_in_degrees_) v *= 180.0 / M_PI;
            tmp[i] = v;
            updated[i] = true;
        }

        bool any_updated = false;
        for (size_t i = 0; i < 5; ++i) {
            if (updated[i]) { any_updated = true; break; }
        }
        if (!any_updated) return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        joint_state_deg_ = tmp;
        for (size_t i = 0; i < 5; ++i) {
            if (updated[i]) joint_state_index_valid_[i] = true;
        }
        joint_state_valid_ = true;
        last_joint_state_time_ = this->get_clock()->now().seconds();

        // Bootstrap: snap last_published_ to the measured pose so the first commanded
        // motion ramps from where the arm actually is. Only joints we have feedback
        // for are updated; the rest keep their start_pose_deg default.
        if (!target_valid_) {
            for (size_t i = 0; i < 5; ++i) {
                if (updated[i]) last_published_deg_[i] = tmp[i];
            }
        }
    }

    bool runIKAndPublish(double x, double y, double z, double rollDeg, double pitchDeg, bool publish_commands = true) {
        const double rollRad = rollDeg * M_PI / 180.0;
        const double pitchRad = pitchDeg * M_PI / 180.0;

        double q1, q2, q3, q4, q5;
        if (!inverseKinematics(x, y, z, rollRad, pitchRad, q1, q2, q3, q4, q5)) return false;

        if (publish_commands) {
            setTarget(q1, q2, q3, q4, q5);
        }

        last_ik_ok_ = true;
        last_ik_message_ = publish_commands ? "ik_ok_target_set" : "ik_ok_dry_run";
        last_command_time_sec_ = this->get_clock()->now().seconds();
        return true;
    }

    void publishDebugStatus() {
        std::array<double, 5> last;
        std::array<double, 5> target;
        std::array<double, 5> js;
        std::array<bool, 5> js_idx_valid;
        bool t_valid, js_valid;
        double js_time;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last = last_published_deg_;
            target = target_deg_;
            js = joint_state_deg_;
            js_idx_valid = joint_state_index_valid_;
            t_valid = target_valid_;
            js_valid = joint_state_valid_;
            js_time = last_joint_state_time_;
        }

        const double now_sec = this->get_clock()->now().seconds();
        const double js_age = js_valid ? (now_sec - js_time) : -1.0;
        const bool js_fresh = js_valid && js_age >= 0.0 && js_age < kJointStateStaleSec;

        double tracking_err = -1.0;
        if (js_fresh) {
            tracking_err = 0.0;
            for (size_t i = 0; i < 5; ++i) {
                if (!js_idx_valid[i]) continue;
                if (!std::isfinite(last[i]) || !std::isfinite(js[i])) {
                    tracking_err = -1.0;
                    break;
                }
                tracking_err = std::max(tracking_err, std::abs(last[i] - js[i]));
            }
        }

        double max_remaining = -1.0;
        if (t_valid) {
            max_remaining = 0.0;
            for (size_t i = 0; i < 5; ++i) {
                if (!std::isfinite(last[i]) || !std::isfinite(target[i])) {
                    max_remaining = -1.0;
                    break;
                }
                max_remaining = std::max(max_remaining, std::abs(target[i] - last[i]));
            }
        }

        std::ostringstream os;
        os << std::fixed << std::setprecision(3)
           << "{"
           << "\"publish_on_action\":" << (publish_on_action_ ? "true" : "false") << ","
           << "\"last_command_source\":\"" << last_command_source_ << "\","
           << "\"last_predefined\":\"" << last_predefined_ << "\","
           << "\"last_action_key\":\"" << last_action_key_ << "\","
           << "\"last_action_result\":\"" << last_action_result_ << "\","
           << "\"last_ik_ok\":" << (last_ik_ok_ ? "true" : "false") << ","
           << "\"last_ik_message\":\"" << last_ik_message_ << "\","
           << "\"last_command_time_sec\":" << last_command_time_sec_ << ","
           << "\"goal_xyz\":{"
           << "\"x\":" << gx_ << ","
           << "\"y\":" << gy_ << ","
           << "\"z\":" << gz_ << "},"
           << "\"goal_rp\":{"
           << "\"roll\":" << groll_ << ","
           << "\"pitch\":" << gpitch_ << "},"
           << "\"keyboard_home_set\":" << (kb_home_set_ ? "true" : "false") << ","
           << "\"joint_keyboard_home_set\":" << (kb_home_joints_set_ ? "true" : "false") << ","
           << "\"max_step_deg_per_tick\":" << max_step_deg_per_tick_ << ","
           << "\"target_valid\":" << (t_valid ? "true" : "false") << ","
           << "\"max_remaining_deg\":" << max_remaining << ","
           << "\"last_published_deg\":[" << last[0] << "," << last[1] << "," << last[2] << "," << last[3] << "," << last[4] << "],"
           << "\"target_deg\":[" << target[0] << "," << target[1] << "," << target[2] << "," << target[3] << "," << target[4] << "],"
           << "\"joint_state_valid\":" << (js_valid ? "true" : "false") << ","
           << "\"joint_state_age_sec\":" << js_age << ","
           << "\"joint_state_fresh\":" << (js_fresh ? "true" : "false") << ","
           << "\"tracking_error_max_deg\":" << tracking_err << ","
           << "\"at_target\":" << (at_target_.load(std::memory_order_acquire) ? "true" : "false")
           << "}";

        std_msgs::msg::String msg;
        msg.data = os.str();
        debug_status_pub_->publish(msg);
    }

    double gx_ = 0.15;
    double gy_ = 0.0;
    double gz_ = 0.35;
    double groll_ = 0.0;
    double gpitch_ = 0.0;

    bool kb_home_set_ = false;
    double kb_home_x_ = 0.15;
    double kb_home_y_ = 0.0;
    double kb_home_z_ = 0.35;
    double kb_home_roll_ = 0.0;
    double kb_home_pitch_ = 0.0;

    bool kb_home_joints_set_ = false;
    std::array<double, 5> kb_home_joints_{0.0, 0.0, 0.0, 0.0, 0.0};

    static inline bool isNan(double v) {
        return std::isnan(v);
    }

    void onGoal(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 5) return;

        const double x = msg->data[0];
        const double y = msg->data[1];
        const double z = msg->data[2];
        const double roll = msg->data[3];
        const double pitch = msg->data[4];

        if (!isNan(x)) gx_ = x;
        if (!isNan(y)) gy_ = y;
        if (!isNan(z)) gz_ = z;
        if (!isNan(roll)) groll_ = roll;
        if (!isNan(pitch)) gpitch_ = pitch;

        last_command_source_ = "topic:/goal";
        const bool ok = runIKAndPublish(gx_, gy_, gz_, groll_, gpitch_, true);
        if (!ok) {
            last_ik_ok_ = false;
            last_ik_message_ = "ik_failed_topic_goal";
            RCLCPP_WARN(this->get_logger(), "IK failed for /goal target xyz=(%.3f, %.3f, %.3f)", gx_, gy_, gz_);
        }
    }

    void onPredefined(const std_msgs::msg::String::SharedPtr msg) {
        const std::string &p = msg->data;

        if (p == "SET_KEYBOARD_HOME" || p == "SET_KB_HOME") {
            kb_home_x_ = gx_;
            kb_home_y_ = gy_;
            kb_home_z_ = gz_;
            kb_home_roll_ = groll_;
            kb_home_pitch_ = gpitch_;
            kb_home_set_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Keyboard home captured at xyz=(%.3f, %.3f, %.3f), roll=%.3f, pitch=%.3f",
                kb_home_x_, kb_home_y_, kb_home_z_, kb_home_roll_, kb_home_pitch_
            );
            last_predefined_ = p;
            last_command_source_ = "topic:/predefined";
            last_ik_ok_ = true;
            last_ik_message_ = "keyboard_home_captured";
            last_command_time_sec_ = this->get_clock()->now().seconds();
            return;
        }

        // Joint-space variant. Snapshots the measured joint angles for joints 1..4
        // (from the bridge) and the last commanded joint5 (no feedback for the gripper).
        // Replay (KEYBOARD_HOME_JOINTS) bypasses IK and commands those joints directly —
        // exact pose reproducibility, no IK branch ambiguity.
        if (p == "SET_KEYBOARD_HOME_JOINTS" || p == "SET_KB_HOME_JOINTS") {
            std::array<double, 5> snapshot{};
            std::array<bool, 5> idx_valid;
            bool js_ok;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                js_ok = joint_state_valid_;
                snapshot = joint_state_deg_;
                idx_valid = joint_state_index_valid_;
                snapshot[4] = last_published_deg_[4];
            }
            if (!js_ok || !(idx_valid[0] && idx_valid[1] && idx_valid[2] && idx_valid[3])) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Cannot set joint-space keyboard home: joints 1..4 feedback not yet received on /joint_states.");
                return;
            }
            kb_home_joints_ = snapshot;
            kb_home_joints_set_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Joint keyboard home captured: [%.2f, %.2f, %.2f, %.2f, %.2f] deg",
                kb_home_joints_[0], kb_home_joints_[1], kb_home_joints_[2],
                kb_home_joints_[3], kb_home_joints_[4]
            );
            last_predefined_ = p;
            last_command_source_ = "topic:/predefined";
            last_ik_ok_ = true;
            last_ik_message_ = "joint_keyboard_home_captured";
            last_command_time_sec_ = this->get_clock()->now().seconds();
            return;
        }

        if (p == "KEYBOARD_HOME_JOINTS" || p == "KB_HOME_JOINTS") {
            if (!kb_home_joints_set_) {
                RCLCPP_WARN(this->get_logger(),
                    "Joint keyboard home not set. Send SET_KEYBOARD_HOME_JOINTS first.");
                return;
            }
            setTarget(kb_home_joints_[0], kb_home_joints_[1], kb_home_joints_[2],
                      kb_home_joints_[3], kb_home_joints_[4]);
            last_predefined_ = p;
            last_command_source_ = "topic:/predefined";
            last_ik_ok_ = true;
            last_ik_message_ = "joint_keyboard_home_target_set";
            last_command_time_sec_ = this->get_clock()->now().seconds();
            return;
        }

        if (p == "HOME") {
            gx_ = 0.15; gy_ = 0.0; gz_ = 0.35;
            groll_ = 0.0; gpitch_ = 0.0;
        } else if (p == "KEYBOARD_HOME" || p == "KB_HOME") {
            if (!kb_home_set_) {
                RCLCPP_WARN(this->get_logger(), "Keyboard home not set yet. Send SET_KEYBOARD_HOME first.");
                return;
            }

            gx_ = kb_home_x_; gy_ = kb_home_y_; gz_ = kb_home_z_;
            groll_ = kb_home_roll_; gpitch_ = kb_home_pitch_;
        } else if (p == "INTERMEDIATE") {
            gx_ = 0.20; gy_ = 0.0; gz_ = 0.60;
            groll_ = 0.0; gpitch_ = 0.0;
        } else if (p == "PREFLOOR") {
            gx_ = 0.25; gy_ = 0.0; gz_ = 0.35;
            groll_ = 0.0; gpitch_ = -75.0;
        } else if (p == "FLOOR") {
            gx_ = 0.35; gy_ = 0.0; gz_ = 0.10;
            groll_ = 0.0; gpitch_ = -75.0;
        } else if (p == "STORAGE") {
            gx_ = 0.0; gy_ = 0.0; gz_ = 0.55;
            groll_ = 0.0; gpitch_ = 100.0;
        } else {
            return;
        }

        last_predefined_ = p;
        last_command_source_ = "topic:/predefined";
        const bool ok = runIKAndPublish(gx_, gy_, gz_, groll_, gpitch_, true);
        if (!ok) {
            last_ik_ok_ = false;
            last_ik_message_ = "ik_failed_predefined";
            RCLCPP_WARN(this->get_logger(), "IK failed for predefined '%s'", p.c_str());
        }
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteKey::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "ExecuteKey received for '%s'", goal->key_label.c_str());
        last_action_key_ = goal->key_label;
        last_action_result_ = "accepted";
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleExecuteKey> /*goal_handle*/) {
        RCLCPP_INFO(this->get_logger(), "ExecuteKey cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleExecuteKey> goal_handle) {
        std::thread{std::bind(&ArmNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteKey> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteKey::Feedback>();
        auto result = std::make_shared<ExecuteKey::Result>();

        last_command_source_ = "action:/arm_ik/execute_key";
        last_action_key_ = goal->key_label;

        feedback->stage = "ik";
        feedback->progress = 0.5f;
        goal_handle->publish_feedback(feedback);

        gx_ = goal->x; gy_ = goal->y; gz_ = goal->z;
        groll_ = goal->roll; gpitch_ = goal->pitch;
        const bool ok = runIKAndPublish(goal->x, goal->y, goal->z, goal->roll, goal->pitch, publish_on_action_);

        if (!ok) {
            result->success = false;
            result->message = "IK failed for requested goal";
            last_ik_ok_ = false;
            last_ik_message_ = "ik_failed_action_goal";
            last_action_result_ = result->message;
            goal_handle->abort(result);
            return;
        }

        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Cancelled";
            last_action_result_ = "cancelled";
            goal_handle->canceled(result);
            return;
        }

        // Dry-run: no joints were published, so there's nothing to wait for.
        if (!publish_on_action_) {
            feedback->stage = "done";
            feedback->progress = 1.0f;
            goal_handle->publish_feedback(feedback);
            result->success = true;
            result->message = "IK solved (dry-run, no publish)";
            last_action_result_ = result->message;
            goal_handle->succeed(result);
            return;
        }

        // Wait for physical arrival before declaring success. The interp loop sets
        // at_target_ true once last_published_deg_ converges and (when feedback is fresh)
        // joint_state_deg_ agrees within kArrivalFeedbackTolDeg.
        constexpr auto kArrivalTimeout = std::chrono::milliseconds(8000);
        constexpr auto kPollPeriod = std::chrono::milliseconds(50);
        constexpr auto kFeedbackPeriod = std::chrono::milliseconds(200);

        const auto wait_start = std::chrono::steady_clock::now();
        auto next_feedback = wait_start;
        bool arrived = false;
        bool cancelled = false;

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) { cancelled = true; break; }
            if (at_target_.load(std::memory_order_acquire)) { arrived = true; break; }

            const auto now = std::chrono::steady_clock::now();
            if (now - wait_start >= kArrivalTimeout) break;

            if (now >= next_feedback) {
                const double elapsed_s = std::chrono::duration<double>(now - wait_start).count();
                const double timeout_s = std::chrono::duration<double>(kArrivalTimeout).count();
                const double frac = std::min(1.0, elapsed_s / timeout_s);
                feedback->stage = "moving";
                feedback->progress = 0.5f + 0.45f * static_cast<float>(frac);
                goal_handle->publish_feedback(feedback);
                next_feedback = now + kFeedbackPeriod;
            }

            std::this_thread::sleep_for(kPollPeriod);
        }

        if (cancelled) {
            result->success = false;
            result->message = "Cancelled";
            last_action_result_ = "cancelled";
            goal_handle->canceled(result);
            return;
        }

        if (!arrived) {
            result->success = false;
            result->message = "Arm did not reach target within timeout";
            last_ik_message_ = "arrival_timeout";
            last_action_result_ = result->message;
            goal_handle->abort(result);
            return;
        }

        feedback->stage = "done";
        feedback->progress = 1.0f;
        goal_handle->publish_feedback(feedback);

        result->success = true;
        result->message = "Arm reached target";
        last_action_result_ = result->message;
        goal_handle->succeed(result);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q3_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q4_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_q5_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr at_target_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_predefined_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp_action::Server<ExecuteKey>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr debug_timer_;
    rclcpp::TimerBase::SharedPtr interp_timer_;
    bool publish_on_action_{false};
    std::string last_command_source_{"startup"};
    std::string last_predefined_{"none"};
    std::string last_action_key_{""};
    std::string last_action_result_{"none"};
    bool last_ik_ok_{false};
    std::string last_ik_message_{"none"};
    double last_command_time_sec_{0.0};

    // Rate-limiter / interpolator state. All accessed under state_mutex_.
    std::mutex state_mutex_;
    std::array<double, 5> last_published_deg_{0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 5> target_deg_{0.0, 0.0, 0.0, 0.0, 0.0};
    bool target_valid_{false};
    double max_step_deg_per_tick_{1.5};

    // Arrival latch. True at startup (nothing pending). Cleared by setTarget(); set by
    // tickInterpolator() once commanded pose has converged and joint feedback agrees
    // (or feedback is unavailable/stale). Used by the action thread and exposed on
    // /arm_ik/at_target for the typing_coordinator's RETURNING_BASE phase.
    std::atomic<bool> at_target_{true};

    // Joint-state feedback state.
    std::array<double, 5> joint_state_deg_{0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<bool, 5> joint_state_index_valid_{false, false, false, false, false};
    bool joint_state_valid_{false};
    double last_joint_state_time_{0.0};
    std::vector<std::string> joint_state_names_;
    bool joint_state_in_degrees_{false};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}
