#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <typing_interfaces/action/execute_key.hpp>
#include <cmath>
#include <algorithm>
#include <functional>
#include <string>
#include <thread>
#include <memory>
#include <sstream>
#include <iomanip>
#include <chrono>

class ArmNode : public rclcpp::Node {
    public:
        using ExecuteKey = typing_interfaces::action::ExecuteKey;
        using GoalHandleExecuteKey = rclcpp_action::ServerGoalHandle<ExecuteKey>;

        ArmNode() : Node("arm_node") {
            pub_q1_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint1", 1);
            pub_q2_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint2", 1);
            pub_q3_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint3", 1);
            pub_q4_ = this->create_publisher<std_msgs::msg::Float64>("arm_teleop/joint4", 1);
            pub_q5_ = this->create_publisher<std_msgs::msg::Int32>("arm_teleop/joint5", 1);
            debug_status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_ik/debug_status", 10);

            this->declare_parameter("publish_on_action", false);
            publish_on_action_ = this->get_parameter("publish_on_action").as_bool();

            sub_goal_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/goal", 10,
                std::bind(&ArmNode::onGoal, this, std::placeholders::_1)
            );

            sub_predefined_ = this->create_subscription<std_msgs::msg::String>(
                "/predefined", 10,
                std::bind(&ArmNode::onPredefined, this, std::placeholders::_1)
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

            RCLCPP_INFO(this->get_logger(), "arm_node ready. Action publish_on_action=%s", publish_on_action_ ? "true" : "false");
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

        bool inverseKinematics(double x, double y, double z,
                               double rollRad, double pitchRad,
                               double &q1d, double &q2d, double &q3d, double &q4d, double &q5d) {

            const double l1 = 0.1;
            const double l2 = 0.43;
            const double l3 = 0.43;
            const double l4 = 0.213;

            const double q1 = std::atan2(y, x);
            const double q5 = rollRad;

            const double a = std::sqrt(x*x + y*y) - l4 * std::cos(pitchRad);
            const double b = z - (l4 * std::sin(pitchRad)) - l1;

            double d = (a*a + b*b - l2*l2 - l3*l3) / (2.0 * l2 * l3);
            d = std::max(-1.0, std::min(1.0, d));

            const double q3 = -std::atan2(std::sqrt(1.0 - d*d), d);
            const double q2 = std::atan2(b, a) - std::atan2(l3 * std::sin(q3), l2 + l3 * std::cos(q3));
            const double q4 = pitchRad - q2 - q3;

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

    void publishJoints(double q1, double q2, double q3, double q4, double q5Deg) {
        std_msgs::msg::Float64 m;
        m.data = q1; pub_q1_->publish(m);
        m.data = q2; pub_q2_->publish(m);
        m.data = q3; pub_q3_->publish(m);
        m.data = q4; pub_q4_->publish(m);

        std_msgs::msg::Int32 s;
        s.data = myMap(-90.0, 90.0, 88, 268, q5Deg);
        pub_q5_->publish(s);
    }

    bool runIKAndPublish(double x, double y, double z, double rollDeg, double pitchDeg, bool publish_commands = true) {
        const double rollRad = rollDeg * M_PI / 180.0;
        const double pitchRad = pitchDeg * M_PI / 180.0;

        double q1, q2, q3, q4, q5;
        if (!inverseKinematics(x, y, z, rollRad, pitchRad, q1, q2, q3, q4, q5)) return false;

        if (publish_commands) {
            publishJoints(q1, q2, q3, q4, q5);
        }

        last_ik_ok_ = true;
        last_ik_message_ = publish_commands ? "ik_ok_published" : "ik_ok_dry_run";
        last_command_time_sec_ = this->get_clock()->now().seconds();
        return true;
    }

    void publishDebugStatus() {
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
           << "\"keyboard_home_set\":" << (kb_home_set_ ? "true" : "false")
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

        const bool ok = runIKAndPublish(goal->x, goal->y, goal->z, goal->roll, goal->pitch, publish_on_action_);

        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Cancelled";
            last_action_result_ = "cancelled";
            goal_handle->canceled(result);
            return;
        }

        feedback->stage = "done";
        feedback->progress = 1.0f;
        goal_handle->publish_feedback(feedback);

        if (ok) {
            result->success = true;
            result->message = publish_on_action_ ? "IK solved and command published" : "IK solved (dry-run, no publish)";
            last_action_result_ = result->message;
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "IK failed for requested goal";
            last_ik_ok_ = false;
            last_ik_message_ = "ik_failed_action_goal";
            last_action_result_ = result->message;
            goal_handle->abort(result);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q3_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_q4_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_q5_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_status_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_predefined_;
    rclcpp_action::Server<ExecuteKey>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr debug_timer_;
    bool publish_on_action_{false};
    std::string last_command_source_{"startup"};
    std::string last_predefined_{"none"};
    std::string last_action_key_{""};
    std::string last_action_result_{"none"};
    bool last_ik_ok_{false};
    std::string last_ik_message_{"none"};
    double last_command_time_sec_{0.0};
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}