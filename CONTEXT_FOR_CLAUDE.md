# Executive Context (for Claude Pro)

Project: autonomous_typing_zed (ROS 2 Humble)

Goal
- Detect ArUco markers from a ZED camera, localize a keyboard, and command a robot arm to press keys autonomously.
- Primary target is Jetson Orin Nano with a ZED2i camera.

Core packages and roles
- zed_aruco
  - zed_aruco_node: ZED image ingest + ArUco detection + keyboard pose + target key tracking.
  - typing_coordinator: converts vision targets into arm goals, handles safety gates, servo mode, and action flow.
  - calibration_probe (optional): TF-based calibration helper.
  - fake_vision_publisher / fake_execute_key_server: simulation without hardware.
- typing_interfaces
  - ExecuteKey.action used for arm commands.
- arm_ik
  - arm_node implements ExecuteKey action server at /arm_ik/execute_key and keeps legacy /goal, /predefined topics.
- aruco_py
  - aruco_node: standalone ArUco detector for non-ZED sources.

Key ROS topics/actions
- From vision: keyboard/target_key, keyboard/target_point_px, keyboard/target_valid, keyboard/target_confidence, keyboard/state
- To arm: /arm_ik/execute_key (ExecuteKey action)
- Safety: keyboard/contact_pressed, keyboard/emergency_stop, keyboard/transform_valid

Recommended entry points
- Runtime quick start: RUNTIME_COMMANDS.md
- ZED + coordinator launch: src/zed_aruco/launch/zed_typing_integration.launch.py
- Node logic: src/zed_aruco/zed_aruco/zed_aruco_node.py, src/zed_aruco/zed_aruco/typing_coordinator.py

Current behavior notes
- Motion is gated (motion_enabled=false by default) and action publish can be dry-run.
- Servo mode provides closed-loop XY alignment + press/retract with contact input.

What to analyze
- Vision -> typing pipeline robustness, topic interfaces, safety gates, and configuration parameters.
- Mapping from image-space targets to arm-space goals (TF mode vs heuristic mode).


# Contexto del proyecto - ROS2 Humble + Gazebo + Brazo robótico

## Setup actual
- Ubuntu 24.04 host, ROS2 Humble en Docker (imagen: ros2_humble_gazebo)
- Workspace: ~/ros2_ws
- Docker alias: ros2docker (entra al contenedor), ros2shell (segunda terminal)
- Build alias: cb (build completo), cbp <paquete> (build selectivo)

## Paquetes en ~/ros2_ws/src/
- mi_rover_description: URDF del rover completo (chasis + 4 ruedas + brazo 5DOF)
- autonomous_typing_zed/src/arm_ik: nodo C++ que recibe (x,y,z,roll,pitch), calcula IK y publica ángulos en grados a arm_teleop/joint1..4 (Float64) y arm_teleop/joint5 (Int32)
- autonomous_typing_zed/src/typing_interfaces: action ExecuteKey usado por arm_node
- autonomous_typing_zed/src/zed_aruco: pipeline de visión (ZED + ArUco + typing_coordinator)

## Estado del brazo en Gazebo
- JointTrajectoryController corriendo en /brazo_controller/follow_joint_trajectory
- Joints: Joint_1 a Joint_5, tipo revolute, límites reales del brazo físico
- ros2_control configurado en brazo.urdf con gz_ros2_control/GazeboSimSystem
- controllers.yaml en mi_rover_description/config/

## Bridge temporal
- mi_rover_description/src/arm_bridge.py: escucha arm_teleop/joint1..4, convierte grados a radianes, manda al JointTrajectoryController
- Este bridge es temporal, el objetivo es conectar arm_node directamente

## Flujo objetivo
/goal (x,y,z,roll,pitch) → arm_node (IK) → arm_teleop/joint1..4 (grados) → arm_bridge.py (grados→radianes) → JointTrajectoryController → Gazebo

## Siguiente paso
Compilar arm_ik + typing_interfaces en el workspace Docker y verificar que arm_node se conecta correctamente con el bridge para mover el brazo en Gazebo dado un punto objetivo.

## Goal final
Simular el pipeline completo de tecleo autónomo:
zed_aruco detecta teclado → typing_coordinator manda goals → arm_node calcula IK → brazo se mueve en Gazebo
Sin hardware real: usar fake_vision_publisher.py y fake_execute_key_server.py del paquete zed_aruco

Resumen Técnico — Sesión de Depuración del Brazo Autónomo                                                                                                                              
                                                                                                                                                                                         
  Contexto del Proyecto                                                                                                                                                                  
                                                                                                                                                                                         
  Sistema ROS 2 Humble para tipeo autónomo en teclado usando brazo robótico de 4 eslabones (l1=0.1m, l2=0.43m, l3=0.43m, l4=0.213m), cámara ZED y marcadores ArUco, simulado en Ignition 
  Gazebo Fortress.                                                                                                                                                                       
                                                                                                                                                                                         
  ---                                                                                                                                                                                    
  1. Diagnóstico Inicial — debug_status mostraba valores obsoletos
                                                                                                                                                                                         
  Problema        
                                                                                                                                                                                         
  El tópico /arm_ik/debug_status mostraba siempre los defaults (gx_=0.15, gz_=0.35) sin importar qué objetivo se enviara.                                                                
  
  Causa raíz                                                                                                                                                                             
                  
  El handler de la acción ExecuteKey en arm_node llamaba a runIKAndPublish() pero nunca actualizaba las variables miembro gx_, gy_, gz_, groll_, gpitch_ que alimentan debug_status.     
  
  Fix aplicado en src/arm_ik/src/main.cpp (execute() ~línea 306)                                                                                                                         
                  
  gx_ = goal->x; gy_ = goal->y; gz_ = goal->z;                                                                                                                                           
  groll_ = goal->roll; gpitch_ = goal->pitch;                                                                                                                                            
                                                                                                                                                                                         
  Bonus                                                                                                                                                                                  
                                                                                                                                                                                         
  - ros2 topic echo truncaba el JSON. Solución: suscriptor Python directo a /arm_ik/debug_status.                                                                                        
  
  ---                                                                                                                                                                                    
  2. El brazo no se movía — arm_bridge ausente
                                                                                                                                                                                         
  Síntoma
                                                                                                                                                                                         
  joint_states reportaba todos los joints en 0 a pesar de que arm_node publicaba a /arm_teleop/joint1..5.                                                                                
  
  Causa raíz                                                                                                                                                                             
                  
  Faltaba ejecutar arm_bridge.py (ubicado en ~/ros2_ws/src/mi_rover_description/src/arm_bridge.py), que es el puente crítico:                                                            
  - Suscribe a arm_teleop/joint1..5 (Float64, en grados)
  - Convierte a radianes                                                                                                                                                                 
  - Envía FollowJointTrajectory al controlador /brazo_controller/follow_joint_trajectory
  - 50 Hz, time_from_start = 1s                                                                                                                                                          
                                                                                                                                                                                         
  Fix                                                                                                                                                                                    
                                                                                                                                                                                         
  Se añadió Terminal 5 al checklist de bring-up:                                                                                                                                         
  python3 src/mi_rover_description/src/arm_bridge.py
                                                                                                                                                                                         
  ---                                                                                                                                                                                    
  3. Múltiples instancias del coordinador
                                                                                                                                                                                         
  Problema        

  Una corrida previa de typing_coordinator quedó viva y peleaba con la nueva instancia.                                                                                                  
  
  Fix                                                                                                                                                                                    
                  
  pkill -f typing_coordinator                                                                                                                                                            
  
  ---                                                                                                                                                                                    
  4. Coordinador con coordenadas incorrectas
                                                                                                                                                                                         
  Problema
                                                                                                                                                                                         
  El typing_coordinator arrancaba con defaults (base_x=0.25, target_z=0.12, target_pitch=-75) que no corresponden al setup actual.                                                       
  
  Fix                                                                                                                                                                                    
                  
  Relanzar con parámetros explícitos:
  ros2 run zed_aruco typing_coordinator --ros-args \
    -p base_x:=0.78 -p target_z:=-0.21 -p target_pitch:=0.0 \
    -p goal_cooldown_sec:=2.0                                                                                                                                                            
                                                                                                                                                                                         
  El goal_cooldown_sec:=2.0 resolvió el problema de "teclas cada 0.3s".                                                                                                                  
                                                                                                                                                                                         
  ▎ Nota pendiente: ros2 param set en runtime no funciona por falta de callback de parámetros — requiere parche en typing_coordinator.py.                                                
                                                                                                                                                                                         
  ---                                                                                                                                                                                    
  5. Reposicionamiento del teclado en el mundo
                                                                                                                                                                                         
  Decisión
                                                                                                                                                                                         
  Mover el teclado 15 cm más lejos del rover para evitar que el brazo se atorara y levantara el rover al chocar contra la parte superior del panel.                                      
  
  Cambio en ~/ros2_ws/src/mi_rover_description/worlds/simulation.sdf                                                                                                                     
                  
  - keyboard: pose 1.15 0 0.765 (antes 1.0 0 0.765)                                                                                                                                      
  - keyboard_stand: pose 1.15 0 0.3525 (antes 1.0 0 0.3525)
  - Comentario añadido: Front face at x=1.15 → arm-frame base_x=0.93                                                                                                                     
                                                                                                                                                                                         
  Base del brazo en mundo: (0.22, 0, 0.975) → frente del panel a 0.93 m en el frame del brazo.                                                                                           
                                                                                                                                                                                         
  ---                                                                                                                                                                                    
  6. Convención de signos de joints — Discrepancia URDF ↔ IK
                                                                                                                                                                                         
  Análisis del URDF (brazo.urdf)
                                                                                                                                                                                         
  ┌─────────┬────────────────────────┬──────────┬────────┐                                                                                                                               
  │  Joint  │       origin xyz       │   rpy    │  axis  │                                                                                                                               
  ├─────────┼────────────────────────┼──────────┼────────┤                                                                                                                               
  │ Joint_1 │ 0 0 0.03               │ 0 0 0    │ 0 0 -1 │
  ├─────────┼────────────────────────┼──────────┼────────┤
  │ Joint_2 │ 0.00625 0.03585 0.0742 │ -π/2 0 0 │ 0 0 1  │                                                                                                                               
  ├─────────┼────────────────────────┼──────────┼────────┤                                                                                                                               
  │ Joint_3 │ 0 -0.45045 -0.12605    │ -π 0 0   │ 0 0 -1 │                                                                                                                               
  ├─────────┼────────────────────────┼──────────┼────────┤                                                                                                                               
  │ Joint_4 │ 0.016 0.4295 -0.0771   │ π 0 0    │ 0 0 1  │
  ├─────────┼────────────────────────┼──────────┼────────┤                                                                                                                               
  │ Joint_5 │ 0 -0.2475 -0.0057      │ π/2 0 0  │ 0 0 1  │
  └─────────┴────────────────────────┴──────────┴────────┘                                                                                                                               
                  
  Decisión                                                                                                                                                                               
                  
  Negar q1 y q3 para alinearse con los ejes invertidos del URDF.                                                                                                                         
  
  q1d = -radToDeg(q1);                                                                                                                                                                   
  q3d = -radToDeg(q3);                                                                                                                                                                   
  
  ---                                                                                                                                                                                    
  7. Hallazgo crítico — La pose de descanso del URDF no coincide con la asunción del IK
                                                                                                                                                                                         
  Síntoma
                                                                                                                                                                                         
  Tras todas las correcciones anteriores, el brazo se levantaba vertical encima del teclado en lugar de extenderse horizontalmente hacia él. Imagen del usuario confirmó: "thinks the    
  keyboard its up there". joint_states confirmó que los joints alcanzaban los valores comandados — el bug está en la formulación del IK, no en la cadena de control.
                                                                                                                                                                                         
  Análisis        

  Calculando forward kinematics desde joint_states (q1=0.012, q2=-0.098, q3=0.770, q4=0.868) usando los rpy del URDF:                                                                    
  - El origen de Joint_3 en q2=q3=0 cae en (0.006, -0.090, 0.555) en el frame de la base del brazo.
  - Esto significa que el upper arm en reposo apunta hacia +Z (arriba), no hacia +X (adelante) como asume el IK analítico.                                                               
                                                                                                                          
  El supuesto "estado funcional" anterior con teclado en x=1.0 era en realidad el brazo estrellándose contra el tope del panel, no alcanzando teclas correctamente.                      
                                                                                                                                                                                         
  Fix aplicado (sin probar) en src/arm_ik/src/main.cpp líneas 95-103                                                                                                                     
                                                                                                                                                                                         
  // URDF rest pose has the upper arm pointing +Z (up); IK assumes it points +X (forward).                                                                                               
  // The 90° offset on q2 rotates the rest pose to match IK convention.                                                                                                                  
  q1d = -radToDeg(q1);                                                                                                                                                                   
  q2d = -radToDeg(q2) + 90.0;                                                                                                                                                            
  q3d = -radToDeg(q3);                                                                                                                                                                   
  q4d = -radToDeg(q4) - 90.0;
  q5d = radToDeg(q5);                                                                                                                                                                    
                                                                                                                                                                                         
  Riesgo conocido — Límites de joint
                                                                                                                                                                                         
  - A base_x=0.93: q4_pub ≈ −137.6° (dentro de ±150° ✓)                                                                                                                                  
  - A base_x=0.78: q4_pub ≈ −160° (fuera de rango ✗)
                                                                                                                                                                                         
  ---             
  Estado Actual y Próximos Pasos                                                                                                                                                         
                                                                                                                                                                                         
  Completado
                                                                                                                                                                                         
  - Fix de gx_/gy_/gz_ en handler de acción                                                                                                                                              
  - Documentación del flujo correcto con arm_bridge.py
  - Reubicación de teclado a x=1.15                                                                                                                                                      
  - Parámetros correctos del coordinador (base_x=0.78, target_z=−0.21, target_pitch=0.0, cooldown=2s)                                                                                    
  - Negación de q1 y q3 para alinear con axes URDF                                                                                                                                       
  - Edición del offset ±90° en q2/q4 (en código, sin compilar)                                                                                                                           
                                                                                                                                                                                         
  Pendiente — Acción inmediata cuando reanudemos                                                                                                                                         
                                                                                                                                                                                         
  1. Recompilar arm_ik:                                                                                                                                                                  
  cd ~/ros2_ws    
  colcon build --symlink-install --packages-select arm_ik                                                                                                                                
  source install/setup.bash
  2. Reiniciar Terminal 2 (arm_node) con el binario nuevo.                                                                                                                               
  3. Probar tipeo y observar si ahora el brazo desciende horizontalmente al teclado.                                                                                                     
  4. Si funciona: ajustar target_z empíricamente.                                                                                                                                        
  5. Si no funciona: derivar IK que considere los offsets xyz y rotaciones rpy del URDF link por link en lugar de aplicar offset post-hoc.                                               
                                                                                                                                                                                         
  Pendiente — Largo plazo                                                                                                                                                                
                                                                                                                                                                                         
  - Probar modo servo (state machine de typing_coordinator).                                                                                                                             
  - Añadir parameter callback al coordinador para soportar ros2 param set en runtime.
  - Validar que q4_pub no se salga de ±150° en todo el espacio de trabajo del teclado.                                                                                                   
                                                                                                                                                                                         
  ---                                                                                                                                                                                    
  Archivos Modificados                                                                                                                                                                   
                  
  ┌──────────────────────────────────────────────────────────┬──────────────────────────────────────────────────────────────────────────┐
  │                         Archivo                          │                                  Cambio                                  │
  ├──────────────────────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────┤
  │ src/arm_ik/src/main.cpp                                  │ Update de gx_…gpitch_ en execute(); negación q1/q3; offset ±90° en q2/q4 │
  ├──────────────────────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────┤
  │ ~/ros2_ws/src/mi_rover_description/worlds/simulation.sdf │ Teclado y stand movidos de x=1.0 a x=1.15                                │                                                
  └──────────────────────────────────────────────────────────┴──────────────────────────────────────────────────────────────────────────┘                                                
                                                                                                                                                                                         
  Hipótesis no verificada                                                                                                                                                                
                  
  El offset +90°/−90° en q2/q4 es la teoría de trabajo que compensaría la pose vertical de descanso del URDF. No ha sido probada. Si falla, la causa más probable es que los rpy no      
  triviales en cada joint (especialmente las rotaciones de π en Joint_3/Joint_4) introduzcan transformaciones que un offset constante no puede corregir, y habría que reescribir el IK
  con la cadena cinemática completa del URDF.

---

# Update — Resolution (2026-05-06)

## Bug found: the −90° on q4 was wrong

The "+90 on q2 / −90 on q4" hypothesis was half right. q2 needs +90° (URDF rest = arm along +Z, IK rest = arm along +X — an absolute rest-pose offset). q3 and q4 measure *relative to the previous link*, and that relative angle is zero at rest in **both** conventions, so they need only the sign flip — **no offset**.

Concretely, at `main.cpp:99-103` the correct mapping is:
```cpp
q1d = -radToDeg(q1);
q2d = -radToDeg(q2) + 90.0;
q3d = -radToDeg(q3);
q4d = -radToDeg(q4);   // NO -90
q5d = radToDeg(q5);
```

The spurious −90 on q4 caused two visible symptoms:
1. The gripper rotated to point straight up at the wrist position → "arm vertical above keyboard."
2. At base_x ≤ 0.78 the resulting q4_urdf exceeded ±150°, so the IK rejected the goal entirely and the arm stayed in URDF rest (vertical) → reinforced the "vertical above keyboard" appearance.

## Wrong assumption corrected: arm-base world z

The earlier note "Base del brazo en mundo: (0.22, 0, 0.975)" was wrong by ~57 cm. Empirically, in the current Gazebo SDF the keyboard top sits at arm-frame `z ≈ +0.35` (positive, *above* the arm base), not at `z = -0.21`. The arm base is much lower in the rover than that note implied.

## Working sim pose for autonomous-typing tests

In Gazebo simulation, the camera-on-gripper sees the keyboard at:
- `base_x = 0.7`
- `target_z = 0.35`
- `target_pitch = 0` (horizontal — gripper-tip just grazes the key tops)

**These values are simulation-only.** The simulation is a testbed for the typing pipeline (vision → coordinator → action → IK → motion), not a substitute for hardware calibration. Real-world bring-up will require fresh calibration of arm-base position, camera mount geometry, and keyboard placement.

## Joint-limit risk no longer applies

With q4 corrected, the previously reported "q4_pub ≈ −160° at base_x=0.78" risk goes away — at the working sim pose, q4_urdf is well within ±150° across the relevant workspace.

## Vertical-keyboard refactor (action mode)

Committed to vertical-only keyboards (per user direction). `typing_coordinator.pixel_to_arm_goal` now returns `(x, y, z)`:
- arm-x = `base_x` (fixed; panel face)
- arm-y = `base_y + dx_px * scale_y_per_px` (horizontal pixel offset)
- arm-z = `target_z + dy_px * scale_z_per_px` (vertical pixel offset)

Parameter `scale_x_per_px` removed; `scale_z_per_px` added. Both call sites (action-dispatch and servo-init paths) updated.

Servo state machine **not yet** adapted — still presses along arm-z. Must be reworked before `servo_mode_enabled:=true` is usable.

## End-to-end action mode verified in Gazebo

Calibrated sim values: see `SIM_CHECKPOINT.md` for the full bring-up + parameter set. Arm visits each "hola" letter at the correct (y, z) on the vertical panel. Outstanding: servo-mode adaptation for vertical (press direction along arm-x, XY correction → YZ correction).  