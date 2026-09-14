# 환자 Depth 접근

첨부 RealSense/YOLO 코드의 **10초 측정 후 이동 방식**은 [BATCH_RUN.md](BATCH_RUN.md)를 따른다. 아래는 기존 연속 Depth 방식이다.

Nav2 액션 성공 → odom 정지 확인(0.5초) → 새 환자 검출 5회 → 저속 접근 → 1m ±5cm 및 정지 확인 → DONE.
거리 기준은 **base_link 원점에서 환자 ROI 대표점까지 수평 거리**이다. 로봇 앞면과 신체 사이 간격이 아니다.
기존 src/params_setting.json의 0.70m 어깨 정렬 기능과 독립된 단계이다.

## 빌드

```bash
cd /home/reboot/cpr_auto_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select patient_approach rear_ackermann_controller --symlink-install
source install/setup.bash
```

## 모터 없이 실행하는 ROS 2D 시뮬레이션

```bash
cd /home/reboot/cpr_auto_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=164 ROS_LOG_DIR=/tmp/patient_approach_ros_logs \
  ros2 run patient_approach ros_simulation.py --output-dir /tmp/patient_approach_sim
cat /tmp/patient_approach_sim/results.json
```

전용 DDS domain을 사용하며 실제 모터 드라이버를 실행하지 않는다. 이 domain을 다른 장비에 사용하지 않는다.
실제 approach_manager, depth_patient_point, 기존 C++ rear_ackermann_node를 실행한다.
모의 NavigateToPose 액션 서버가 0.25m까지 주행시키고 성공을 반환한다.
모의 카메라는 초기 (1.6, 0.2)m 환자에 대한 aligned depth/ROI/CameraInfo 및 optical TF를 발행한다.
기존 C++ 노드가 발행한 좌우 바퀴 각속도를 2D 운동학으로 적분하여 odom과 다음 Depth를 만든다.
Nav2 성공 후에도 Nav2 0 속도를 계속 보내므로 접근 제어권 충돌 여부도 검증한다.

이는 **Gazebo, 실제 Nav2 planner/controller, RGB 사람 검출, 물리적 제동/충돌 검증이 아니다**.
결과 JSON, 노드 로그, 전체 궤적 CSV가 output-dir에 저장된다.

## 기존 시스템에 연결

```text
최초 PoseStamped /mission_goal -> approach_manager -> /navigate_to_pose 액션
Nav2 최종 속도 /nav2/cmd_vel ----------------------> approach_manager
환자 ROI + aligned Depth + CameraInfo -> depth_patient_point -> /patient/point
실제 /odom + base_link←camera optical TF ----------> approach_manager
장애물 감시 /patient_approach/path_clear -----------> approach_manager
approach_manager -> /patient_approach/cmd_vel -> 최종 충돌 감시 -> /cmd_vel
/cmd_vel -> rear_ackermann_node -> 바퀴 각속도 -> 기존 모터 드라이버
```

1. 기존 Nav2 최종 속도 출력이 `/nav2/cmd_vel`로 가게 설정한다. controller/smoother/collision_monitor를 쓰는 경우 중간 연결을 유지하고 최종 출력만 분리한다. 기존 Nav2의 `/cmd_vel` 직접 출력을 그대로 둔 채 연결하면 안 된다.
2. 최종 충돌 감시의 입력은 `/patient_approach/cmd_vel`, 출력은 기존 구동부 `/cmd_vel`로 설정한다. 단일 노드에서 Nav2/접근 속도를 선택하므로 접근 노드는 `/alignment_cmd`를 사용하지 않는다.
3. 기존 실제 odom과 정확한 카메라 optical TF를 사용한다. 시뮬레이션의 카메라 위치/자세를 실제 카메라에 복사하지 않는다.
4. 환자 검출기가 `/patient/roi` (`geometry_msgs/PolygonStamped`)를 발행하도록 어댑터를 연결한다. 두 점의 x/y는 같은 환자 몸통 ROI의 좌상단/우하단 **픽셀 좌표**, z는 0이다. stamp는 원본 영상 촬영 시각, frame_id는 aligned Depth optical frame이다. RGB와 Depth는 정렬·보정되어야 한다. ROI는 배경/바닥을 최대한 제외하고 동일 환자를 추적해야 한다. 이 패키지에 사람 검출 모델은 포함되지 않는다.
5. 실제 장애물 감시기가 경로가 비어 있을 때만 `/patient_approach/path_clear` (`std_msgs/Bool`)에 true를 10Hz 이상 보낸다. false 또는 0.3초 이상 끊김은 접근을 정지시킨다. 실제 장비에서 고정 true 발행으로 대체하지 않는다. 이 신호는 최종 충돌 감시나 비상정지를 대체하지 않는다.

```bash
ros2 launch patient_approach approach.launch.py \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  roi_topic:=/patient/roi odom_topic:=/odom \
  nav_cmd_topic:=/nav2/cmd_vel \
  output_cmd_topic:=/patient_approach/cmd_vel target_distance:=1.0
```

토픽명은 예시이다. 실제 카메라의 정렬 영상과 대응 CameraInfo를 선택한다.
Depth 16UC1은 mm, 32FC1은 m로 처리한다. ROI와 Depth 시각 차이는 최대 50ms이다.
base_link 원점에서 목표 거리까지 계산하며, 실물 앞면 여유 거리로 사용하려면 로봇 외형/환자 표면을 반영한 별도 거리 정의가 필요하다.

```bash
# x/y는 기존에 성공하던 목표 좌표로 바꾼다. 이 명령은 실제 연결 시 주행을 시작한다.
ros2 topic pub --once /mission_goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'
ros2 topic echo /patient_approach/state
ros2 topic echo /patient/point
ros2 service call /patient_approach/stop std_srvs/srv/Trigger '{}'
```

최초 goal은 `/goal_pose` 대신 `/mission_goal`로 보낸다. RViz Nav2 Goal 버튼은 manager를 우회할 수 있으므로 위 토픽을 사용한다.
Nav2 실행 자체는 기존 launch를 유지한다. 이 launch는 카메라, Nav2, 충돌 감시, 실제 모터 드라이버를 시작하지 않는다.
FAILED는 자동 재출발하지 않으며 원인 해결 후 새 mission_goal이 필요하다.
실제 드라이버의 command_timeout watchdog은 유지한다. manager 프로세스가 죽을 때는 하위 계층이 stale 속도를 정지시켜야 한다.
