# 첨부한 카메라 코드와 추가 접근 코드 실행

> MID-360, FAST-LIO, Nav2, ZLAC 모터를 `body` TF로 함께 실행할 때는
> `HARDWARE_BODY_RUN.md`와 `body_hardware.launch.py`를 사용한다. 아래 3~4절은
> 각 노드를 따로 연결할 때의 설명이다.

파일은 이미 아래 위치에 배치했다. 별도로 복사할 필요 없다.

| 파일 | 역할 |
|---|---|
| `scripts/patient_depth_batch.py` | 첨부 RealSense/YOLO 중심 ROI Depth 중앙값 로직을 활용한 10초 측정 노드 |
| `scripts/batch_approach_manager.py` | 기존 접근 관리 코드의 Nav2 처리 재사용, 측정 후 odom으로 목표까지 이동 |
| `scripts/approach_manager.py` | 위 관리 노드가 import하는 공통 Nav2/정지/속도 선택 처리 |
| `launch/batch_approach.launch.py` | 두 노드를 함께 실행 |
| `test/ros_simulation.py` | 합성 Depth와 모의 Nav2, 기존 C++ 바퀴 변환기를 연결하는 시험 |

작업공간 루트의 `camera_destance.py` 및 원본 첨부 파일은 수정하지 않았다. 실행할 코드는 이 패키지의 수정본이다.

## 1. 빌드

```bash
cd /home/reboot/cpr_auto_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select patient_approach rear_ackermann_controller --symlink-install
source install/setup.bash
```

## 2. 카메라/모터 없이 두 코드 연결 시험

```bash
cd /home/reboot/cpr_auto_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=165 \
ROS_LOG_DIR=/tmp/patient_batch_ros_logs \
ros2 run patient_approach ros_simulation.py --batch \
  --output-dir /tmp/patient_batch_sim
```

전용 domain 165를 다른 장비에서 쓰지 않는다. 위 명령 하나가 필요한 노드와 goal을 자동 실행한다. 실제 모터 드라이버나 RealSense를 실행하지 않는다. 전체 7개 시험에는 약 2분이 걸린다.

```text
모의 Nav2 0.25m 주행 성공
 -> 실제 batch_approach_manager: odom 정지 확인
 -> /goal_complete = true
 -> 실제 patient_depth_batch: 10초간 정지 측정
 -> /patient/measurement + /patient_distance + /patient_move_distance
 -> 실제 batch_approach_manager: odom 기반 추가 이동
 -> 기존 rear_ackermann_node의 바퀴 각속도 출력
 -> 모의 2D 차체 위치/odom/Depth 갱신
 -> DONE 및 좌우 바퀴 명령 0
```

카메라 대신 기존 `depth_patient_point.py`에 합성 aligned Depth/ROI/CameraInfo를 입력하고, optical TF를 거쳐 측정 노드에 전달한다. **실제 YOLO 추론, RealSense SDK, 실제 Nav2 경로 계획, Gazebo 또는 실물 제동을 시험하는 것은 아니다.**

시험: 정상 float Depth, 정상 mm Depth, 이동 중 Depth 중단(일회성 측정이므로 odom으로 완료), 장애물, odom 중단, 10초간 미검출, Nav2 거절. 측정 도중 반복 true를 보내도 10초 타이머가 초기화되지 않고 결과가 1회 발행되는지도 검증한다.

결과: output-dir의 `results.json`, `trajectory.csv`, 각 노드 `.log`.

## 3. 실제 RealSense 입력으로 실행

먼저 이 조건이 필요하다.

- 실행 Python 환경에 `pyrealsense2`, `ultralytics`, `numpy`와 ROS 2 Humble 라이브러리가 있어야 한다.
- 내려받기가 완료된 YOLO 가중치 `.pt` 파일이 필요하다. `.part` 파일은 사용할 수 없다. `model_path`에 실제 경로를 지정한다.
- 이 노드는 RealSense 장치를 SDK로 직접 연다. 같은 장치를 사용하는 `camera_destance.py` 또는 `realsense2_camera`는 동시에 실행하지 않는다.
- 실제 카메라 장착 위치/자세의 `base_link <- camera_color_optical_frame` TF와 실제 `/odom`이 필요하다. 시뮬레이션의 TF를 실물에 복사하지 않는다.
- `/patient_approach/path_clear` (`std_msgs/Bool`)는 실제 경로 감시기가 10Hz 이상 발행한다. false/0.3초 timeout은 이동을 중단한다.
- 최종 충돌 감시와 구동부 command timeout을 유지한다. 경로 신호에 고정 true를 발행해 대체하지 않는다.

```bash
cd /home/reboot/cpr_auto_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch patient_approach batch_approach.launch.py \
  input_mode:=realsense \
  model_path:=/실제/경로/yolo11n.pt \
  class_id:=0 \
  camera_frame:=camera_color_optical_frame \
  odom_topic:=/odom \
  nav_cmd_topic:=/nav2/cmd_vel \
  output_cmd_topic:=/patient_approach/cmd_vel \
  measure_duration:=10.0 target_distance:=1.0
```

`class_id:=0`은 COCO 사람, 컵으로 시험하려면 `class_id:=41`이다. `model_path`와 카메라 frame 이름은 실물 값으로 바꾼다. 자동 가중치 다운로드는 하지 않는다. GUI 창은 열지 않는다.

## 4. Nav2 및 구동부 연결

```text
최초 /mission_goal (PoseStamped)
  -> batch_approach_manager -> /navigate_to_pose (기존 Nav2)
Nav2 최종 속도 /nav2/cmd_vel
  -> batch_approach_manager
  -> /patient_approach/cmd_vel
  -> 최종 충돌 감시
  -> /cmd_vel -> 기존 rear_ackermann_node -> 실제 드라이버
```

기존 Nav2 최종 출력이 `/cmd_vel`로 직접 가지 않도록 `/nav2/cmd_vel`로 분리해야 한다. Humble velocity_smoother를 쓰면 해당 launch의 최종 출력 remapping `('cmd_vel_smoothed', 'cmd_vel')`을 `('cmd_vel_smoothed', '/nav2/cmd_vel')`로 바꾼다. behavior_server의 속도도 manager 이전 경로에 연결해야 하며 `/cmd_vel`로 우회시키지 않는다. 구체적인 기존 launch가 제공되지 않았으므로 사용자 Nav2 launch는 이 패키지에서 자동 수정하지 않는다.

현재 `approach.launch.py`의 연속 Depth 제어와 `batch_approach.launch.py`는 함께 실행하지 않는다. 두 manager를 동시에 실행하면 안 된다.

최초 goal은 `/goal_pose` 대신 `/mission_goal`로 보낸다. 아래 x/y를 기존에 성공하던 좌표로 교체한다. **실물 연결 후 이 명령은 주행을 시작한다.**

```bash
ros2 topic pub --once /mission_goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'
```

관리 노드가 Nav2 성공과 실제 정지를 확인한 뒤 `/goal_complete`를 자동 발행한다. 외부에서 따로 true를 보낼 필요 없다. 외부 goal_complete만 발행하면 카메라 측정만 시작하며, manager의 mission 흐름을 대신하지 않는다.

```bash
ros2 topic echo /patient_approach/state
ros2 topic echo /patient_distance
ros2 topic echo /patient_move_distance
ros2 service call /patient_approach/stop std_srvs/srv/Trigger '{}'
```

## 거리와 제어 의미

- `/patient_distance` (`Float32`): 원본과 달리 **TF 변환 후 base_link에서 환자 대표점까지 수평 거리(m)**.
- `/patient_move_distance` (`Float32`): 위 거리에서 목표 거리 1m를 뺀 추가 이동 거리. 관찰용이다.
- `/patient/measurement` (`PointStamped`): 촬영 시각 및 base_link 좌표가 포함된 제어용 결과. manager는 방향과 오래된 결과를 구별하기 위해 이 토픽을 사용한다.
- 목표점까지 odom 위치 오차가 2cm 이내이고 실제 속도가 정지 상태면 DONE.
- 1m는 로봇 앞면에서의 간격이 아니다. 환자 대표점과 base_link 원점 기준이다.
- 10초 종료 시 유효 샘플 10개 이상, 마지막 샘플 0.3초 이내, 샘플 분포 안정성을 확인한다. 미검출이어도 종료하며 FAILED 처리한다. 이는 10초 연속 검출 보장은 아니다.
- 동일 측정 세션의 반복 true는 무시한다. 다음 mission에서 false로 재무장한다.
- 측정 후에는 저장한 목표를 odom으로 따라가며 환자를 다시 추적하지 않는다. 환자가 움직이는 상황에서 거리 유지 기능으로 사용하면 안 된다.
