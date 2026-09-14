# `body` TF 기준 실물 통합 실행

## 좌표계와 속도 경로

```text
map -> camera_init -> body -> camera_link -> camera_color_optical_frame
        FAST-LIO      static camera TF

Nav2 /nav2/cmd_vel
 -> patient_approach_manager
 -> /patient_approach/raw_cmd_vel
 -> collision_monitor (MID-360 cloud)
 -> /cmd_vel
 -> rear_ackermann_node
 -> ZLAC8015D driver
```

FAST-LIO의 `/Odometry`는 `header.frame_id=camera_init`, `child_frame_id=body`여야 한다.
이 구성에서는 `hardware_drive.launch.py`의 wheel odometry를 끈다. 휠 odometry까지
켜면 기존 설정의 `odom -> base_link`가 추가되어 서로 다른 base/odom 체계를 동시에
운영하게 된다.

## 빌드

```bash
cd /home/reboot/cpr_auto_ws
source /home/reboot/anaconda3/etc/profile.d/conda.sh
conda activate cart
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select patient_approach
source install/setup.bash
```

`cart` 환경에서 `cv_bridge`가 `_ARRAY_API` 오류를 내면 NumPy 1.x가 필요하다.

```bash
conda install "numpy<2"
```

## 실행 전 확인

MID-360 주소는 현재 driver 설정대로 host `192.168.1.5`, sensor `192.168.1.126`이다.
모터 USB 장치가 존재하는지 확인한다.

```bash
ip -4 addr
ls -l /dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BC0489ABCD-if00
```

기존에 따로 실행한 Livox, FAST-LIO, Nav2, 카메라 노드, ZLAC 노드와 static TF를 모두
종료한 다음 통합 launch 하나만 실행한다.

```bash
cd /home/reboot/cpr_auto_ws
source /home/reboot/anaconda3/etc/profile.d/conda.sh
conda activate cart
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch patient_approach body_hardware.launch.py \
  model_path:=/home/reboot/yolo11n.pt \
  class_id:=41 \
  target_distance:=0.5 \
  show_preview:=true \
  set_initial_pose:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 \
  initial_pose_qz:=0.0 initial_pose_qw:=1.0
```

기본 지도는 `map/fastlio_map.pcd`와
`map/fastlio_map_2d_clean_p2/fastlio_map_2d_clean_p2.yaml`이다. 다른 지도라면
`pcd_map:=... map_yaml:=...`를 추가한다.

## 움직이기 전에 확인

다른 터미널에서 같은 환경을 source한 후 다음 값들을 확인한다.

```bash
ros2 topic echo /Odometry --once
ros2 topic echo /cloud_registered_body --once --field header.frame_id
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_ros tf2_echo body camera_color_optical_frame
ros2 topic echo /patient_approach/path_clear
ros2 lifecycle get /collision_monitor
ros2 topic info /cmd_vel --verbose
```

필수 결과는 cloud frame `body`, collision monitor `active`, 장애물이 없을 때
`path_clear: true`이다. `/cmd_vel` publisher는 collision monitor 하나여야 한다.

현재 카메라 TF 기본값은 기존 명령에서 사용한 `body -> camera_link`의
`x=-0.5 m, z=0.6 m, pitch=-40 deg`이다. 이 값이 실측 장착 위치와 다르면 launch
파일의 static transform 값을 먼저 수정한다. 충돌 정지 영역도 `body` 원점 기준
`x=-0.65..0.92 m`, `y=-0.36..0.36 m`이므로 실제 차체 외곽보다 작으면 안 된다.

## 임무 실행과 정지

아래 좌표를 기존에 성공했던 목적지로 바꾸면 실제 주행을 시작한다.

```bash
ros2 topic pub --once /mission_goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'
```

Nav2 성공 후 정지 확인, class 41(cup) 10초 측정, 추가 접근 순으로 자동 진행한다.

```bash
ros2 topic echo /patient_approach/state
ros2 topic echo /patient_distance
ros2 topic echo /patient_move_distance
```

즉시 소프트웨어 정지는 다음 서비스로 요청한다. 터미널의 통합 launch는 `Ctrl+C`로
종료한다.

```bash
ros2 service call /patient_approach/stop std_srvs/srv/Trigger '{}'
```

기존 Nav2가 목적지 성공 후 `/goal_complete=true`를 발행하는 구성도 사용할 수 있다.
접근 관리자는 정지된 `/Odometry`를 확인한 뒤 `MEASURING`, `MOVING` 순서로 전환한다.
통합 관리자가 Nav2까지 담당하게 하려면 `/mission_goal`을 사용한다. 측정거리가
0.5 m 이하이면 `/patient_move_distance`는 0이며 모터가 추가로 움직이지 않는다.
