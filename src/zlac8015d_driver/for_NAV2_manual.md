## 코드 구조
/cmd_vel
  ↓
좌·우 바퀴 속도 계산
  ↓
속도/가속도 제한
  ↓
통신 연결·응답 확인
  ↓
Enable / Stop / Fault 처리
  ↓
명령 timeout 시 정지
  ↓
실제 모터


## 1. 전달할 파일

다음 세 항목을 상대 workspace의 `src/` 아래에 복사한다.

```text
src/rear_ackermann_controller/
src/zlac8015d_driver/
src/params_setting.json
```

`rear_ackermann_controller`는 표준 ROS `Twist`를 좌·우 바퀴 각속도로 바꾸는 계층이고, `zlac8015d_driver`는 그 결과를 RS485 Modbus RTU로 ZLAC8015D에 보낸다.

## 2. 반드시 유지할 ROS 인터페이스

Nav2 또는 다른 상위 제어 노드는 아래 한 토픽만 발행하면 된다.

```text
/cmd_vel    geometry_msgs/msg/Twist
```

사용하는 필드는 다음 두 개다.

```text
linear.x    전진(+)/후진(-) 선속도 [m/s]
angular.z   반시계(+)/시계(-) 회전 각속도 [rad/s]
```

`linear.y`, `linear.z`, `angular.x`, `angular.y`는 사용하지 않는다. `/cmd_vel` publisher를 추가로 만들 필요도 없고, ZLAC8015D driver가 `/cmd_vel`을 직접 구독하지도 않는다.

## 3. 명령 변환 경로

```text
Nav2
  └─ /cmd_vel (Twist)
       └─ rear_ackermann_controller
            ├─ /rear_left_wheel_speed_cmd  (Float64, wheel rad/s)
            └─ /rear_right_wheel_speed_cmd (Float64, wheel rad/s)
                 └─ zlac8015d_driver
                      └─ RS485 Modbus RTU → ZLAC8015D
```

기존 `rear_ackermann_controller`는 다음 식을 사용한다.

```text
left wheel speed  = linear.x - angular.z × rear_track / 2
right wheel speed = linear.x + angular.z × rear_track / 2
wheel rad/s       = wheel linear speed / wheel_radius
```

그 다음 ZLAC driver가 다음 식으로 실제 모터 명령 RPM을 만든다.

```text
motor RPM = wheel rad/s × 60 / (2π) × gear_ratio
```

따라서 `rear_track_m`, `wheel_radius_m`, `max_wheel_speed_mps`, `max_yaw_rate_rad_s`는 `params_setting.json`에서 한 곳만 관리한다. ZLAC YAML에 같은 차량 기하 값을 중복으로 넣지 않는다.

## 4. 설치와 빌드

```bash
sudo apt install libmodbus-dev
cd <사용자_workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select rear_ackermann_controller zlac8015d_driver
source install/setup.bash
```

## 5. 하드웨어 설정

빌드 전 또는 실행 전에 `src/zlac8015d_driver/config/zlac8015d.yaml`을 실제 장비에 맞춘다.

반드시 확인할 항목:

```yaml
serial_port: "/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BC0489ABCD-if00"  # USB-RS485의 실제 고정 경로
driver_id: 1                         # ZLAC8015D에 설정된 Modbus ID
gear_ratio: 1.0                      # 모터 축 RPM / 휠 축 RPM
left_motor_inverted: false           # 저속 단독 시험 뒤 확정
right_motor_inverted: false          # 저속 단독 시험 뒤 확정
max_motor_rpm: 10.0                  # 모터/기구의 안전한 한계 RPM, 최대 10으로 고정
```

포트는 `/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BC0489ABCD-if00`로 고정

`max_motor_rpm`은 ZLAC register의 절대 범위인 3000 RPM이 아니라 실제 모터와 감속기, 장착 기구가 안전하게 허용하는 값을 사용한다.

Nav2의 최대 `linear.x`와 `angular.z`도 `params_setting.json`의 `max_wheel_speed_mps`, `max_yaw_rate_rad_s`보다 크게 설정하지 않는 것을 권장한다. 기존 controller가 clamp하지만, Nav2 속도 제한과 차량 제한을 일치시키는 편이 안전하다.

## 6. 실제 하드웨어 실행

```bash
cd <사용자_workspace>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zlac8015d_driver hardware_drive.launch.py
```

이 launch는 다음 두 노드만 실행한다.

```text
/rear_ackermann_node
/zlac8015d_driver
```

실제 하드웨어 launch와 Gazebo의 `alignment_control.launch.py`, Gazebo bridge, Gazebo wheel controller는 동시에 실행하지 않는다.

Nav2를 별도 terminal에서 실행한다. Nav2가 `/cmd_vel`을 발행하면 위 드라이버가 자동으로 처리한다.


## 7. 상태 확인
| 목적 | Topic | 타입/경로 |
| Nav2 이동 명령 | `/cmd_vel` | `linear.x`, `angular.z` |
| 좌측 휠 명령 | `/rear_left_wheel_speed_cmd` | `data` (rad/s) |
| 우측 휠 명령 | `/rear_right_wheel_speed_cmd` | `data` (rad/s) |
| 좌측 실제 속도 | `/zlac8015d/left_actual_rpm` | `data` (RPM) |
| 우측 실제 속도 | `/zlac8015d/right_actual_rpm` | `data` (RPM) |
| 좌측 fault bitmask | `/zlac8015d/left_fault` | `data` |
| 우측 fault bitmask | `/zlac8015d/right_fault` | `data` |
| RS485 연결 상태 | `/zlac8015d/connected` | `data` |
| 드라이버 상태 | `/zlac8015d/state` | `data` |


