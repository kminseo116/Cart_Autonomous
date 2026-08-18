# ZLAC8015D 하드웨어 드라이버

이 패키지는 기존 `rear_ackermann_controller`가 발행하는 좌·우 바퀴 각속도 토픽을 받아 ZLAC8015D RS485 Modbus RTU 속도 명령으로 변환한다. `/cmd_vel` 또는 `/alignment_cmd`를 직접 구독하지 않으며, 차동 구동 역기구학도 다시 계산하지 않는다.

## 인터페이스

- 입력: `/rear_left_wheel_speed_cmd`, `/rear_right_wheel_speed_cmd` (`std_msgs/msg/Float64`, 바퀴 rad/s)
- 출력: `/zlac8015d/left_actual_rpm`, `/zlac8015d/right_actual_rpm` (`Float64`)
- 상태: `/zlac8015d/left_fault`, `/zlac8015d/right_fault` (`UInt16`), `/zlac8015d/connected` (`Bool`), `/zlac8015d/state` (`String`)
- 명시적 fault clear: `/zlac8015d/reset_fault` (`std_srvs/srv/Trigger`)

ZLAC8015D Ver3.1의 velocity mode 레지스터 `0x200D`, control word `0x200E`, target velocity `0x2088~0x2089`, actual velocity `0x20AB~0x20AC`, fault `0x20A5~0x20A6`를 사용한다. 속도는 signed 16-bit RPM으로 `0x10` multiple-register write 한 번에 동기 전송한다. 일부 통신 문서의 `0x2031 Stop` 예시는 같은 문서의 control-word 표와 충돌하므로 사용하지 않는다.

## 설치와 빌드

```bash
sudo apt install libmodbus-dev
cd /home/jeonga/cart_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rear_ackermann_controller zlac8015d_driver
source install/setup.bash
```

`config/zlac8015d.yaml`에서 `serial_port`, 감속비, 안전 RPM, 좌우 방향을 실제 장비 값으로 바꾼다. `serial_port`에는 `/dev/ttyUSB0`보다 `/dev/serial/by-id/...`의 고정 경로를 권장한다.

## 실행

```bash
ros2 launch zlac8015d_driver hardware_drive.launch.py
```

이 launch는 `rear_ackermann_controller`와 실제 모터 드라이버만 실행한다. Gazebo의 wheel bridge, `alignment_control.launch.py`, 또는 simulation launch와 동시에 실행하지 않는다.

## 안전 시험 순서

1. 물리 E-stop을 준비하고 바퀴를 지면에서 완전히 띄운다.
2. 전원과 노드 실행만으로 바퀴가 회전하지 않는지 확인한다.
3. 낮은 값으로 왼쪽, 오른쪽 바퀴를 각각 시험하고 `left_motor_inverted`, `right_motor_inverted`를 확정한다.
4. 매우 낮은 속도의 직진과 제자리 회전을 확인한다.
5. wheel command를 멈춰 `command_timeout_sec` 뒤 0 RPM이 전송되는지 확인한다.
6. Ctrl+C, USB-RS485 분리, 드라이버 fault에서 정지/명령 차단을 확인한다.
7. 재연결 뒤 이전 속도가 자동 복원되지 않고 새 명령 전까지 0 RPM인지 확인한다.

`Quick Stop`은 RS485 드라이버 명령일 뿐 물리 E-stop을 대체하지 않는다. fault는 자동으로 clear하지 않으며, 원인을 제거한 후에만 사용자가 `/zlac8015d/reset_fault`를 호출해야 한다.
