# STM32 저수준 제어 펌웨어

BADUK(바둑이) 안내로봇에서 **모터 구동, 조향, IMU 기반 속도 제어, 안전 정지**를 담당하는 STM32H735G-DK 펌웨어입니다. 라즈베리파이(ROS)가 인지·경로계획을 담당하고, STM32는 rosserial로 연결되어 실시간성이 중요한 저수준 제어와 안전 신호를 전담합니다.

> ⚠️ **복원 코드 안내**: 원본 STM32 프로젝트 소스는 남아있지 않아, 캡스톤 발표자료와 별도로 작성해둔 "STM32 코드 리뷰" 문서(주변장치 구성, 함수 시그니처, ROS 토픽/상태 로직, 루프 주기까지 상세 기술됨)를 근거로 **기능적으로 동일하게 재구성한 참고용 코드**입니다. STM32CubeMX가 생성하는 `Drivers/`, `.ioc`, 링커 스크립트 등 보일러플레이트는 포함하지 않았고, 로직이 담긴 애플리케이션 코드만 담았습니다.

## 보드 및 주변장치

| 항목 | 내용 |
|---|---|
| 보드 | STM32H735G-DK |
| 통신 (ROS) | USART1 — rosserial (DMA RX + IDLE 라인 인터럽트) |
| 통신 (디버그) | USART3 — 500ms 주기 상태 로그 |
| 조향 서보 | TIM1 CH4 PWM (1000~2000µs) |
| 구동 ESC | TIM4 CH2 PWM (1000~2000µs, 중립 1500µs) |
| 조이스틱 | ADC2 + DMA (50ms 주기 트리거) |
| IMU | MPU6050, I2C4 |
| 안전 스위치 | 터치 센서 GPIO |

## ROS 인터페이스 (rosserial, USART1)

| 토픽 | 타입 | 방향 | 역할 |
|---|---|---|---|
| `/steering_angle` | `std_msgs/Float32` | Subscribe | 조향각(rad) → 서보 각도 변환 |
| `/state_msg` | `std_msgs/Int32` | Subscribe | 0=정지, 1=수동(정지 처리), 2=자동 |
| `/lidar_distance` | `std_msgs/Float32` | Subscribe | 0.5m 이하면 장애물 근접으로 판단, 주행 차단 |
| `stm32_echo` | `std_msgs/Int32` | Publish | 테스트/헬스체크용 |

## 주행 판단 로직 (10ms 루프)

1. 터치 센서가 눌려있지 않으면 무조건 `Motor_Stop()`
2. `current_state`가 `2`(자동) 또는 `-1`(초기값)일 때만 아래 우선순위로 판단
   - 장애물 없음 + 조이스틱 X축이 뒤로 충분히 젖혀짐 → 후진
   - 장애물 없음 + 조이스틱 X축이 앞으로 충분히 젖혀짐 → 고정 속도(700) 전진
   - 장애물 없음 + IMU pitch > 35° (목줄이 아래를 향함) → pitch에 비례한 속도로 전진
   - 그 외 → 정지
3. 그 외 상태(`0`, `1`)에서는 정지

## 폴더 구조

```
stm32/
└── Core/
    ├── Inc/   # 헤더
    └── Src/
        ├── main.c              # HAL 초기화, 50ms/10ms 루프, 주행 판단 로직
        ├── motor_control.c     # Servo_SetAngle, ESC_SetPWM, Motor_Forward/Backward/Stop
        ├── mpu6050.c           # I2C4 IMU 읽기 + 상보필터(gyro 98% / accel 2%)
        ├── joystick.c          # ADC2 DMA 조이스틱 데드존 처리
        ├── debug_uart.c        # USART3 디버그 로그
        ├── rosserial_hardware.cpp  # rosserial Hardware 클래스 (UART1 DMA+IDLE)
        └── ros_node.cpp        # NodeHandle, Subscriber/Publisher, 콜백
```
