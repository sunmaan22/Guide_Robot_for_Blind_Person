#include "main.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "joystick.h"
#include "debug_uart.h"
#include "ros_node.h"

/* 조이스틱 X축(전후진) 임계값 - 3.1/4.2절 스펙 */
#define JOY_X_BACKWARD_TH 42000
#define JOY_X_FORWARD_TH  20000
#define FORWARD_FIXED_SPEED 700

/* IMU pitch 기반 전진 속도 매핑 (목줄이 아래를 향할수록 빨라짐) */
#define PITCH_SPEED_TH_DEG 35.0f

/* 주변장치 핸들 (CubeMX가 생성하는 초기화 함수들이 채워준다고 가정) */
ADC_HandleTypeDef  hadc2;
DMA_HandleTypeDef  hdma_adc2;
I2C_HandleTypeDef  hi2c4;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef  hdma_usart1_rx;

/* CubeMX가 생성하는 표준 초기화 함수들 (본 저장소에는 HAL 보일러플레이트를
 * 포함하지 않으므로 시그니처만 선언한다 - 실제 프로젝트에서는 CubeMX가
 * SystemClock_Config/MX_GPIO_Init 등을 자동 생성한다) */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

static int Touch_IsActive(void)
{
    return HAL_GPIO_ReadPin(TOUCH_SW_GPIO_Port, TOUCH_SW_Pin) == GPIO_PIN_SET;
}

/* 3~4절: 터치 안전 스위치 + 조이스틱/IMU pitch 기반 주행 판단 */
static void UpdateDriveDecision(float pitch_deg, uint16_t joy_x)
{
    if (!Touch_IsActive()) {
        Motor_Stop();
        return;
    }

    int32_t state = ROS_GetState();
    if (state != 2 && state != -1) {
        Motor_Stop();
        return;
    }

    int obstacle = ROS_ObstacleNear();

    if (!obstacle && joy_x > JOY_X_BACKWARD_TH) {
        Motor_Backward();
    } else if (!obstacle && joy_x < JOY_X_FORWARD_TH) {
        Motor_Forward(FORWARD_FIXED_SPEED);
    } else if (!obstacle && pitch_deg > PITCH_SPEED_TH_DEG) {
        int16_t speed;
        if (pitch_deg >= 90.0f) {
            speed = 1000;
        } else {
            speed = (int16_t)(700 + (pitch_deg - 36.0f) * (300.0f / 54.0f));
        }
        Motor_Forward(speed);
    } else {
        Motor_Stop();
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC2_Init();
    MX_I2C4_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();

    /* 서보/ESC PWM 채널 활성화 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    Motor_Stop();

    MPU6050_Init();
    Joystick_Init();
    ROS_Init();

    uint32_t last_adc_tick = HAL_GetTick();
    uint32_t last_ros_tick = HAL_GetTick();
    uint32_t last_debug_tick = HAL_GetTick();
    uint32_t last_imu_tick = HAL_GetTick();

    float pitch_deg = 0.0f, roll_deg = 0.0f;

    while (1) {
        uint32_t now = HAL_GetTick();

        /* IMU는 매 루프 반복마다 읽어 상보필터 적분 주기를 일정하게 유지 */
        MPU6050_Data imu;
        Read_Accel(&imu);
        Read_Gyro(&imu);
        float dt_s = (now - last_imu_tick) / 1000.0f;
        if (dt_s <= 0.0f) dt_s = 0.001f;
        Calculate_CompAngle(&imu, dt_s, &pitch_deg, &roll_deg);
        last_imu_tick = now;

        /* 50ms 주기: 조이스틱 ADC 변환 트리거 */
        if (now - last_adc_tick >= 50) {
            Joystick_TriggerRead();
            last_adc_tick = now;
        }

        /* 조향: 조이스틱 Y축을 직접 서보로 반영 (로컬 수동 조향) */
        Servo_SetAngle(Joystick_GetSteeringAngleDeg());

        /* 주행 판단: 터치 안전 스위치 + 자동/초기 상태 + 조이스틱X/장애물/IMU pitch */
        UpdateDriveDecision(pitch_deg, Joystick_GetRawX());

        /* 10ms 주기: rosserial spinOnce (steering_callback이 필요 시 서보를 덮어씀) */
        if (now - last_ros_tick >= 10) {
            ROS_SpinOnce();
            last_ros_tick = now;
        }

        /* 500ms 주기: 디버그 상태 로그 */
        if (now - last_debug_tick >= 500) {
            Debug_PrintStatus(pitch_deg, Joystick_GetRawX(), Joystick_GetRawY(), Touch_IsActive());
            last_debug_tick = now;
        }
    }
}
