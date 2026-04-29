#include "Run.h"
#include "Task_Init_Main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TrajectoryMath.h"
#include "math.h"
#include "Callback.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
uint8_t STP3_Data[194], STP4_Data[194]; // 测距数据
LiftSystem_t LiftSystem;                // 3个提升电机
PCMotor_t PCMotor;                      // 接收PC命令
TransMotor_t TransMotor;                // 发送USB数据
uint8_t myUsbRxData[64] = {0};          // 接收到的数据
uint8_t test_triggered_up = 0;          // 上台阶测试触发标志位
uint8_t test_triggered_down = 0;        // 下台阶测试触发标志位
uint8_t test_triggered_test_front = 0;  // 前轮测试触发标志位
uint8_t test_triggered_test_mid = 0;    // 中轮测试触发标志位
uint8_t test_triggered_test_rear = 0;   // 后轮测试触发标志位
uint8_t reset = 0;                      // 重置标志位
// Arm_t arm;                          // 机械臂是否执行一次动作结构体实例
uint8_t houliang=0;
TaskHandle_t Rising_Task_handle = NULL;
static bool IsMotorAtTarget(const LiftMotor_t *motor, float target_rad); // 检查电机是否到达目标位置
/**
 * @brief 升降任务
 * @param pvParameters 任务参数
 */
void Rising_Task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  LiftSystem_Init(&LiftSystem); // 初始化提升系统
  vTaskDelay(100);
  APP_SetZeroPosition();                      // 设置提升电机位置为0
  USB_CDC_Init(USB_CDC_callback, NULL, NULL); // 初始化USB CDC
  for (;;)
  {
    uint32_t current_tick = xTaskGetTickCount();
    Parse_PC_Command();            // 解析PC命令
    if (test_triggered_test_front) // 前轮测试触发标志位
    {
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[0], LiftSystem.lift_cmd[0].target_height, LiftSystem.lift_cmd[0].duration_ms);
      test_triggered_test_front = 0;
    }
    if (test_triggered_test_mid) // 中轮测试触发标志位
    {
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[1], LiftSystem.lift_cmd[1].target_height, LiftSystem.lift_cmd[1].duration_ms);
      test_triggered_test_mid = 0;
    }
    if (test_triggered_test_rear) // 后轮测试触发标志位
    {
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[2], LiftSystem.lift_cmd[2].target_height, LiftSystem.lift_cmd[2].duration_ms);
      test_triggered_test_rear = 0;
    }
    if (reset) // 重置标志位
    {
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[0], 0, LiftSystem.lift_cmd[0].duration_ms);
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[1], 0, LiftSystem.lift_cmd[1].duration_ms);
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[2], 0, LiftSystem.lift_cmd[2].duration_ms);
      reset = 0;
    }
		if(houliang)
		{
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[1], LiftSystem.lift_cmd[1].target_height, LiftSystem.lift_cmd[1].duration_ms);
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[2], LiftSystem.lift_cmd[2].target_height, LiftSystem.lift_cmd[2].duration_ms);
      houliang = 0;	
		}

    //    if (Remote_Control.First.Left_Key_Up && Remote_Control.Second.Left_Key_Up)
    //    {
    //      arm.head = 0xAA;
    //      arm.state = 1;
    //      arm.back = 0xBB;
    //      HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&arm, sizeof(Arm_t));
    //    }
    LiftSystem_Update(&LiftSystem); // 更新提升系统状态
    for (int i = 0; i < 3; i++)     // 更新提升电机轨迹
    {
      LiftMotor_UpdateTrajectory(&LiftSystem.motors[i], current_tick);
    }

    for (int i = 0; i < 3; i++) // 更新提升电机状态
    {
      LiftMotor_t *m = &LiftSystem.motors[i];
      float ff_torque = m->exp_acc * LiftSystem.inertia_gain + m->exp_torque; // 计算力矩
      float target_pos = m->pos_offset + m->exp_rad;                          // 计算目标位置
      RobStrideMotionControl(&m->Rs_motor,
                             m->Rs_motor.motor_id,
                             ff_torque,
                             target_pos,
                             m->exp_omega,
                             LiftSystem.motion_kp,
                             LiftSystem.motion_kd);
    }
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
  }
}

void APP_SetZeroPosition(void) // 设置提升电机位置为0
{
  for (int i = 0; i < 3; i++)
  {
    LiftSystem.motors[i].pos_offset = LiftSystem.motors[i].Rs_motor.state.rad;
  }
}
static bool IsMotorAtTarget(const LiftMotor_t *motor, float target_rad) // 判断电机是否到达目标位置
{
  // 1. 安全检查
  if (motor == NULL)
    return false;
  // 2. 计算位置偏差
  float current_rad = (motor->Rs_motor.state.rad - motor->pos_offset) * motor->inv_motor;
  float pos_error = fabsf(current_rad - target_rad);
  // 3. 判定逻辑：位置误差在范围内
  bool is_pos_ok = pos_error < LiftSystem.pos_error_threshold;
  return (is_pos_ok);
}
void LiftSystem_Init(LiftSystem_t *sys) // 初始化提升系统
{
  if (sys == NULL)
    return;
  sys->climb_state = CLIMB_IDLE;       // 初始化上台阶状态为待闲状态
  sys->last_state = CLIMB_IDLE;        // 初始化上台阶状态为待闲状态
  sys->descend_state = DOWN_IDLE;      // 初始化下台阶状态为待闲状态
  sys->last_descend_state = DOWN_IDLE; // 初始化下台阶状态为待闲状态
  sys->work_mode = LIFT_MODE_IDLE;     // 初始化工作模式为待机状态
  sys->motion_kp = 150.0f;             // 初始化运动比例系数为150.0f
  sys->motion_kd = 2.0f;               // 初始化运动微分系数为2.0f
  sys->inertia_gain = 0.08f;           // 初始化惯性增益系数为0.08f
  sys->height_lift_up = 0.23f;         // 初始化上台阶高度为0.23f
  sys->back_height_retract = 0.0f;     // 初始化回缩高度为0.0f
  sys->pos_error_threshold = 0.18f;    // 初始化位置误差阈值为0.12f

  for (int i = 0; i < 3; i++)
  {
    sys->lift_cmd[i].target_height = 0.0f; // 初始化提升命令目标高度为0.0f
    sys->lift_cmd[i].duration_ms = 700.0f; // 初始化提升命令持续时间为700ms
    sys->motors[i].exp_rad = 0.0f;         // 初始化提升电机期望角度为0.0f
    sys->motors[i].exp_omega = 0.0f;       // 初始化提升电机期望角速度为0.0f
    sys->motors[i].exp_acc = 0.0f;         // 初始化提升电机期望加速度为0.0f
    sys->motors[i].use_trajectory = 0;     // 初始化提升电机是否使用轨迹为0
  }
}
void Parse_PC_Command(void) // 解析PC命令
{
  if (LiftSystem.work_mode != LIFT_MODE_IDLE) // 如果提升系统不是待机状态
  {
    PCMotor.state = 0;
    return;
  }
  if (PCMotor.state != 1 && PCMotor.state != 2) // 如果PC命令不是上台阶或下台阶
  {
    return;
  }
  float target_lift_height = 0.23f;
  if (PCMotor.height == 1)
  {
    target_lift_height = 0.23f;
  }
  else if (PCMotor.height == 2)
  {
    target_lift_height = 0.42f;
  }

  LiftSystem.height_lift_up = target_lift_height;// 设置上台阶高度为目标高度

  if (PCMotor.state == 1) // 准备上台阶
  {
    LiftSystem.work_mode = LIFT_MODE_CLIMB_UP;
    LiftSystem.climb_state = CLIMB_LIFT_ALL;
    Send_TransMotor_State(1);
  }
  else if (PCMotor.state == 2) // 准备下台阶
  {
    LiftSystem.work_mode = LIFT_MODE_CLIMB_DOWN;
    LiftSystem.descend_state = DOWN_FORWARD_1;
    Send_TransMotor_State(1);
  }
  PCMotor.state = 0;
}
void Send_TransMotor_State(uint8_t state) // 发送提升电机状态
{
  TransMotor.head = 0xAB;
  TransMotor.state_callback = state;
  TransMotor.back = 0xBA;
  CDC_Transmit_FS((uint8_t *)&TransMotor, sizeof(TransMotor_t));
}
void LiftSystem_Update(LiftSystem_t *sys) // 更新提升系统状态
{
  if (sys == NULL)
    return;

  switch (sys->work_mode)
  {
  case LIFT_MODE_IDLE:
    // 待机处理
    break;
  case LIFT_MODE_CLIMB_UP:
    ClimbFSM_Step(sys); // 上台阶逻辑
    break;
  case LIFT_MODE_CLIMB_DOWN:
    ClimbDown_Step(sys); // 下台阶逻辑
    break;
	case ROMOTE_MODE:
		break;
  }
}
void ClimbFSM_Step(LiftSystem_t *sys) // 状态机更新
{
  if (sys == NULL)
    return;
  bool is_state_entry = (sys->climb_state != sys->last_state);
  sys->last_state = sys->climb_state;
  switch (sys->climb_state)
  {
  case CLIMB_IDLE:
    break;
  case CLIMB_LIFT_ALL:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[1], (sys->height_lift_up), sys->lift_cmd[1].duration_ms);
      LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->height_lift_up, sys->lift_cmd[2].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[1], distance_to_motor_rad(sys->height_lift_up)) &&
        IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->height_lift_up)))
    {
      sys->climb_state = CLIMB_FORWARD_1;
    }
    break;
  case CLIMB_FORWARD_1:
    chassis.exp_vel.x = -0.12f;
    sys->climb_state = CLIMB_FORWARD_2;
    break;
  case CLIMB_RETRACT_FRONT:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[0], sys->back_height_retract, sys->lift_cmd[0].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[0], distance_to_motor_rad(sys->back_height_retract)))
    {
      sys->climb_state = CLIMB_FORWARD_2;
    }
    break;
  case CLIMB_FORWARD_2:
    if (sys->sensor_front.distance < 380.0f)
    {
      sys->climb_state = CLIMB_RETRACT_MID;
    }
    break;
  case CLIMB_RETRACT_MID:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[1], sys->back_height_retract, sys->lift_cmd[1].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[1], distance_to_motor_rad(sys->back_height_retract)))
    {
      sys->climb_state = CLIMB_FORWARD_3;
    }
    break;

  case CLIMB_FORWARD_3:
    if (sys->sensor_rear.distance < 380.0f)
    {
      sys->climb_state = CLIMB_RETRACT_REAR;
    }
    break;
  case CLIMB_RETRACT_REAR:
    if (is_state_entry)
    {
			
      LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->back_height_retract, sys->lift_cmd[2].duration_ms);
      sys->state_start_tick = xTaskGetTickCount();
    }
    if (IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->back_height_retract)) && (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(2000)))
    {
      sys->climb_state = CLIMB_DONE;
    }
    break;
  case CLIMB_DONE:
    chassis.exp_vel.x = 0.0f;
    sys->climb_state = CLIMB_IDLE;
    sys->work_mode = LIFT_MODE_IDLE;
    Send_TransMotor_State(2);
    break;
  }
}
void ClimbDown_Step(LiftSystem_t *sys) // 状态机更新
{
  if (sys == NULL)
    return;
  bool is_state_entry = (sys->descend_state != sys->last_descend_state);
  sys->last_descend_state = sys->descend_state;
  switch (sys->descend_state)
  {
  case DOWN_IDLE:
    break;
  case DOWN_FORWARD_1:
    chassis.exp_vel.x = -0.12f;
    if (sys->motors[0].Rs_motor.state.torque <= 2.0f)
    {
      sys->state_start_tick = xTaskGetTickCount();
    }
    else
    {
      if (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(400))
      {
        sys->descend_state = DOWN_EXTEND_FRONT;
      }
    }
    break;
  case DOWN_EXTEND_FRONT:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[0], sys->height_lift_up, sys->lift_cmd[0].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[0], distance_to_motor_rad(sys->height_lift_up)))
    {
      sys->descend_state = DOWN_FORWARD_2;
    }
    break;
  case DOWN_FORWARD_2:
    if (sys->sensor_rear.distance > 450.0f)
    {
      sys->descend_state = DOWN_EXTEND_MID;
    }
    break;
  case DOWN_EXTEND_MID:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[1], sys->height_lift_up, sys->lift_cmd[1].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[1], distance_to_motor_rad(sys->height_lift_up)))
    {
      sys->descend_state = DOWN_FORWARD_3;
    }
    break;
  case DOWN_FORWARD_3:
    if (is_state_entry) 
    {
      sys->state_start_tick = xTaskGetTickCount();
    }
    if (sys->motors[2].Rs_motor.state.torque >= -1.0f)
    {
      sys->state_start_tick = xTaskGetTickCount();
    }
    else
    {
      if (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(1500))
      {
        sys->descend_state = DOWN_EXTEND_REAR;
      }
    }
    break;
  case DOWN_EXTEND_REAR:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->height_lift_up, sys->lift_cmd[2].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->height_lift_up)))
    {
      sys->descend_state = DOWN_ALL;
    }
    break;
  case DOWN_ALL:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[0], sys->back_height_retract, sys->lift_cmd[0].duration_ms);
      LiftMotor_SetTrajectoryTarget(&sys->motors[1], sys->back_height_retract, sys->lift_cmd[1].duration_ms);
      LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->back_height_retract, sys->lift_cmd[2].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[0], distance_to_motor_rad(sys->back_height_retract)) &&
        IsMotorAtTarget(&sys->motors[1], distance_to_motor_rad(sys->back_height_retract)) &&
        IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->back_height_retract)))
    {
      sys->descend_state = DOWN_DONE;
    }
    break;
  case DOWN_DONE:
    chassis.exp_vel.x = 0.0f;
    sys->descend_state = DOWN_IDLE;
    sys->work_mode = LIFT_MODE_IDLE;
    Send_TransMotor_State(2);
    break;
  default:
    break;
  }
}