#include "Run.h"
#include "Task_Init_Main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TrajectoryMath.h"
#include "math.h"

uint8_t STP3_Data[194],STP4_Data[194];
LiftSystem_t LiftSystem;               // 3个提升电机
//Arm_t arm;                             // 机械臂是否执行一次动作结构体实例
uint8_t test_triggered_up = 0;         // 上台阶测试触发标志位
uint8_t test_triggered_down = 0;       // 下台阶测试触发标志位
uint8_t test_triggered_test_front = 0; // 前轮测试触发标志位
uint8_t test_triggered_test_mid = 0;   // 中轮测试触发标志位
uint8_t test_triggered_test_rear = 0;  // 后轮测试触发标志位
uint8_t shuang = 0;                    // 是否双轮测试标志位
uint8_t reset =0;
TaskHandle_t Rising_Task_handle = NULL;
static bool IsMotorAtTarget(const LiftMotor_t *motor, float target_rad);
/**
 * @brief 升降任务
 * @param pvParameters 任务参数
 */
void Rising_Task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  LiftSystem_Init(&LiftSystem);
  vTaskDelay(100);
  APP_SetZeroPosition();
  for (;;)
  {
    uint32_t current_tick = xTaskGetTickCount();
    if (test_triggered_up && LiftSystem.work_mode == LIFT_MODE_IDLE)
    {
      LiftSystem.work_mode = LIFT_MODE_CLIMB_UP;
      LiftSystem.climb_state = CLIMB_LIFT_ALL; // 初始化上台阶状态
      test_triggered_up = 0;
    }
    else if (test_triggered_down && LiftSystem.work_mode == LIFT_MODE_IDLE)
    {
      LiftSystem.work_mode = LIFT_MODE_CLIMB_DOWN;
      LiftSystem.descend_state = DOWN_FORWARD_1; // 初始化下台阶状态
      test_triggered_down = 0;
    }
    if (test_triggered_test_front)
    {
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[0], LiftSystem.lift_cmd[0].target_height, LiftSystem.lift_cmd[0].duration_ms);
      test_triggered_test_front = 0;
    }
    if (test_triggered_test_mid)
    {
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[1], LiftSystem.lift_cmd[1].target_height, LiftSystem.lift_cmd[1].duration_ms);
      test_triggered_test_mid = 0;
    }
    if (test_triggered_test_rear)
    {
      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[2], LiftSystem.lift_cmd[2].target_height, LiftSystem.lift_cmd[2].duration_ms);
      test_triggered_test_rear = 0;
    }
		if(reset)
		{
			LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[0], 0, LiftSystem.lift_cmd[0].duration_ms);
			LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[1], 0, LiftSystem.lift_cmd[1].duration_ms);
			LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[2], 0, LiftSystem.lift_cmd[2].duration_ms);
			reset=0;
		}
			
//    if (Remote_Control.First.Left_Key_Up && Remote_Control.Second.Left_Key_Up)
//    {
//      arm.head = 0xAA;
//      arm.state = 1;
//      arm.back = 0xBB;
//      HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&arm, sizeof(Arm_t));
//    }
    if (shuang)
    {
      test_triggered_test_front = 1;
      test_triggered_test_rear = 1;
      shuang = 0;
    }
    LiftSystem_Update(&LiftSystem);
    for (int i = 0; i < 3; i++)
    {
      LiftMotor_UpdateTrajectory(&LiftSystem.motors[i], current_tick);
    }

    for (int i = 0; i < 3; i++)
    {
      LiftMotor_t *m = &LiftSystem.motors[i];
      float ff_torque = m->exp_acc * LiftSystem.inertia_gain + m->exp_torque;
      float target_pos = m->pos_offset + m->exp_rad;
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
  sys->climb_state = CLIMB_IDLE;
  sys->last_state = CLIMB_IDLE;
  sys->descend_state = DOWN_IDLE;
  sys->last_descend_state = DOWN_IDLE;
  sys->work_mode = LIFT_MODE_IDLE;
  sys->motion_kp = 150.0f;
  sys->motion_kd = 2.0f;
  sys->inertia_gain = 0.08f;
  sys->height_lift_up = 0.23f;
  sys->back_height_retract = 0.0f;
  sys->pos_error_threshold = 0.12f;

  for (int i = 0; i < 3; i++)
  {
    sys->lift_cmd[i].target_height = 0.0f;
    sys->lift_cmd[i].duration_ms = 700.0f;
    sys->motors[i].exp_rad = 0.0f;
    sys->motors[i].exp_omega = 0.0f;
    sys->motors[i].exp_acc = 0.0f;
    sys->motors[i].use_trajectory = 0;
  }
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
//			LiftMotor_SetTrajectoryTarget(&sys->motors[0], sys->lift_cmd[0].target_height, sys->lift_cmd[0].duration_ms);
//			LiftMotor_SetTrajectoryTarget(&sys->motors[1], sys->lift_cmd[1].target_height, sys->lift_cmd[1].duration_ms);
//			LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->lift_cmd[2].target_height, sys->lift_cmd[2].duration_ms);
		LiftMotor_SetTrajectoryTarget(&sys->motors[0], sys->height_lift_up, sys->lift_cmd[0].duration_ms);
		LiftMotor_SetTrajectoryTarget(&sys->motors[1], sys->height_lift_up, sys->lift_cmd[1].duration_ms);
		LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->height_lift_up, sys->lift_cmd[2].duration_ms);
    }
//		if (IsMotorAtTarget(&sys->motors[0], distance_to_motor_rad(sys->lift_cmd[0].target_height)) &&
//        IsMotorAtTarget(&sys->motors[1], distance_to_motor_rad(sys->lift_cmd[1].target_height)) &&
//        IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->lift_cmd[2].target_height)))
    if (IsMotorAtTarget(&sys->motors[0], distance_to_motor_rad(sys->height_lift_up)) &&
        IsMotorAtTarget(&sys->motors[1], distance_to_motor_rad(sys->height_lift_up)) &&
        IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->height_lift_up)))
    {
      sys->climb_state = CLIMB_FORWARD_1;
    }
    break;
  case CLIMB_FORWARD_1:
		if (is_state_entry) 
		{
     sys->state_start_tick = xTaskGetTickCount();
    }
		if (xTaskGetTickCount() - sys->state_start_tick >= 500) {
        sys->climb_state = CLIMB_RETRACT_FRONT;
    }
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
		if(sys->sensor_front.distance < 380.0f)
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
    if(sys->sensor_rear.distance < 380.0f)
    {
      sys->climb_state = CLIMB_RETRACT_REAR;
    }
    break;
  case CLIMB_RETRACT_REAR:
    if (is_state_entry)
    {
      LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->back_height_retract, sys->lift_cmd[2].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->back_height_retract)))
    {
      sys->climb_state = CLIMB_DONE;
    }
    break;
  case CLIMB_DONE:
    sys->climb_state = CLIMB_IDLE;
    sys->work_mode = LIFT_MODE_IDLE;
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
   if (sys->motors[0].Rs_motor.state.torque <= 2.5f) 
    {
        sys->state_start_tick = xTaskGetTickCount(); 
    }
		else 
    {
        if (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(200)) 
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
    if(sys->sensor_rear.distance > 450.0f)
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
		if (xTaskGetTickCount() - sys->state_start_tick >= 700) {
        sys->descend_state = DOWN_EXTEND_REAR;
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
    sys->descend_state = DOWN_IDLE;
    sys->work_mode = LIFT_MODE_IDLE;
    break;
  default:
    break;
  }
}
