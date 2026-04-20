#include "Run.h"
#include "Task_Init_Main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TrajectoryMath.h"
#include "math.h"

LiftSystem_t LiftSystem; // 3个提升电机
Arm_t arm;
uint8_t test_triggered_up = 0;
uint8_t test_triggered_down = 0;
uint8_t test_triggered_test_front = 0;
uint8_t test_triggered_test_mid = 0;
uint8_t test_triggered_test_rear = 0;
float hi=0.1f;
uint8_t shuang=0;
// 按键状态机变量
static uint8_t key_press_count_200 = 0; // 按键次数计数器
static uint8_t key_press_count_400 = 0; // 按键次数计数器
static uint8_t key_down_count_200 = 0;  // 上次按键状态，用于检测下降沿
static uint8_t key_down_count_400 = 0;  // 上次按键状态，用于检测下降沿
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
//		if(Remote_Control.First.Right_Key_Down && Remote_Control.Second.Right_Key_Down)
//		{
//			 LiftSystem.lift_cmd[0].target_height = 0.32f;
//        LiftSystem.lift_cmd[1].target_height = 0.32f;
//        LiftSystem.lift_cmd[2].target_height = 0.32f;
//        test_triggered_test_front = 1;
//        test_triggered_test_mid = 1;
//        test_triggered_test_rear = 1;
//		}
//		if(Remote_Control.First.Right_Key_Right && Remote_Control.Second.Right_Key_Right)
//		{
//			 LiftSystem.lift_cmd[0].target_height =hi;
//			LiftSystem.lift_cmd[0].duration_ms =500.0f;
//        test_triggered_test_front = 1;
//		}
//		if(Remote_Control.First.Right_Key_Up && Remote_Control.Second.Right_Key_Up)
//		{
//			 LiftSystem.lift_cmd[1].target_height =hi;
//			LiftSystem.lift_cmd[1].duration_ms =500.0f;
//        test_triggered_test_mid = 1;
//		}
//		if(Remote_Control.First.Right_Key_Left && Remote_Control.Second.Right_Key_Left)
//		{
//			 LiftSystem.lift_cmd[2].target_height =hi;
//			LiftSystem.lift_cmd[2].duration_ms =500.0f;
//        test_triggered_test_rear = 1;
//		}
//		
//		
		
		
////    if(Remote_Control.First.Right_Key_Down && Remote_Control.Second.Right_Key_Down)
////    {
////      // 检测到按键下降沿，计数器加1
////      key_press_count_200++;
////      if (key_press_count_200 > 4)
////      {
////        key_press_count_200 = 1; // 超过4次后回到1
////      }
////      // 根据按键次数执行不同任务
////      switch (key_press_count_200)
////      {
////      case 1: // 第1次按下：前电机升起
////        LiftSystem.lift_cmd[0].target_height = 0.22f;
////        LiftSystem.lift_cmd[1].target_height = 0.22f;
////        LiftSystem.lift_cmd[2].target_height = 0.22f;
////        test_triggered_test_front = 1;
////        test_triggered_test_mid = 1;
////        test_triggered_test_rear = 1;
////        break;

////      case 2: // 第2次按下：前电机归位
////        LiftSystem.lift_cmd[0].target_height = 0.0f;
////        test_triggered_test_front = 1;
////        break;

////      case 3: // 第3次按下：中电机归位
////        LiftSystem.lift_cmd[1].target_height = 0.0f;
////        test_triggered_test_mid = 1;
////        break;

////      case 4: // 第4次按下：全部归位
////        LiftSystem.lift_cmd[2].target_height = 0.0f;
////        test_triggered_test_rear = 1;
////        break;

////      default:
////        break;
////      }
////    }
////    if (Remote_Control.First.Right_Key_Up && Remote_Control.Second.Right_Key_Up)
////    {
////      // 检测到按键下降沿，计数器加1
////      key_press_count_400++;
////      if (key_press_count_400 > 4)
////      {
////        key_press_count_400 = 1; // 超过4次后回到1
////      }
////      // 根据按键次数执行不同任务
////      switch (key_press_count_400)
////      {
////      case 1: // 第1次按下：前电机升起
////        LiftSystem.lift_cmd[0].target_height = 0.42f;
////        LiftSystem.lift_cmd[1].target_height = 0.42f;
////        LiftSystem.lift_cmd[2].target_height = 0.42f;
////        test_triggered_test_front = 1;
////        test_triggered_test_mid = 1;
////        test_triggered_test_rear = 1;
////        break;

////      case 2: // 第2次按下：前电机归位
////        LiftSystem.lift_cmd[0].target_height = 0.0f;
////        test_triggered_test_front = 1;
////        break;

////      case 3: // 第3次按下：中电机归位
////        LiftSystem.lift_cmd[1].target_height = 0.0f;
////        test_triggered_test_mid = 1;
////        break;

////      case 4: // 第4次按下：全部归位
////        LiftSystem.lift_cmd[2].target_height = 0.0f;
////        test_triggered_test_rear = 1;
////        break;

////      default:
////        break;
////      }
////    }
////		
////    if (Remote_Control.First.Right_Key_Left && Remote_Control.Second.Right_Key_Left)
////    {
////      key_down_count_200++;
////      if (key_down_count_200 > 4)
////      {
////        key_down_count_200 = 1; // 超过4次后回到1
////      }
////      // 根据按键次数执行不同任务
////      switch (key_down_count_200)
////      {
////      case 1: // 第1次按下：前电机升起
////        LiftSystem.lift_cmd[0].target_height = 0.22f;
////        test_triggered_test_front = 1;
////        break;

////      case 2: // 第2次按下：前电机归位
////        LiftSystem.lift_cmd[1].target_height = 0.22f;
////        test_triggered_test_mid = 1;
////        break;

////      case 3: // 第3次按下：中电机归位
////        LiftSystem.lift_cmd[2].target_height = 0.22f;
////        test_triggered_test_rear = 1;
////        break;

////      case 4: // 第4次按下：全部归位
////        LiftSystem.lift_cmd[0].target_height = 0.0f;
////        LiftSystem.lift_cmd[1].target_height = 0.0f;
////        LiftSystem.lift_cmd[2].target_height = 0.0f;
////        test_triggered_test_rear = 1;
////        test_triggered_test_front = 1;
////        test_triggered_test_mid = 1;
////        break;

////      default:
////        break;
////      }
////    }
////    if (Remote_Control.First.Right_Key_Right && Remote_Control.Second.Right_Key_Right)
////    {
////      key_down_count_400++;
////      if (key_down_count_400 > 4)
////      {
////        key_down_count_400 = 1; // 超过4次后回到1
////      }
////      // 根据按键次数执行不同任务
////      switch (key_down_count_400)
////      {
////      case 1: // 第1次按下：前电机升起
////        LiftSystem.lift_cmd[0].target_height = 0.45f;
////        test_triggered_test_front = 1;
////        break;

////      case 2: // 第2次按下：前电机归位
////        LiftSystem.lift_cmd[1].target_height = 0.45f;
////        test_triggered_test_mid = 1;
////        break;

////      case 3: // 第3次按下：中电机归位
////        LiftSystem.lift_cmd[2].target_height = 0.45f;
////        test_triggered_test_rear = 1;
////        break;

////      case 4: // 第4次按下：全部归位
////        LiftSystem.lift_cmd[0].target_height = 0.1f;
////        LiftSystem.lift_cmd[1].target_height = 0.1f;
////        LiftSystem.lift_cmd[2].target_height = 0.1f;
////        test_triggered_test_rear = 1;
////        test_triggered_test_front = 1;
////        test_triggered_test_mid = 1;
////        break;

////      default:
////        break;
////      }
////    }
//    if (Remote_Control.First.Left_Key_Down && Remote_Control.Second.Left_Key_Down)
//    {
//			key_down_count_400=0;
//			key_down_count_200=0;
//			key_press_count_400=0;
//			key_press_count_200=0;
//      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[0], 0.0f, 1500.0f);
//      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[1], 0.0f, 1500.0f);
//      LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[2], 0.0f, 1500.0f);
//    }
			if (Remote_Control.First.Left_Key_Up && Remote_Control.Second.Left_Key_Up)
   {
		 arm.head=0xAA;
		 arm.state=1;
		 arm.back=0xBB;
		 HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&arm, sizeof(Arm_t));
	 }
	 if(shuang)
	 {
		 test_triggered_test_front=1;
		 test_triggered_test_rear=1;
		 shuang=0;
	 }
	 
    LiftSystem_Update(&LiftSystem);
    for (int i = 0; i < 3; i++)
    {
      LiftMotor_UpdateTrajectory(&LiftSystem.motors[i], current_tick);
    }
		
    LiftMotor_t *m0 = &LiftSystem.motors[0];
float ff_torque0 = m0->exp_acc * LiftSystem.inertia_gain + m0->exp_torque;
float target_pos0 = m0->pos_offset + m0->exp_rad;
RobStrideMotionControl(&m0->Rs_motor, m0->Rs_motor.motor_id,
                        ff_torque0, target_pos0, m0->exp_omega,
                        70, LiftSystem.motion_kd);

LiftMotor_t *m1 = &LiftSystem.motors[1];
float ff_torque1 = m1->exp_acc * LiftSystem.inertia_gain + m1->exp_torque;
float target_pos1 = m1->pos_offset + m1->exp_rad;
RobStrideMotionControl(&m1->Rs_motor, m1->Rs_motor.motor_id,
                        ff_torque1, target_pos1, m1->exp_omega,
                        70, LiftSystem.motion_kd);

LiftMotor_t *m2 = &LiftSystem.motors[2];
float ff_torque2 = m2->exp_acc * LiftSystem.inertia_gain + m2->exp_torque;
float target_pos2 = m2->pos_offset + m2->exp_rad;
RobStrideMotionControl(&m2->Rs_motor, m2->Rs_motor.motor_id,
                        ff_torque2, target_pos2, m2->exp_omega,
                        70, LiftSystem.motion_kd);
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
  sys->motion_kp = 12.0f;
  sys->motion_kd = 0.2f;
  sys->inertia_gain = 0.08f;
  sys->height_lift_up = 0.2f;
  sys->back_height_retract = 0.0f;
  sys->pos_error_threshold = 0.001f;

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
      LiftMotor_SetTrajectoryTarget(&sys->motors[0], sys->height_lift_up, sys->lift_cmd[0].duration_ms);
      LiftMotor_SetTrajectoryTarget(&sys->motors[1], sys->height_lift_up, sys->lift_cmd[1].duration_ms);
      LiftMotor_SetTrajectoryTarget(&sys->motors[2], sys->height_lift_up, sys->lift_cmd[2].duration_ms);
    }
    if (IsMotorAtTarget(&sys->motors[0], distance_to_motor_rad(sys->height_lift_up)) &&
        IsMotorAtTarget(&sys->motors[1], distance_to_motor_rad(sys->height_lift_up)) &&
        IsMotorAtTarget(&sys->motors[2], distance_to_motor_rad(sys->height_lift_up)))
    {
      sys->climb_state = CLIMB_FORWARD_1;
    }
    break;
  case CLIMB_FORWARD_1:
    // TODO: 检查底盘是否前进到位
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
    // TODO: 检查底盘是否前进到位
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
    // TODO: 检查底盘是否前进到位
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
    // TODO: 检查底盘是否前进到位
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
    // TODO: 检查底盘是否前进到位
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
    // TODO: 检查底盘是否前进到位
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
