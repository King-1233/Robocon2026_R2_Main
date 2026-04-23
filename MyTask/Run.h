#ifndef __RUN_H__
#define __RUN_H__
#include "Callback.h"
#include "PID_old.h"
#include "step.h"
#include "RobStride2.h"
#include "Conversion.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include "ForceChassis.h" 
#include "Task_Init.h"
#include "STP-23L.h"
typedef struct
{
    RobStride_t Rs_motor;      // 电机结构体
    float pos_offset;          // 位置偏移量
    int8_t inv_motor;          // 电机方向取反标志位
    float exp_rad;             // 期望弧度
    float exp_omega;           // 期望角速度
    float exp_torque;          // 期望力矩
    float exp_acc;             // 期望加速度
    QuinticParam_t traj_param; // 五次多项式轨迹参数
    uint8_t use_trajectory;    // 是否使用轨迹规划标志
} LiftMotor_t;

// 上台阶动作状态机枚举
typedef enum
{
    CLIMB_IDLE = 0,      // 0: 空闲状态/待机
    CLIMB_LIFT_ALL,      // 1: 整体抬升车身
    CLIMB_FORWARD_1,     // 2: 第一次前进 (前轮搭上台阶)
    CLIMB_RETRACT_FRONT, // 3: 收起前轮机构
    CLIMB_FORWARD_2,     // 4: 第二次前进 (中轮搭上台阶)
    CLIMB_RETRACT_MID,   // 5: 收起中间底盘
    CLIMB_FORWARD_3,     // 6: 第三次前进 (后轮搭上台阶)
    CLIMB_RETRACT_REAR,  // 7: 收起后轮机构
    CLIMB_DONE           // 8: 上台阶完成
} ClimbState_e;
typedef enum
{
    DOWN_IDLE = 0,
    DOWN_FORWARD_1,
    DOWN_EXTEND_FRONT,
    DOWN_FORWARD_2,
    DOWN_EXTEND_MID,
    DOWN_FORWARD_3,
    DOWN_EXTEND_REAR,
    DOWN_ALL,
    DOWN_DONE, // 下台阶完成
} DescendState_e;
typedef enum
{
    LIFT_MODE_IDLE = 0,
    LIFT_MODE_CLIMB_UP,
    LIFT_MODE_CLIMB_DOWN,
} LiftMode_e;
typedef struct
{
    float target_height; // 目标高度（米）
    float duration_ms;   // 轨迹运行时间（毫秒）
} LiftCmd_t;
typedef struct
{
    uint8_t head; 
    uint8_t state;
	uint8_t back;
}Arm_t;//机械臂是否执行一次动作结构体
typedef struct
{
    LiftMotor_t motors[3];             // 电机指针数
    LiftCmd_t lift_cmd[3];             // 上台阶命令
    float motion_kp;                   // 运动控制刚度 Kp
    float motion_kd;                   // 运动控制阻尼 Kd
    float inertia_gain;                // 惯量增益
    ClimbState_e climb_state;          // 当前所处的状态
    ClimbState_e last_state;           // 上一个状态
    DescendState_e descend_state;      // 下台阶状态
    DescendState_e last_descend_state; // 上一个下台阶状态
    LiftMode_e work_mode;              // 工作模式
	  STP_23L_Data sensor_front;
	  STP_23L_Data sensor_rear;
    float height_lift_up;              // 车身整体抬升的目标高度 (米)
    float back_height_retract;         // 机构收起时的目标高度 (米)
    float pos_error_threshold;         // 判断是否到位的位置误差阈值 (弧度或米)
} LiftSystem_t;
// 任务句柄声明
extern TaskHandle_t Rising_Task_handle;
extern LiftSystem_t LiftSystem; // 3个提升电机
// 任务函数声明
void Rising_Task(void *pvParameters);      // 上台阶任务
void APP_SetZeroPosition(void);            // 设置提升电机位置为0
void ClimbFSM_Step(LiftSystem_t *sys);     // 状态机更新
void LiftSystem_Init(LiftSystem_t *sys);   // 初始化提升系统
void ClimbDown_Step(LiftSystem_t *sys);    // 下台阶状态机更新
void LiftSystem_Update(LiftSystem_t *sys); // 更新提升系统状态
#endif
