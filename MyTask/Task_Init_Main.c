#include "freertos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Task_Init_Main.h"
#include "Run.h"
#include <math.h>

void Task_Init_Main(void)
{
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    MotorInit();
    // 创建上台阶任务
    xTaskCreate(Rising_Task,
                "Rising_Task",
                512,
                NULL,
                3,
                &Rising_Task_handle);
}

void MotorInit(void)
{
    vTaskDelay(2000);

    RS_Offest_inv(&LiftSystem.motors[0], -1, 0.0f,0.0f);//方向好像需要改
    RS_Offest_inv(&LiftSystem.motors[1], 1, 0.0f, 0.0f);
    RS_Offest_inv(&LiftSystem.motors[2], 1, 0.0f, 0.0f);
    vTaskDelay(100);
    RobStrideInit(&LiftSystem.motors[0].Rs_motor, &hcan1, 0x01, CyberGear); // 前
    RobStrideInit(&LiftSystem.motors[1].Rs_motor, &hcan1, 0x02, CyberGear); // 中
    RobStrideInit(&LiftSystem.motors[2].Rs_motor, &hcan1, 0x03, CyberGear); // 后
    vTaskDelay(100);
    RobStrideSetMode(&LiftSystem.motors[0].Rs_motor, RobStride_MotionControl);
    vTaskDelay(100);
    RobStrideSetMode(&LiftSystem.motors[1].Rs_motor, RobStride_MotionControl);
    vTaskDelay(100);
    RobStrideSetMode(&LiftSystem.motors[2].Rs_motor, RobStride_MotionControl);
    vTaskDelay(300);
    RobStrideEnable(&LiftSystem.motors[0].Rs_motor);
    vTaskDelay(300);
    RobStrideEnable(&LiftSystem.motors[1].Rs_motor);
    vTaskDelay(300);
    RobStrideEnable(&LiftSystem.motors[2].Rs_motor);

    vTaskDelay(2000);
}
void RS_Offest_inv(LiftMotor_t *LiftMotor, int8_t inv_motor, float pos_offset, float torque) // 电机方向取反
{
    {
        LiftMotor->inv_motor = inv_motor;
        LiftMotor->pos_offset = pos_offset;
        LiftMotor->exp_torque = torque * inv_motor;
    }
}
void RampToTarget(float *val, float target, float step) // 速度斜坡
{
    float diff = target - *val;
    if (fabsf(diff) < step)
    {
        *val = target;
    }
    else
    {
        *val += (diff > 0 ? step : -step);
    }
}

