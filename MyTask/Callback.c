#include "Callback.h"
#include "Task_Init_Main.h"
#include "Run.h"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)// 接收CAN消息回调函数
{
	if (hcan->Instance == CAN1)
	{
		uint8_t buf[8];
		uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);

		RobStrideRecv_Handle(&LiftSystem.motors[0].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&LiftSystem.motors[1].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&LiftSystem.motors[2].Rs_motor, &hcan1, ID, buf);
	}
}
