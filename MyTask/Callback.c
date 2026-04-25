#include "Callback.h"
#include "Task_Init_Main.h"
#include "Run.h"
#include "STP-23L.h"
#include "Task_Init.h"
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
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)// 接收CAN消息回调函数
//{
//	if (hcan->Instance == CAN2)
//	{
//		uint8_t buf[8];
//		uint32_t ID = CAN_Receive_DataFrame(&hcan2, buf);
//		RobStrideRecv_Handle(&LiftSystem.motors[1].Rs_motor, &hcan2, ID, buf);
//	}
//}
	
void UART_IT(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) && __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        if (huart->Instance == USART3)
        { // 波特率230400
            HAL_UART_DMAStop(huart);
            STP_23L_DataProcess(STP3_Data, &LiftSystem.sensor_front);
            HAL_UART_Receive_DMA(&huart3, STP3_Data, sizeof(STP3_Data));
        }
        if (huart->Instance == UART4)
        { // 波特率230400
            HAL_UART_DMAStop(huart);
            STP_23L_DataProcess(STP4_Data, &LiftSystem.sensor_rear);
            HAL_UART_Receive_DMA(&huart4, STP4_Data, sizeof(STP4_Data));
        } 
	}
}