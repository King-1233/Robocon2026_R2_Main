#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "drive_callback.h"
#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_device.h"
#include "motorEx.h"
#include "Run.h"
#include "RobStride2.h"
//#define M_PI 3.1415926f
#define MAX_ROBOT_VEL 2.0f //	 m/s
#define MAX_ROBOT_OMEGA ANGLE2RAD(60.0f)

void Task_Init(void);

typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;
#pragma pack(1)
typedef struct
{
    uint8_t head;       // 帧头
    uint32_t key_value; // uint32_t按键值
    uint8_t tail;       // 帧尾
} KeyUint32Pack_t;
#pragma pack()
typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef enum{
    STOP,
    REMOTE,
    AUTO,
}ChassisMode;
typedef struct {
    float Vx;
    float Vy;
    float Vw;
} Chassis_Vel_t;
#pragma pack(1)
typedef struct
{
    uint8_t head;
    float expectDirection[2];
    float expextVelocity[2];
    uint8_t tail;
} Pack_TransRemote_t;
#pragma pack()

typedef struct
{
    UART_HandleTypeDef *huart;
    uint16_t len;
    uint8_t data[32];   // 你的结构体大约 1+8+8+1=18字节，32足够
} UartTxMsg_t;

void Send_Remote_Data(UART_HandleTypeDef *huart,float dir_one, float dir_two,float vel_one, float vel_two);
extern Chassis_Vel_t chassis_vel;
extern ChassisMode chassis_mode;       // 当前底盘模式
extern Remote_Handle_t Remote_Control;
#endif
