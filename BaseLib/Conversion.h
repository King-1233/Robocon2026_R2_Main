#ifndef __CONVERSION_H_
#define __CONVERSION_H_

// 齿轮半径，单位：m
#define GEAR_RADIUS_M 0.0495f // 99mm / 2 = 4.95mm = 0.0495m

float distance_to_motor_rad(float distance_m); // 线位移(m) 转换为 目标角度(rad)
float motor_rad_to_distance(float motor_rad);  // 电机角度(rad) 转换为 线位移(m)
float velocity_to_rad_s(float velocity_m_s);   // 线速度(m/s) 转换为 目标角速度(rad/s)
float rad_s_to_velocity_m_s(float rad_s);      // 电机角速度(rad/s) 转换为 线速度(m/s)

#endif
