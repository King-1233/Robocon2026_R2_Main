#include "Conversion.h"
#include <math.h>

/**
 * @brief 线位移转换为电机角度
 * @param distance_m 齿条移动距离，单位：m
 * @return 目标电机角度，单位：rad
 */
float distance_to_motor_rad(float distance_m)
{
  return distance_m / GEAR_RADIUS_M;
}

/**
 * @brief 电机角度转换为线位移
 * @param motor_rad 电机角度，单位：rad
 * @return 齿条移动位移，单位：m
 */
float motor_rad_to_distance(float motor_rad)
{
  return motor_rad * GEAR_RADIUS_M;
}

/**
 * @brief 线速度转换为电机角速度
 * @param velocity_m_s 线速度，单位：m/s
 * @return 目标电机角速度，单位：rad/s
 */
float velocity_to_rad_s(float velocity_m_s)
{
  return velocity_m_s / GEAR_RADIUS_M;
}

/**
 * @brief 电机角速度转换为线速度
 * @param rad_s 电机角速度，单位：rad/s
 * @return 线速度，单位：m/s
 */
float rad_s_to_velocity_m_s(float rad_s)
{
  return rad_s * GEAR_RADIUS_M;
}
