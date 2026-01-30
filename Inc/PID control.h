/**
 * PID control.c
 * @author ChrisP @ M-HIVE

 * This library source code is for cascade double loop pid control for STM32 Drone Development online course.
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2020
 * Rev. 1.0
 *
 * Where to take the online course.
 * https://www.inflearn.com/course/STM32CubelDE-STM32F4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C (Korean language supported only)
 *
 * Where to buy MH-FC V2.2 STM32F4 Drone Flight Controller.
 * https://smartstore.naver.com/mhivestore/products/4961922335
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif


typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;
	
	float reference;
	float meas_value;
	float meas_value_prev;
	float error;
	float error_sum;
	float error_deriv;
	float error_deriv_filt;
	
	float p_result;
	float i_result;
	float d_result;
	
	float pid_result;
}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;


extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;
extern PIDDouble altitude;
extern PIDSingle pos_north_pid;   // Position North → Desired velocity North
extern PIDSingle pos_east_pid;    // Position East  → Desired velocity East
extern PIDSingle vel_north_pid;   // Velocity North → Pitch setpoint
extern PIDSingle vel_east_pid;    // Velocity East  → Roll setpoint (note: dấu âm ở roll)

// Biến toàn cục lưu góc tilt setpoint do Position Hold tạo ra
extern float Position_pitch;   // Pitch setpoint từ position hold (degree)
extern float Position_roll;    // Roll setpoint từ position hold (degree)
void Position_Hold_Calculation(float target_lat, float target_lon,          // GPS setpoint (độ)
    float current_lat, float current_lon,        // GPS hiện tại (độ)
    float yaw_deg, float yaw_rate_deg        );                       // Yaw từ BNO080 (degree)

void Single_Altitude_PID_Calculation(PIDDouble* axis, float set_point_press, float press );
void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle, float rate);
void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point, float value);
void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point, float angle, float rate);
void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);
void PID_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__PID_CONTROL_H */
#ifndef RAD2DEG
#define RAD2DEG 57.2957795f
#endif

#ifndef DEG2RAD
#define DEG2RAD 0.0174532925f
#endif
