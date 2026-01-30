#include "PID control.h"
#include <math.h>
PIDDouble roll;
PIDDouble pitch;
PIDSingle yaw_heading;
PIDSingle yaw_rate;
PIDDouble altitude;
// ===== GPS UPDATE RATE =====
#define GPS_DT 0.001f   // 5Hz GPS (RẤT QUAN TRỌNG)

// ===== LIMIT =====
#define MAX_VEL        3.0f     // m/s
#define MAX_ACC        5.0f     // m/s^2
#define GRAVITY        9.81f

// ===== Velocity LPF =====
#define GPS_VEL_LPF    0.3f

// ===== Position → Velocity PID =====
PIDSingle pos_north_pid;
PIDSingle pos_east_pid;

// ===== Velocity → Accel PID =====
PIDSingle vel_north_pid;
PIDSingle vel_east_pid;

// ===== GPS velocity =====
static float vel_north = 0.0f;
static float vel_east  = 0.0f;
static float vel_i_north = 0.0f;
static float vel_i_east  = 0.0f;

float Position_pitch = 0.0f;   // setpoint pitch từ position hold
float Position_roll  = 0.0f;   // setpoint roll  từ position hold

#define ALT_ERR_SUM_MAX   300.0f
#define ALT_OUT_MAX       400.0f     // delta throttle max
#define ALT_D_LPF_ALPHA   0.3f       // D filter
#define DEG2RAD (M_PI / 180.0f)
#define LIMIT(x, min, max) \
    do { if ((x) > (max)) (x) = (max); else if ((x) < (min)) (x) = (min); } while(0)

// Hằng số tuning (có thể điều chỉnh sau)
#define POS_DEADZONE        0.5f    // mét (tăng một chút vì không có velocity loop)
#define MAX_TILT_ANGLE      10.0f   // độ, giới hạn góc nghiêng khi loiter
#define POS_ERR_SUM_MAX     15.0f   // anti-windup cho position PID
#define POS_ERR_SUM_MIN    -15.0f
#define DT 0.001f
#define OUTER_DERIV_FILT_ENABLE 1
#define INNER_DERIV_FILT_ENABLE 1
void PID_Init(void)
{

    // ===== Roll Axis =====
    roll.out.kp = 6.0f;  roll.out.ki = 0.3f;  roll.out.kd = 0.4f;
    roll.in.kp  = 5.0f;  roll.in.ki  = 0.5f;  roll.in.kd  = 0.17f;

    // ===== Pitch Axis =====
    pitch.out.kp = 6.0f; pitch.out.ki = 0.3f; pitch.out.kd = 0.4f;
    pitch.in.kp  = 5.0f; pitch.in.ki  = 0.5f; pitch.in.kd  = 0.17f;

    // ===== Yaw Axis =====
    yaw_heading.kp = 5.0f; yaw_heading.ki = 0.0f;  yaw_heading.kd = 0.30f;// 0.8
    yaw_rate.kp    = 3.0f; yaw_rate.ki    = 0.0f; yaw_rate.kd    = 0.02f;//0.9

    altitude.out.kp = 7.5f; altitude.out.ki = 0.01f; altitude.out.kd = 0.02f;

    // =====  POSITION HOLD PIDs =====
        // Outer loop: Position → Desired velocity
    pos_north_pid.kp = 0.8f;	pos_north_pid.ki = 0.0f; pos_north_pid.kd = 0.1f;
    pos_east_pid.kp = 0.8f;	pos_east_pid.ki = 0.0f; pos_east_pid.kd = 0.1f;

      // Velocity → Accel
    vel_north_pid.kp = 1.8f; vel_north_pid.ki = 0.3f; vel_north_pid.kd = 0.0f;
    vel_east_pid.kp = 1.8f; vel_east_pid.ki = 0.3f; vel_east_pid.kd = 0.0f;

    // ===== Roll Axis =====
//    roll.out.kp = 45.0f;  roll.out.ki = 3.0f;  roll.out.kd = 4.0f;
//    roll.in.kp  = 5.0f;  roll.in.ki  = 5.0f;  roll.in.kd  = 1.5f;
//
//    // ===== Pitch Axis =====
//    pitch.out.kp = 45.0f; pitch.out.ki = 3.0f; pitch.out.kd = 4.0f;
//    pitch.in.kp  = 5.0f; pitch.in.ki  = 5.0f; pitch.in.kd  = 1.5f;
//
//    // ===== Yaw Axis =====
//    yaw_heading.kp = 50.0f; yaw_heading.ki = 0.0f;  yaw_heading.kd = 20.0f;// 0.8
//    yaw_rate.kp    = 15.0f; yaw_rate.ki    = 0.0f; yaw_rate.kd    = 2.0;//0.9

}

void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{   //if(axis == &pitch)  angle -= 2.4f;
	/*********** Bắt đầu bên ngoài PID kép (Điều khiển vị trí góc nghiêng và lăn) *************/
	axis->out.reference = set_point_angle;	//Điểm đặt của điều khiển PID bên ngoài ( Giá trị mong muốn (góc target))
	axis->out.meas_value = angle;			//Góc quay BNO080

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Định nghĩa lỗi của vòng lặp bên ngoài(Sai số = góc target - góc đo)
	axis->out.p_result = axis->out.error * axis->out.kp;			//Tính kết quả P của vòng lặp bên ngoài (P-term = Kp * error)

	axis->out.error_sum = axis->out.error_sum + axis->out.error * DT;	//Định nghĩa tổng của vòng lặp bên ngoài (// Tích lũy sai số cho I-term (error_sum += error * dt))
#define OUT_ERR_SUM_MAX 500
#define OUT_I_ERR_MIN -OUT_ERR_SUM_MAX
	if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
	else if(axis->out.error_sum < OUT_I_ERR_MIN) axis->out.error_sum = OUT_I_ERR_MIN;   // Giới hạn error_sum để tránh tích lũy quá mức (anti-windup)
	axis->out.i_result = axis->out.error_sum * axis->out.ki;			//Tính toán kết quả của vòng lặp bên ngoài
// I-term = error_sum * Ki
	axis->out.error_deriv = -rate;										//Xác định đạo hàm của vòng lặp ngoài (tốc độ = Tốc độ góc ICM-20602)
// D-term dùng tốc độ quay từ gyro (ICM-20602).
// Vì: d(góc)/dt = rate ⇒ đạo hàm error ~ -rate
#if !OUTER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;			//Tính toán kết quả D của vòng lặp bên ngoài
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.4f + axis->out.error_deriv * 0.6f;	//bộ lọc đạo hàm
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;									//Tính toán kết quả D của vòng lặp bên trong
#endif

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Tính toán kết quả PID của vòng lặp bên ngoài
	// Tổng hợp PID outer loop (kết quả chính là "tốc độ mong muốn")
	/****************************************************************************************/
	
	/************ PID kép Inner Begin (Kiểm soát tốc độ góc lăn và nghiêng) **************/
	axis->in.reference = axis->out.pid_result;	//Điểm đặt của điều khiển PID bên trong là kết quả PID của vòng lặp bên ngoài (đối với điều khiển PID kép)
	axis->in.meas_value = rate;					//Tốc độ góc ICM-20602

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Định nghĩa lỗi của vòng lặp bên trong  Sai số tốc độ = tốc độ mong muốn - tốc độ thực
	axis->in.p_result = axis->in.error * axis->in.kp;			//Tính toán kết quả P của vòng lặp bên trong ( P-term = Kp * error (rate))

	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;	//Định nghĩa tổng của vòng lặp bên trong // Tích lũy error cho I-term (rate)
#define IN_ERR_SUM_MAX 500
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->in.error_sum > IN_ERR_SUM_MAX) axis->in.error_sum = IN_ERR_SUM_MAX;
	else if(axis->in.error_sum < IN_I_ERR_MIN) axis->in.error_sum = IN_I_ERR_MIN;


	axis->in.i_result = axis->in.error_sum * axis->in.ki;							//Tính toán kết quả của vòng lặp bên trong

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;	//Xác định đạo hàm của vòng lặp bên trong
	axis->in.meas_value_prev = axis->in.meas_value;									//Làm mới giá trị_trước thành giá trị mới nhất

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Tính toán kết quả D của vòng lặp bên trong
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;	//bộ lọc đạo hàm
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Tính toán kết quả D của vòng lặp bên trong
#endif
	
	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Tính toán kết quả PID của vòng lặp bên trong
	/****************************************************************************************/
}

void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Bắt đầu PID đơn (Vị trí góc lệch) *************/
	axis->reference = set_point_angle;	//Điểm đặt của hướng lái @ cần lái ở giữa.
	axis->meas_value = angle;			//Góc nghiêng BNO080_Yaw hiện tại @ cần lái nằm ở giữa.

	axis->error = axis->reference - axis->meas_value;	//Xác định lỗi điều khiển góc lệch

	if(axis->error > 180.f) axis->error -= 360.f;
	else if(axis->error < -180.f) axis->error += 360.f;
	
	axis->p_result = axis->error * axis->kp;			//Tính toán kết quả P của điều khiển góc lệch

	axis->error_sum = axis->error_sum + axis->error * DT;	//Xác định tổng điều khiển góc lệch
	axis->i_result = axis->error_sum * axis->ki;			//Tính toán kết quả của điều khiển góc lệch

	axis->error_deriv = -rate;						//Xác định sự khác biệt của điều khiển góc lệch
	axis->d_result = axis->error_deriv * axis->kd;	//Tính toán kết quả D của điều khiển góc lệch
	
	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Tính toán kết quả PID của điều khiển góc lệch
	/***************************************************************/
}

void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point_rate, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Bắt đầu PID đơn (Kiểm soát tốc độ góc Yaw) *************/
	axis->reference = set_point_rate;	//Điểm đặt của hướng lái @ cần lái không nằm ở tâm.
	axis->meas_value = rate;			//ICM20602.gyro_z @ thanh điều hướng hiện tại không ở giữa.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw rate control
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw rate control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw rate control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw rate control

	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / DT;	//Define differentiation of yaw rate control
	axis->meas_value_prev = axis->meas_value;								//Refresh value_prev to the latest value
	axis->d_result = axis->error_deriv * axis->kd;							//Calculate D result of yaw rate control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw control
	/*******************************************************************/
}


void Single_Altitude_PID_Calculation(
    PIDDouble* axis,
    float set_point_press,
    float press)
{
    /*********** ALTITUDE PID (SINGLE LOOP) *************/
    axis->out.reference  = set_point_press;
    axis->out.meas_value = press;

    // ===== ERROR =====
    axis->out.error = axis->out.reference - axis->out.meas_value;

    // ===== P =====
    axis->out.p_result = axis->out.error * axis->out.kp;

    // ===== D (on measurement) =====
    float raw_d = -(axis->out.meas_value - axis->out.meas_value_prev) / DT;
    axis->out.meas_value_prev = axis->out.meas_value;

    axis->out.error_deriv_filt =
        axis->out.error_deriv_filt * (1.0f - ALT_D_LPF_ALPHA) +
        raw_d * ALT_D_LPF_ALPHA;

    axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;

    // ===== I (ANTI-WINDUP) =====
    float pid_no_i = axis->out.p_result + axis->out.d_result;

    if (fabsf(pid_no_i) < ALT_OUT_MAX)   // chỉ tích I khi chưa saturate
    {
        axis->out.error_sum += axis->out.error * DT;

        if (axis->out.error_sum > ALT_ERR_SUM_MAX)
            axis->out.error_sum = ALT_ERR_SUM_MAX;
        else if (axis->out.error_sum < -ALT_ERR_SUM_MAX)
            axis->out.error_sum = -ALT_ERR_SUM_MAX;
    }

    axis->out.i_result = axis->out.error_sum * axis->out.ki;

    // ===== TOTAL =====
    axis->out.pid_result =
        axis->out.p_result +
        axis->out.i_result +
        axis->out.d_result;

    // ===== LIMIT OUTPUT =====
    if (axis->out.pid_result > ALT_OUT_MAX)
        axis->out.pid_result = ALT_OUT_MAX;
    else if (axis->out.pid_result < -ALT_OUT_MAX)
        axis->out.pid_result = -ALT_OUT_MAX;
}
void GPS_Velocity_Update(
    float lat_now, float lon_now,
    float lat_prev, float lon_prev)
{
    float lat_to_m = 111139.0f;
    float lon_to_m = lat_to_m * cosf(lat_now * DEG2RAD);

    float vn_raw = (lat_now - lat_prev) * lat_to_m / GPS_DT;
    float ve_raw = (lon_now - lon_prev) * lon_to_m / GPS_DT;

    // Low-pass filter
    vel_north = vel_north * (1.0f - GPS_VEL_LPF) + vn_raw * GPS_VEL_LPF;
    vel_east  = vel_east  * (1.0f - GPS_VEL_LPF) + ve_raw * GPS_VEL_LPF;
}
void Position_Hold_Calculation(
    float target_lat, float target_lon,
    float current_lat, float current_lon,
    float yaw_deg,
    float yaw_rate_deg)
{
	if (fabsf(yaw_rate_deg) > 20.0f)
	{
	    Position_pitch = 0.0f;
	    Position_roll  = 0.0f;
	    vel_i_north = 0;
	    vel_i_east  = 0;
	    return;
	}

    /* ===== 1. POSITION ERROR (WORLD) ===== */
    float lat_to_m = 111139.0f;
    float lon_to_m = lat_to_m * cosf(current_lat * DEG2RAD);

    float err_north = (target_lat - current_lat) * lat_to_m;
    float err_east  = (target_lon - current_lon) * lon_to_m;

    if (fabsf(err_north) < POS_DEADZONE) err_north = 0.0f;
    if (fabsf(err_east)  < POS_DEADZONE) err_east  = 0.0f;

    /* ===== 2. POSITION → VELOCITY (WORLD) ===== */
    float vel_sp_north = pos_north_pid.kp * err_north;
    float vel_sp_east  = pos_east_pid.kp  * err_east;

    LIMIT(vel_sp_north, -MAX_VEL, MAX_VEL);
    LIMIT(vel_sp_east,  -MAX_VEL, MAX_VEL);

    /* ===== 3. VELOCITY PID (WORLD) ===== */
    float vel_err_n = vel_sp_north - vel_north;
    float vel_err_e = vel_sp_east  - vel_east;

    /* ===== YAW RATE COMPENSATION (BẮT BUỘC) ===== */
    float yaw_rate = yaw_rate_deg * DEG2RAD;
    vel_err_n -= yaw_rate * vel_east;
    vel_err_e += yaw_rate * vel_north;


    /* --- Anti-drift khi yaw quay --- */
    if (fabsf(yaw_rate_deg) < 15.0f)
    {
        vel_i_north += vel_err_n * vel_north_pid.ki * GPS_DT;
        vel_i_east  += vel_err_e * vel_east_pid.ki  * GPS_DT;

        LIMIT(vel_i_north, -MAX_ACC, MAX_ACC);
        LIMIT(vel_i_east,  -MAX_ACC, MAX_ACC);
    }
    else
    {
        vel_i_north = 0.0f;
        vel_i_east  = 0.0f;
    }

    float acc_north =
        vel_north_pid.kp * vel_err_n + vel_i_north;

    float acc_east  =
        vel_east_pid.kp * vel_err_e + vel_i_east;

    LIMIT(acc_north, -MAX_ACC, MAX_ACC);
    LIMIT(acc_east,  -MAX_ACC, MAX_ACC);

    /* ===== 4. ACCEL → TILT (WORLD) ===== */
    float pitch_cmd_world =  -acc_north / GRAVITY * RAD2DEG;
    float roll_cmd_world  =  acc_east  / GRAVITY * RAD2DEG;

    /* ===== 5. WORLD → BODY (XOAY 1 LẦN DUY NHẤT) ===== */
    float yaw = yaw_deg * DEG2RAD;
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    Position_pitch =  cy * pitch_cmd_world - sy * roll_cmd_world;
    Position_roll  =  sy * pitch_cmd_world + cy * roll_cmd_world;

    LIMIT(Position_pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    LIMIT(Position_roll,  -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->error_sum = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&roll.in);
	Reset_PID_Integrator(&roll.out);
	Reset_PID_Integrator(&pitch.in);
	Reset_PID_Integrator(&pitch.out);
	Reset_PID_Integrator(&yaw_heading);
	Reset_PID_Integrator(&yaw_rate);
	Reset_PID_Integrator(&altitude.out);
	Reset_PID_Integrator(&pos_east_pid);
	Reset_PID_Integrator(&pos_north_pid);
	Reset_PID_Integrator(&vel_north_pid);
	Reset_PID_Integrator(&vel_east_pid);
}
