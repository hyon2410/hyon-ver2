/*
 * compass.c
 *
 *  Created on: 17 Mar 2023
 *      Author: sahin
 */
#include "compass.h"
#include "math.h"
#include "FS-iA6B.h"
#include "main.h"
#include "mpu6050.h"
#include "log_to_flash.h"
#define MAG_DEC 0.0f
#define COMPASS_TIMEOUT 1

// QMC5883L I2C Address (7-bit is 0x0D, shifted left is 0x1A)
#define QMC5883L_ADDR (0x0D << 1)

double MagX_float,MagY_float,MagZ_float;
float heading_corrected;
extern float calibration_buffer_float[5];
extern float Zsf,Zoff;
float Sf_Toplam=0;


/// this function rescues the hang i2c bus that is connected to compass
void i2c_disconnected_compass(){

	uint32_t data;
	HAL_I2C_DeInit(&hi2c3);
	data=GPIOA->MODER;
	data= data & ~0b11<<16;
	data= data | 0b01<<16;
	GPIOA->MODER=data;


	data=GPIOA->OTYPER;

	data = data & ~0b1<<8;
	GPIOA->OTYPER=data;

	data=GPIOA->AFR[1];
	data=  data & ~0b1111;
	GPIOA->AFR[1]=data;

	for(int i=0;i<14;i++) // either 9 or 14
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	}

	  HAL_I2C_Init(&hi2c3);
}
extern I2C_HandleTypeDef hi2c2;
void compass_read_corrected(){
	// QMC5883L: Read 6 bytes starting from register 0x00 (Data Output X LSB)
	HAL_I2C_Mem_Read(&hi2c3, QMC5883L_ADDR, 0x00, 1, MAGZ, 6, COMPASS_TIMEOUT);

	// QMC5883L is Little Endian (LSB first) and order is X, Y, Z
	MagX = (int16_t)(MAGZ[1] << 8 | MAGZ[0]);
	MagY = (int16_t)(MAGZ[3] << 8 | MAGZ[2]);
	MagZ = (int16_t)(MAGZ[5] << 8 | MAGZ[4]);

	MagY=1*MagY;
	MagX=1*MagX;
	MagZ=1*MagZ;

	MagX = Xsf* (MagX - Xoff);

	MagY = Ysf * (MagY - Yoff);

	MagZ = Zsf* (MagZ - Zoff);

	MagX_float = +(double)MagX*cos(angle_roll_output*M_PI/180)
	+(double)MagY*sin(angle_pitch_output*M_PI/180)*sin(angle_roll_output*M_PI/180)
	-(double)MagZ*cos(angle_pitch_output*M_PI/180)*sin(angle_roll_output*M_PI/180);

	MagY_float = (double)MagY*cos(angle_pitch_output*M_PI/180)
	-(double)MagZ*sin(angle_pitch_output*M_PI/180);

	double norm = sqrt(MagX_float*MagX_float + MagY_float*MagY_float);
	if(norm > 0.0001f){
	    MagX_float /= norm;
	    MagY_float /= norm;
	}

	 heading_corrected = atan2(MagY_float,MagX_float ) * 180 / M_PI + MAG_DEC;
	 if (heading_corrected >= 360) heading_corrected -= 360;
	 if (heading_corrected < 0) heading_corrected += 360;

}
void compass_read(){

	// QMC5883L Read
	HAL_I2C_Mem_Read(&hi2c3, QMC5883L_ADDR, 0x00, 1, MAGZ, 6, COMPASS_TIMEOUT);
	MPU6050_Read_All_Benim(&hi2c2, &MPU6050_1);

	MagX = (int16_t)(MAGZ[1] << 8 | MAGZ[0]);
	MagY = (int16_t)(MAGZ[3] << 8 | MAGZ[2]);
	MagZ = (int16_t)(MAGZ[5] << 8 | MAGZ[4]);

	MagY=1*MagY;
	MagX=1*MagX;
	MagZ=1*MagZ;

}
void compas_calibrate()
{
    int compass_calibrated = 0;
    uint32_t start_time = HAL_GetTick();
    uint32_t last_print = start_time;

    compass_read();  // đọc lần đầu để khởi tạo min/max
    compass_cal_values[0] = MagX;  // min X
    compass_cal_values[1] = MagX;  // max X
    compass_cal_values[2] = MagY;  // min Y
    compass_cal_values[3] = MagY;  // max Y
    compass_cal_values[4] = MagZ;  // min Z
    compass_cal_values[5] = MagZ;  // max Z

    printf("Bat dau calibrate... Min/Max ban dau:\r\n");
    printf("X: %6d -> %6d   Y: %6d -> %6d   Z: %6d -> %6d\r\n",
           compass_cal_values[0], compass_cal_values[1],
           compass_cal_values[2], compass_cal_values[3],
           compass_cal_values[4], compass_cal_values[5]);

    while(compass_calibrated == 0)
    {
        compass_read();  // đọc dữ liệu thô

        // Cập nhật min/max
        if (MagX < compass_cal_values[0]) compass_cal_values[0] = MagX;
        if (MagX > compass_cal_values[1]) compass_cal_values[1] = MagX;
        if (MagY < compass_cal_values[2]) compass_cal_values[2] = MagY;
        if (MagY > compass_cal_values[3]) compass_cal_values[3] = MagY;
        if (MagZ < compass_cal_values[4]) compass_cal_values[4] = MagZ;
        if (MagZ > compass_cal_values[5]) compass_cal_values[5] = MagZ;

        // In tiến trình mỗi 1 giây (giảm spam terminal)
        if (HAL_GetTick() - last_print >= 1000)
        {
            last_print = HAL_GetTick();

            float Xsf_temp = (float)(compass_cal_values[1] - compass_cal_values[0]) / 2.0f;
            float Ysf_temp = (float)(compass_cal_values[3] - compass_cal_values[2]) / 2.0f;
            float Zsf_temp = (float)(compass_cal_values[5] - compass_cal_values[4]) / 2.0f;

            printf("Thoi gian: %lu s | Min/Max:\r\n", (HAL_GetTick() - start_time)/1000);
            printf("X: %6d -> %6d   Y: %6d -> %6d   Z: %6d -> %6d\r\n",
                   compass_cal_values[0], compass_cal_values[1],
                   compass_cal_values[2], compass_cal_values[3],
                   compass_cal_values[4], compass_cal_values[5]);
            printf("Scale temp: X=%.2f  Y=%.2f  Z=%.2f\r\n\r\n", Xsf_temp, Ysf_temp, Zsf_temp);
        }

        // Tính scale & offset (để in hoặc dùng sau)
        Xsf = (float)(compass_cal_values[1] - compass_cal_values[0]) / 2.0f;
        Ysf = (float)(compass_cal_values[3] - compass_cal_values[2]) / 2.0f;
        Zsf = (float)(compass_cal_values[5] - compass_cal_values[4]) / 2.0f;
        Xoff = (float)(compass_cal_values[1] + compass_cal_values[0]) / 2.0f;
        Yoff = (float)(compass_cal_values[3] + compass_cal_values[2]) / 2.0f;
        Zoff = (float)(compass_cal_values[4] + compass_cal_values[5]) / 2.0f;

        Sf_Toplam = (Xsf + Ysf + Zsf) / 3.0f;
        Xsf = Sf_Toplam / Xsf;
        Ysf = Sf_Toplam / Ysf;
        Zsf = Sf_Toplam / Zsf;

        if(Is_iBus_Received() == 1 ){
            if (iBus.RH <1100 && iBus.RV <1100) {
                compass_calibrated = 1;
                printf("Nguoi dung yeu cau ket thuc calib.\r\n");
            }
        }

        // Timeout an toàn (ví dụ 120 giây)
        if (HAL_GetTick() - start_time > 120000) {
            compass_calibrated = 1;
            printf("Timeout 120s - tu dong ket thuc calib.\r\n");
        }
    }

    // In kết quả cuối cùng
    printf("\r\n=== KET QUA CALIBRATE COMPASS ===\r\n");
    printf("Xsf = %.4f   Ysf = %.4f   Zsf = %.4f\r\n", Xsf, Ysf, Zsf);
    printf("Xoff= %6d   Yoff= %6d   Zoff= %6d\r\n", (int)Xoff, (int)Yoff, (int)Zoff);

    fill_calibration_buffer_compass();
    printf("Dang luu vao flash (Sector 10)... ");

    // Ghi flash với check lỗi
    HAL_FLASH_Unlock();
    FLASH_Erase_Sector(FLASH_SECTOR_10, FLASH_VOLTAGE_RANGE_3);

    HAL_StatusTypeDef status = HAL_OK;
    uint32_t addr = Sector14_Address;
    for(int i = 0; i < 6; i++) {
        uint32_t word;
        memcpy(&word, &calibration_buffer_float[i], sizeof(float));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word);
        if(status != HAL_OK) {
            printf("LOI ghi flash tai offset %d! Status=%d\r\n", i, status);
            break;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();

    if(status == HAL_OK) {
        printf("THANH CONG!\r\n");
    } else {
        printf("THAT BAI!\r\n");
    }
}

int compass_connect(){
	// Check connection with QMC5883L address
	while(HAL_I2C_Mem_Read(&hi2c3, QMC5883L_ADDR, 0x00, 1, data_compass, 6, 100)!=HAL_OK){
		i2c_disconnected_compass();
	}
	uint8_t data=0;

	// QMC5883L Initialization
	// 1. Reset Period (Register 0x0B) -> Recommended 0x01
	data = 0x01;
	HAL_I2C_Mem_Write(&hi2c3, QMC5883L_ADDR, 0x0B, 1, &data, 1, COMPASS_TIMEOUT);

	// 2. Control Register 1 (Register 0x09)
	// Mode: Continuous (01), ODR: 200Hz (11), RNG: 8G (01), OSR: 512 (00) -> 0x1D
	data = 0x1D;
	HAL_I2C_Mem_Write(&hi2c3, QMC5883L_ADDR, 0x09, 1, &data, 1, COMPASS_TIMEOUT);

	return 0;

}

