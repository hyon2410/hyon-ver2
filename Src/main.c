/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Quaternion.h"
#include "FS-iA6B.h"
#include <string.h>
#include "PID control.h"
# include <stdio.h>
#include "log_to_flash.h"
#include "mpu6050.h"
#include "compass.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int _write(int file, char* p, int len)
{
	for(int i=0;i<len;i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, *(p+i));
	}
	return len;
}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Sector5_Address 0x08020000   // - 0x0803 FFFF 128 Kbyte
#define Sector6_Address 0x08040000   // - 0x0805 FFFF 128 Kbyte
#define Sector7_Address 0x08060000   // - 0x0807 FFFF 128 Kbyte
#define Sector8_Address 0x08080000   // - 0x0809 FFFF 128 Kbyte
#define Sector9_Address 0x080A0000   // - 0x080B FFFF 128 Kbyte
#define Sector10_Address 0x080C0000  // - 0x080D FFFF 128 Kbyte
#define Sector11_Address 0x080E0000  // - 0x080F FFFF 128 Kbyte

// Remapped for F405 (Max Sector 11)
#define Sector14_Address 0x080C0000  // Use Sector 10 for Compass
#define Sector15_Address 0x080E0000  // Use Sector 11 for IMU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
///////////////////MPU6050 RELATED VARIABLES//////////////////
MPU6050_t MPU6050_1;
double angle_pitch_output, angle_roll_output , angle_yaw_rate_output;
extern float gyro_roll_input,gyro_pitch_input,gyro_yaw_input;
extern float accel_input[4];
extern float accel_output[4];
////////////////CALIBRATION RELATED VARIABLES/////////////////
float calibration_buffer_float[6];
float imu_calibration_values[6];
//////////////COMPASS & GPS RELATED VARIABLES////////////////

uint8_t gps_buffer[100];
int16_t MagX,MagY,MagZ;
uint8_t data_compass[13];
uint8_t MAGZ[6];
float heading;
HAL_StatusTypeDef result_compass;
int16_t compass_offset_y;
float compass_scale_y;
int16_t compass_offset_z;
float compass_scale_z;
int16_t compass_offset_x;
int16_t compass_cal_values[6];
float Xsf,Ysf,Xoff,Yoff,Zsf,Zoff;
extern float heading_corrected;
int gps_connected=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t uart3_rx_flag;
extern uint8_t uart3_rx_data;


extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;

extern uint8_t uart1_rx_flag;
extern uint8_t uart1_rx_data;

extern uint8_t tim7_1ms_flag;
extern uint8_t tim7_10ms_flag;
extern uint8_t tim7_20ms_flag;
extern uint8_t tim7_100ms_flag;
extern uint8_t tim7_200ms_flag;
extern uint8_t tim7_1000ms_flag;

unsigned char failsafe_flag = 0;
unsigned char low_bat_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int Is_iBus_Throttle_Min(void);
void ESC_Calibration(void);
int Is_iBus_Received(void);
void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	unsigned char motor_arming_flag = 0;
	unsigned short iBus_SwA_Prev = 0;
	unsigned char iBus_rx_cnt = 0;
	unsigned short ccr1, ccr2, ccr3, ccr4;

	float yaw_heading_reference;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
 // MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  LL_TIM_EnableCounter(TIM3); //Buzzer

//  LL_TIM_EnableIT_UPDATE(TIM4);     // bật interrupt overflow
//  LL_TIM_EnableCounter(TIM4);       // bật timer
//  NVIC_EnableIRQ(TIM4_IRQn);

  LL_USART_EnableIT_RXNE(USART1); //Debug UART
 // LL_USART_EnableIT_RXNE(UART4); //GPS
  LL_USART_EnableIT_RXNE(USART3); //FS-iA6B

  LL_TIM_EnableCounter(TIM5); //Motor PWM
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);

  LL_TIM_EnableCounter(TIM7); //10Hz, 50Hz, 1kHz loop
  LL_TIM_EnableIT_UPDATE(TIM7);


  TIM3->PSC = 1000;
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  HAL_Delay(60);
  printf("Checking sensor connection..\n");

  while (MPU6050_Init_Benim(&hi2c2) == 1) {
      printf("MPU6050 init failed, retry...\r\n");
      HAL_Delay(200);
  }
  	  printf("All sensors OK!\n\n");

  	  printf("Loading PID Gain...\n");
  	  PID_Init();
  	  printf("\nAll gains OK!\n\n");





  while(Is_iBus_Received() == 0)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  TIM3->PSC = 3000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  HAL_Delay(200);
  }


  if(iBus.SwC == 2000)
  {
	  ESC_Calibration();
	  while(iBus.SwC != 1000)
	  {
		  Is_iBus_Received();
	  }
  }


  while(Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  TIM3->PSC = 1000;
	  HAL_Delay(70);
  }

  /* ========= NORMAL STARTUP ========= */
      read_calib_value(Sector15_Address,6,imu_calibration_values);
      MPU6050_1.pitch_calibration_value = imu_calibration_values[0];
      MPU6050_1.roll_calibration_value  = imu_calibration_values[1];
      MPU6050_1.yaw_calibration_value   = imu_calibration_values[2];
      MPU6050_1.Accel_X_Raw_Offset      = imu_calibration_values[3];
      MPU6050_1.Accel_Y_Raw_Offset      = imu_calibration_values[4];
      MPU6050_1.Accel_Z_Raw_Rate        = imu_calibration_values[5];

      read_calib_value(Sector14_Address,6,imu_calibration_values);
      Xsf  = imu_calibration_values[0];
      Ysf  = imu_calibration_values[1];
      Xoff = imu_calibration_values[2];
      Yoff = imu_calibration_values[3];
      Zsf  = imu_calibration_values[4];
      Zoff = imu_calibration_values[5];

      printf("Compass connect...\r\n");
      while(compass_connect());

      /* ========= CALIBRATE MPU6050 ========= */
      if(iBus.LH > 1800 && iBus.LV < 1100)
      {
          HAL_Delay(2000);
          printf("BAT DAU CALIBRATE MPU6050...\r\n");
          printf("Giu drone im 10-15 giay...\r\n");

          calibrate_mpu6050();

          printf("CALIBRATE XONG! Khoi dong lai drone.\r\n");
          while(1); // bắt reboot
      }
      /* ========= CALIBRATE COMPASS ========= */
      else if(iBus.RH > 1800 && iBus.RV > 1800)
      {
          printf("\r\n=== BAT DAU CALIBRATE COMPASS ===\r\n");
          printf("Xoay drone theo moi huong 20-60 giay...\r\n");

          read_calib_value(Sector15_Address,6,imu_calibration_values);
          MPU6050_1.pitch_calibration_value = imu_calibration_values[0];
          MPU6050_1.roll_calibration_value  = imu_calibration_values[1];
          MPU6050_1.yaw_calibration_value   = imu_calibration_values[2];
          MPU6050_1.Accel_X_Raw_Offset      = imu_calibration_values[3];
          MPU6050_1.Accel_Y_Raw_Offset      = imu_calibration_values[4];
          MPU6050_1.Accel_Z_Raw_Rate        = imu_calibration_values[5];

          printf("Compass connect...\r\n");
          while(compass_connect());

          compas_calibrate();

          printf("=== CALIBRATE COMPASS XONG ===\r\n");
          printf("Khoi dong lai drone.\r\n");
          while(1); // bắt reboot
      }

  printf("Start\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if(tim7_1ms_flag == 1)
	  {
		  tim7_1ms_flag = 0;
		  MPU6050_Read_All_Benim(&hi2c2, &MPU6050_1);

				Double_Roll_Pitch_PID_Calculation(&pitch, (iBus.RV - 1500) * 0.1f , angle_pitch_output, gyro_pitch_input);
				Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - 1500) * 0.1f , angle_roll_output , gyro_roll_input);


		  if(iBus.LV < 1030 || motor_arming_flag == 0)
		  {
			  Reset_All_PID_Integrator();
		  }

		  if(iBus.LH < 1485 || iBus.LH > 1515)
		  {
			  yaw_heading_reference = heading_corrected;
			  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH - 1501), gyro_yaw_input);

			  ccr1 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result ;
			  ccr2 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result ;
			  ccr3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result ;
			  ccr4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result ;
		  }
		  else
		  {
			  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, heading_corrected, gyro_yaw_input);

			  ccr1 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result ;
			  ccr2 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result ;
			  ccr3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result ;
			  ccr4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result ;
		  }

}

	  if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
	  {
		  if(iBus.LV < 1010)
		  {
			  motor_arming_flag = 1;
			  yaw_heading_reference = heading_corrected;
		  }
		  else
		  {
			  while(Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000)
			  {
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

				  TIM3->PSC = 1000;
				  HAL_Delay(70);
				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  HAL_Delay(70);
			  }
		  }
	  }
	  iBus_SwA_Prev = iBus.SwA;

	  if(iBus.SwA != 2000)
	  {
		  motor_arming_flag = 0;
	  }

	  if(motor_arming_flag == 1)
	  {
		  if(failsafe_flag == 0)
		  {
			  if(iBus.LV > 1030)
			  {
				  TIM5->CCR1 = ccr1 > 21000 ? 21000 : ccr1 < 11000 ? 11000 : ccr1;
				  TIM5->CCR2 = ccr2 > 21000 ? 21000 : ccr2 < 11000 ? 11000 : ccr2;
				  TIM5->CCR3 = ccr3 > 21000 ? 21000 : ccr3 < 11000 ? 11000 : ccr3;
				  TIM5->CCR4 = ccr4 > 21000 ? 21000 : ccr4 < 11000 ? 11000 : ccr4;
			  }
			  else
			  {
				  TIM5->CCR1 = 11000;
				  TIM5->CCR2 = 11000;
				  TIM5->CCR3 = 11000;
				  TIM5->CCR4 = 11000;
			  }
		  }
		  else
		  {
			  TIM5->CCR1 = 10500;
			  TIM5->CCR2 = 10500;
			  TIM5->CCR3 = 10500;
			  TIM5->CCR4 = 10500;
		  }
	  }
	  else
	  {
		  TIM5->CCR1 = 10500;
		  TIM5->CCR2 = 10500;
		  TIM5->CCR3 = 10500;
		  TIM5->CCR4 = 10500;
	  }
	  compass_read_corrected();

	  if(ibus_rx_cplt_flag == 1)
	  {
		  ibus_rx_cplt_flag = 0;
		  if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		  {
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);

			  iBus_Parsing(&ibus_rx_buf[0], &iBus);
			  iBus_rx_cnt++;
			  if(iBus_isActiveFailsafe(&iBus) == 1)
			  {
				  failsafe_flag = 1;
			  }
			  else
			  {
				  failsafe_flag = 0;
			  }
//		 printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
//					  iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwC, iBus.SwB, iBus.SwD, iBus.VrA, iBus.VrB);
//			  HAL_Delay(100);
		  }
	  }

	  if(tim7_1000ms_flag == 1)
	  {
		  tim7_1000ms_flag = 0;
		  if(iBus_rx_cnt == 0)
		  {
			  failsafe_flag = 2;
		  }
		  iBus_rx_cnt = 0;
	  }
//	  if(tim7_100ms_flag == 1)
//	  {
//		  tim7_100ms_flag = 0;
//	  printf("SUMMARY: P=%.2f R=%.2f H=%.2f YR=%.2f Pv=%.2f Rv=%.2f Yv=%.2f \r\n",
//	 		  	                 angle_pitch_output, angle_roll_output, heading_corrected, angle_yaw_rate_output,gyro_pitch_input,gyro_roll_input, gyro_yaw_input );
//
//	  }
	  if(failsafe_flag == 1 || failsafe_flag == 2 ||  iBus.SwC == 2000) //low_bat_flag == 1 ||
	  {
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
	  else
	  {
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int Is_iBus_Throttle_Min(void)
{
	if(ibus_rx_cplt_flag == 1)
	{
		ibus_rx_cplt_flag = 0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			if(iBus.LV < 1010) return 1;
		}
	}

	return 0;
}

void ESC_Calibration(void)
{
	  TIM5->CCR1 = 21000;
	  TIM5->CCR2 = 21000;
	  TIM5->CCR3 = 21000;
	  TIM5->CCR4 = 21000;
	  HAL_Delay(7000);
	  TIM5->CCR1 = 10500;
	  TIM5->CCR2 = 10500;
	  TIM5->CCR3 = 10500;
	  TIM5->CCR4 = 10500;
	  HAL_Delay(8000);
}

int Is_iBus_Received(void)
{
	if(ibus_rx_cplt_flag == 1)
	{
		ibus_rx_cplt_flag = 0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			return 1;
		}
	}

	return 0;
}







/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
