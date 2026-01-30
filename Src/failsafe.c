/*
 * failsafe.c
 *
 *  Created on: Feb 24, 2023
 *      Author: sahin
 */
#include "failsafe.h"
//#include "pwm_esc.h"
//#include "receiver.h"

extern I2C_HandleTypeDef hi2c2;
/// this function rescues the hang i2c bus that is connected to imu
void i2c_disconnected(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. Disable I2C */
    HAL_I2C_DeInit(&hi2c2);

    /* 2. Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 3. Configure PB10 & PB11 as Open-Drain GPIO */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 4. Release both lines HIGH */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 5. Toggle SCL 9 pulses to release slave */
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* 6. Re-init I2C */
    HAL_I2C_Init(&hi2c2);
}
// worst scenerio, shut the motors and wait for watchdog to reset the mcu
void failsafe_handler()
{
	//stop_motors();
	while(1);

}
//void red_led_on(){
//	HAL_GPIO_WritePin(GPIOB, LED3_Pin, SET);
//}
//void red_led_off(){
//	HAL_GPIO_WritePin(GPIOB, LED3_Pin, RESET);
//}
//void green_led_on(){
//	HAL_GPIO_WritePin(GPIOB, LED1_Pin, SET);
//}
//void green_led_off(){
//	HAL_GPIO_WritePin(GPIOB, LED1_Pin, RESET);
//}
//void blue_led_on(){
//	HAL_GPIO_WritePin(GPIOB, LED2_Pin, SET);
//}
//void blue_led_off(){
//	HAL_GPIO_WritePin(GPIOB, LED2_Pin, RESET);
//}

