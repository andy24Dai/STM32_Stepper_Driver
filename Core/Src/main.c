/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BUF_LEN 128
#define UPDATE_PROFILE_PERIOD 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t usb_tx_buffer[USB_BUF_LEN];
uint16_t usb_tx_buf_len;

uint8_t usb_rx_buffer[USB_BUF_LEN];
uint16_t usb_rx_buf_len;
uint8_t rx_flag = 0;

float target_setpoints[3];
float current_setpoints[3];

float TICK_PERIOD = 0.015625; // ms/tick
float TICKS_PER_UPDATE;

float position_kP = 0.00024;
float MOTOR_MAX_SPEED = 0.3; // steps/tick
float MOTOR_MAX_ACCEL = 3.f; // steps/tick/s

float MOTOR_MAX_ANGLE = 70.; // degrees
float MOTOR_MIN_ANGLE = -30; // degrees
float MOTOR_MAX_ERR_STEPS;
float MOTOR_MAX_ERR_ACCEL_STEPS;

float stupid_debug_variable = 0;
float stupid_debug_variable2 = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void USB_RX_Callback(uint8_t* Buf, uint32_t *Len){
	memcpy(usb_rx_buffer, Buf, *Len);
	usb_rx_buf_len = *Len;
	rx_flag = 1;

	usb_rx_buffer[*Len] = '\0';


	char* token = strtok((char*) usb_rx_buffer, " ");
	target_setpoints[0] = atoff(token);

	token = strtok(NULL, " ");
	target_setpoints[1] = atoff(token);

	token = strtok(NULL, " ");
	target_setpoints[2] = atoff(token);

}

float sign(float n){
	return (n > 0) ? 1.f : -1.f;
}

int Degrees_to_Steps(float angle){
	return (int)((angle/360.)*3200.);
}

float Steps_to_Degrees(int steps){
	return (steps/3200.) * 360;
}

float Ticks_to_Ms(float ticks){
	return ticks*TICK_PERIOD;
}

float Ms_to_Ticks(float ms){
	return ms/TICK_PERIOD;
}



typedef struct {
	GPIO_TypeDef* step_port;
	GPIO_TypeDef* dir_port;
	uint16_t step_pin;
	uint16_t dir_pin;
	float step_accum;
	float velocity_current;
	float speed_max;
	float accel;
	int direction;
	int position_current;
	int position_target;
	int total_error;
} StepperMotor;

StepperMotor CreateMotorDefault(GPIO_TypeDef* step_port, GPIO_TypeDef* dir_port,
		uint16_t step_pin, uint16_t dir_pin){
	return (StepperMotor){
		step_port,
		dir_port,
		step_pin,
		dir_pin,
		0,
		0,
		MOTOR_MAX_SPEED,
		MOTOR_MAX_ACCEL,
		0,
		0,
		0,
		0};
}

StepperMotor motor1;
StepperMotor motor2;
StepperMotor motor3;

// FUNCTION DEFINES

void Step_Motor(StepperMotor* motor, uint8_t* flag);

// UPDATES
void Update_Positions(float ang1, float ang2, float ang3);
void Update_Stepper(StepperMotor* motor, int new_target, float max_speed, float accel);
void Update_Stepper_Degrees(StepperMotor* motor, float position);
void Update_Stepper_Speeds();
void Update_Stepper_Speed(StepperMotor* motor, int* decel_steps, uint8_t* calc_flag);
void Update_Profiles(int steps1, int steps2, int steps3);


// UTIL

void Generate_Profile(StepperMotor* motor, float error, float speed, float accel);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	TICKS_PER_UPDATE = ((float) UPDATE_PROFILE_PERIOD) / TICK_PERIOD;

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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim2);

  motor1 = CreateMotorDefault(GPIOA, GPIOA, 9, 10);
  motor2 = CreateMotorDefault(GPIOA, GPIOB, 15, 3);
  motor3 = CreateMotorDefault(GPIOB, GPIOB, 4, 5);

  uint32_t currTime = 0;
  uint32_t prevTimeProfileFollow = 0;
  uint32_t prevTimeTransmit = 0;

  motor1.position_current = Degrees_to_Steps(-32.78);
  motor2.position_current = Degrees_to_Steps(-32.78);
  motor3.position_current = Degrees_to_Steps(-32.78);

  motor1.position_target = Degrees_to_Steps(-32.78);
  motor2.position_target = Degrees_to_Steps(-32.78);
  motor3.position_target = Degrees_to_Steps(-32.78);


  MOTOR_MAX_ERR_ACCEL_STEPS = (float) Degrees_to_Steps(53);
  MOTOR_MAX_ERR_STEPS = (float) (Degrees_to_Steps(MOTOR_MAX_ANGLE) - Degrees_to_Steps(MOTOR_MIN_ANGLE));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	currTime = HAL_GetTick();



//	if(currTime - prevTimeStepUpdate > 4000){
////			Update_Positions(-30,70,70);
//
//		Update_Positions(Steps_to_Degrees(motor1.position_current) + 10,
//		Steps_to_Degrees(motor2.position_current) + 180.,
//		Steps_to_Degrees(motor3.position_current) + 45.);
//		prevTimeStepUpdate = currTime;
//	}
//
//	else if (currTime - prevTimeStepUpdate > 3000){
//		Update_Positions(-180,-180,-180);
//	}
//
//	else if (currTime - prevTimeStepUpdate > 2000){
//			Update_Positions(70,-10,0);
//	}
//
//	else if (currTime - prevTimeStepUpdate > 1000){
//		Update_Positions(180,180,180);
//	}

	if(currTime - prevTimeProfileFollow >= UPDATE_PROFILE_PERIOD){
		
		Update_Stepper_Speeds();
		
		prevTimeProfileFollow = currTime;
	}

	if(currTime - prevTimeTransmit >= 50){
		if(1){//motor1.position_current != motor1.position_target){
			usb_tx_buf_len = snprintf((char*) usb_tx_buffer, USB_BUF_LEN,
					"vi %f vm %f pc %d pt %d debug: %f %f\r\n",
					motor1.velocity_current,
					motor1.speed_max,
					motor1.position_current,
					motor1.position_target,
					stupid_debug_variable,
					stupid_debug_variable2);
			CDC_Transmit_FS(usb_tx_buffer, usb_tx_buf_len);
		}
		prevTimeTransmit = currTime;
	}

	if(rx_flag && usb_rx_buf_len){

		//usb_tx_buf_len = snprintf((char*) usb_tx_buffer, USB_BUF_LEN, "%0.2f %0.2f %0.2f\r\n", target_setpoints[0] ,target_setpoints[1], target_setpoints[2]);
		//CDC_Transmit_FS(usb_tx_buffer, usb_tx_buf_len);

		Update_Positions(target_setpoints[0], target_setpoints[1], target_setpoints[2]);

		rx_flag = 0;
		usb_rx_buf_len = 0;

		prevTimeTransmit = currTime;

	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 374;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */


  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	static uint8_t step_flag_1;
	static uint8_t step_flag_2;
	static uint8_t step_flag_3;

	Step_Motor(&motor1, &step_flag_1);
	Step_Motor(&motor2, &step_flag_2);
	Step_Motor(&motor3, &step_flag_3);
}

void Step_Motor(StepperMotor* motor, uint8_t* flag){

	if(motor->position_current != motor->position_target){

		motor->step_accum += fabsf(motor->velocity_current);

		if(motor->velocity_current > 0){
			motor->dir_port->BSRR = (1 << motor->dir_pin);
			motor->direction = 1;
		}
		else{
			motor->dir_port->BSRR = (1 << (motor->dir_pin+16));
			motor->direction = 0;
		}

		if(motor->step_accum < 1.0f){
			if(*flag){
				motor->step_port->BSRR = (1 << (motor->step_pin+16));
				*flag = 0;
			}
		}

		else{
			motor->step_accum -= 1.0f;
			*flag = 1;
			motor->step_port->BSRR = (1 << (motor->step_pin));

			if(motor->direction){
				motor->position_current++;
			} else {
				motor->position_current--;

			}

			if(motor->position_current == motor->position_target){
				 motor->step_port->BSRR = (1 << (motor->step_pin+16));
				 *flag = 0;
			}
		}
	}

}

void Update_Stepper(StepperMotor* motor, int new_target, float max_speed, float new_accel){
	motor->position_target = new_target;
	motor->speed_max = max_speed;
	motor->accel = new_accel/1000.f; // convert s to ms
	motor->total_error = abs(new_target - motor->position_current);
}

void Update_Stepper_Speeds(){
	static int decel_steps_1;
	static int decel_steps_2;
	static int decel_steps_3;

	static uint8_t calc_flag_1;
	static uint8_t calc_flag_2;
	static uint8_t calc_flag_3;

	Update_Stepper_Speed(&motor1, &decel_steps_1, &calc_flag_1);
	Update_Stepper_Speed(&motor2, &decel_steps_2, &calc_flag_2);
	Update_Stepper_Speed(&motor3, &decel_steps_3, &calc_flag_3);
}

void Update_Stepper_Speed(StepperMotor* motor, int* decel_steps, uint8_t* calc_flag){
	int curr_pos = motor->position_current;
	int targ_pos = motor->position_target;
	int curr_err = targ_pos - curr_pos;

	int tot_err_half = motor->total_error/2;

	float curr_vel = motor->velocity_current;
	float max_spd = motor->speed_max;

	float accel = motor->accel; // steps/tick/ms

	float dir = sign((float) curr_err); // direction of max vel and accel

	//motor at position
	if (curr_err == 0) {
		motor->velocity_current = 0;
	}

	//error is within first half of profile
	else if(abs(curr_err) >= tot_err_half){
		// if velocity not at max
		if((dir > 0 && curr_vel < max_spd) || (dir < 0 && curr_vel > -max_spd)){
			motor->velocity_current += dir*accel* (float) UPDATE_PROFILE_PERIOD;
		}
		else{
			motor->velocity_current = dir*max_spd;
		}

		*calc_flag = 0;

		stupid_debug_variable = -100;
		stupid_debug_variable2 = -100;
	}

	//error is within second half of profile
	else if(abs(curr_err) < tot_err_half){

		if(!*calc_flag){
			float accelTicks = fabsf(accel / TICK_PERIOD);
			*decel_steps = (int) ((curr_vel*curr_vel)/(2.f*accelTicks));

			stupid_debug_variable = *decel_steps;

			int test_steps = 0;

			float sim_curr_vel = fabsf(curr_vel);
			float sim_accum = 0.f;
			int new_steps = 0;

			while(sim_curr_vel > 0){
				sim_accum += sim_curr_vel * TICKS_PER_UPDATE;

				new_steps = (int) sim_accum;
				test_steps += new_steps;
				sim_accum -= new_steps;

				sim_curr_vel -= accel * (float) UPDATE_PROFILE_PERIOD;
			}

			stupid_debug_variable2 = test_steps;

			*decel_steps = test_steps;

			*calc_flag = 1;
		}

		// within deceleration range
		if(abs(curr_err) <= *decel_steps){
			stupid_debug_variable = (float) *decel_steps;
			if(abs(curr_err) >= 1){
				motor->velocity_current -= dir * accel * (float) UPDATE_PROFILE_PERIOD;
				if(motor->velocity_current * dir < 0){
					motor->velocity_current = 0;
					motor->position_current = motor->position_target;
				}
			}
			else{
				motor->velocity_current = 0;
				motor->position_current = motor->position_target;
			}
		}
	}

}



void Update_Positions(float ang1, float ang2, float ang3){

	float ang1Lim = fmaxf(fminf(ang1, MOTOR_MAX_ANGLE), MOTOR_MIN_ANGLE);
	float ang2Lim = fmaxf(fminf(ang2, MOTOR_MAX_ANGLE), MOTOR_MIN_ANGLE);
	float ang3Lim = fmaxf(fminf(ang3, MOTOR_MAX_ANGLE), MOTOR_MIN_ANGLE);

	int steps1 = Degrees_to_Steps(ang1Lim);
	int steps2 = Degrees_to_Steps(ang2Lim);
	int steps3 = Degrees_to_Steps(ang3Lim);

	float err1 = (float) abs(steps1 - motor1.position_current);
	float err2 = (float) abs(steps2 - motor2.position_current);
	float err3 = (float) abs(steps3 - motor3.position_current);

	float maxErr = fmaxf(fmaxf(fabsf(err1), fabsf(err2)), fabsf(err3));
	float maxSpeed = fminf((MOTOR_MAX_SPEED/(MOTOR_MAX_ERR_STEPS)) * maxErr, MOTOR_MAX_SPEED);

	if(maxErr > 0){
		Update_Stepper(&motor1, steps1, maxSpeed*(err1/maxErr),
				fminf(MOTOR_MAX_ACCEL*(err1/MOTOR_MAX_ERR_ACCEL_STEPS), MOTOR_MAX_ACCEL));
		Update_Stepper(&motor2, steps2, maxSpeed*(err2/maxErr),
				fminf(MOTOR_MAX_ACCEL*(err2/MOTOR_MAX_ERR_ACCEL_STEPS), MOTOR_MAX_ACCEL));
		Update_Stepper(&motor3, steps3, maxSpeed*(err3/maxErr),
				fminf(MOTOR_MAX_ACCEL*(err3/MOTOR_MAX_ERR_ACCEL_STEPS), MOTOR_MAX_ACCEL));
	}

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
