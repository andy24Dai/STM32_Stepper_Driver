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

float TICK_PERIOD = 0.000015625; // s/tick

float position_kP = 0.00024;
float MOTOR_MAX_SPEED = 0.3; // steps/tick
float MOTOR_MAX_ACCEL = 1.6; // steps/tick/s

float MOTOR_MAX_ANGLE = 70.; // degrees
float MOTOR_MIN_ANGLE = -30; // degrees



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

float Heaviside(float num){
	return (num > 0) ? num : 0;
}

typedef struct{
	float accel; // steps/tick/ms
	float velocity_cruise;
	int steps_curr;
	int steps_total;
	int steps_accel;
	int steps_decel;
} TrapProfile ;

TrapProfile CreateTrapProfileDefault(){
	return (TrapProfile){MOTOR_MAX_ACCEL, 0, 0, 0, 0, 0};
}

typedef struct {
	GPIO_TypeDef* step_port;
	GPIO_TypeDef* dir_port;
	uint16_t step_pin;
	uint16_t dir_pin;
	float step_accum;
	float velocity_current;
	int direction;
	int position_current;
	int position_target;
	TrapProfile* profile;
} StepperMotor;

StepperMotor CreateMotorDefault(GPIO_TypeDef* step_port, GPIO_TypeDef* dir_port,
		uint16_t step_pin, uint16_t dir_pin, TrapProfile* profile){
	return (StepperMotor){step_port, dir_port, step_pin, dir_pin, 0, 0, 0, 0, 0, profile};
}

TrapProfile profile1;
TrapProfile profile2;
TrapProfile profile3;

StepperMotor motor1;
StepperMotor motor2;
StepperMotor motor3;

// FUNCTION DEFINES

void Step_Motor(StepperMotor* motor, uint8_t* flag);

// UPDATES
void Update_Positions(float ang1, float ang2, float ang3);
void Update_Stepper(StepperMotor* motor, int position);
void Update_Stepper_Degrees(StepperMotor* motor, float position);
void Update_Stepper_Speed(StepperMotor* motor);
void Update_Profiles(int steps1, int steps2, int steps3);


// UTIL

void Generate_Profile(StepperMotor* motor, float error, float speed, float accel);
int Degrees_to_Steps(float angle);
float Steps_to_Degrees(int steps);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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

  profile1 = CreateTrapProfileDefault();
  profile2 = CreateTrapProfileDefault();
  profile3 = CreateTrapProfileDefault();

//  motor1 = CreateMotorDefault(GPIOA, GPIOA, GPIO_PIN_9, GPIO_PIN_10, &profile1);
//  motor2 = CreateMotorDefault(GPIOA, GPIOB, GPIO_PIN_15, GPIO_PIN_3, &profile2);
//  motor3 = CreateMotorDefault(GPIOB, GPIOB, GPIO_PIN_4, GPIO_PIN_5, &profile3);

  motor1 = CreateMotorDefault(GPIOA, GPIOA, 9, 10, &profile1);
  motor2 = CreateMotorDefault(GPIOA, GPIOB, 15, 3, &profile2);
  motor3 = CreateMotorDefault(GPIOB, GPIOB, 4, 5, &profile3);

  uint32_t currTime = 0;
  uint32_t prevTimeStepUpdate = 0;
  uint32_t prevTimeProfileFollow = 0;
  uint32_t prevTimeTransmit = 0;

  motor1.position_current = Degrees_to_Steps(-34);
  motor2.position_current = Degrees_to_Steps(-34);
  motor3.position_current = Degrees_to_Steps(-34);

  motor1.position_target = Degrees_to_Steps(-34);
  motor2.position_target = Degrees_to_Steps(-34);
  motor3.position_target = Degrees_to_Steps(-34);


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

	if(currTime - prevTimeProfileFollow > UPDATE_PROFILE_PERIOD){
		
		Update_Stepper_Speed(&motor1);
		Update_Stepper_Speed(&motor2);
		Update_Stepper_Speed(&motor3);
		
		prevTimeProfileFollow = currTime;
	}

	if(currTime - prevTimeTransmit > 17){
		if(motor1.position_current != motor1.position_target){
		usb_tx_buf_len = snprintf((char*) usb_tx_buffer, USB_BUF_LEN,
				"vi %f a %f ad %f vc %f pc %f pt %f %d %d %d %d \r\n",
				motor1.velocity_current,
				motor1.profile->accel,
				(motor1.velocity_current*motor1.velocity_current)/
				(2.0f*((float)(motor1.profile->steps_total - motor1.profile->steps_curr)))/
				(1000.f*TICK_PERIOD),
				motor1.profile->velocity_cruise,
				Steps_to_Degrees(motor1.position_current),
				Steps_to_Degrees(motor1.position_target),
				motor1.profile->steps_accel,
				motor1.profile->steps_decel,
				motor1.profile->steps_curr,
				motor1.profile->steps_total);
		CDC_Transmit_FS(usb_tx_buffer, usb_tx_buf_len);

		}
		prevTimeTransmit = currTime;
	}

	if(rx_flag && usb_rx_buf_len){

		usb_tx_buf_len = snprintf((char*) usb_tx_buffer, USB_BUF_LEN, "%0.2f %0.2f %0.2f\r\n", target_setpoints[0] ,target_setpoints[1], target_setpoints[2]);
		CDC_Transmit_FS(usb_tx_buffer, usb_tx_buf_len);

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

			motor->profile->steps_curr++;

			if(motor->position_current == motor->position_target){
				 motor->step_port->BSRR = (1 << (motor->step_pin+16));
				 *flag = 0;
			}
		}
	}

}

void Update_Stepper(StepperMotor* motor, int new_target){
	motor->position_target = new_target;

}

void Update_Stepper_Degrees(StepperMotor* motor, float new_target){
	int new_step_target = Degrees_to_Steps(new_target);
	Update_Stepper(motor, new_step_target);

}

void Update_Stepper_Speed(StepperMotor* motor){

	if (motor->position_current == motor->position_target) {
		motor->velocity_current = 0;
		return;
	}

	float accelDir = ((motor->profile->velocity_cruise - motor->velocity_current) > 0) ? 1. : -1.;

	if(motor->profile->steps_curr > motor->profile->steps_total){
		motor->velocity_current = 0;
		motor->position_target = motor->position_current;
	}

	// if motor is accelerating
	else if(motor->profile->steps_curr <= motor->profile->steps_accel){
		motor->velocity_current += motor->profile->accel * (float) UPDATE_PROFILE_PERIOD;
	}

	// if motor is decelerating
	else if(motor->profile->steps_curr >= motor->profile->steps_total - motor->profile->steps_decel){
		float dir = (motor->velocity_current > 0) ? 1.f : -1.f;
		float steps_extra = (fabsf(motor->velocity_current)*(float)UPDATE_PROFILE_PERIOD)/(1000.f*TICK_PERIOD);
		float steps_remaining = (float)(motor->profile->steps_total - motor->profile->steps_curr - (int)steps_extra) ;
		float new_accel = (motor->velocity_current*motor->velocity_current)/(2.0f*steps_remaining);
		new_accel = new_accel/(1000.f*TICK_PERIOD); //convert to steps/tick/ms

		float new_velocity = motor->velocity_current - dir * new_accel * (float) UPDATE_PROFILE_PERIOD;
		if((new_velocity * motor->profile->velocity_cruise) < 0.0f){
			new_velocity = 0.0f;
		}
		motor->velocity_current = new_velocity;
	}

	// motor is cruising
	else{
		motor->velocity_current = motor->profile->velocity_cruise;
	}
}



void Update_Positions(float ang1, float ang2, float ang3){

	float ang1Lim = ang1;//fmaxf(fminf(ang1, MOTOR_MAX_ANGLE), MOTOR_MIN_ANGLE);
	float ang2Lim = ang2;//fmaxf(fminf(ang2, MOTOR_MAX_ANGLE), MOTOR_MIN_ANGLE);
	float ang3Lim = ang3;//fmaxf(fminf(ang3, MOTOR_MAX_ANGLE), MOTOR_MIN_ANGLE);

	int steps1 = Degrees_to_Steps(ang1Lim);
	int steps2 = Degrees_to_Steps(ang2Lim);
	int steps3 = Degrees_to_Steps(ang3Lim);

	Update_Profiles(steps1, steps2, steps3);

	Update_Stepper(&motor1, steps1);
	Update_Stepper(&motor2, steps2);
	Update_Stepper(&motor3, steps3);

}

void Update_Profiles(int steps1, int steps2, int steps3){

	float err1 = (float) (steps1 - motor1.position_current);
	float err2 = (float) (steps2 - motor2.position_current);
	float err3 = (float) (steps3 - motor3.position_current);

	float maxErr = fmaxf(fmaxf(fabsf(err1), fabsf(err2)), fabsf(err3));
	float maxSpeed = fminf((MOTOR_MAX_SPEED/(MOTOR_MAX_ANGLE - MOTOR_MIN_ANGLE)) * maxErr, MOTOR_MAX_SPEED) + 0.01;

	if(maxErr > 0){
		Generate_Profile(&motor1, err1, maxSpeed*(err1/maxErr), MOTOR_MAX_ACCEL*(err1/maxErr));
		Generate_Profile(&motor2, err2, maxSpeed*(err2/maxErr), MOTOR_MAX_ACCEL*(err2/maxErr));
		Generate_Profile(&motor3, err3, maxSpeed*(err3/maxErr), MOTOR_MAX_ACCEL*(err3/maxErr));
	}

}

void Generate_Profile(StepperMotor* motor, float error, float cruise_velocity, float new_accel){
	motor->profile->accel = new_accel/1000.f; // steps/tick/ms
	motor->profile->velocity_cruise = cruise_velocity;
	motor->profile->steps_curr = 0;

	float curr_velocity = motor->velocity_current;
	float accel_ticks = fabsf(new_accel*TICK_PERIOD); //steps/tick^2

	int errorInt = abs((int) error);

	int steps_accel;
	int steps_deccel;


	//TODO add extra steps based on direction
	if(1){//(curr_velocity*cruise_velocity) > 0){ // if velocities in same direction
		motor->profile->steps_total = errorInt;
		steps_accel = (int)((cruise_velocity*cruise_velocity - curr_velocity*curr_velocity)/(2.f*accel_ticks));
		steps_deccel = (int)((cruise_velocity*cruise_velocity)/(2.f*accel_ticks));

		if(steps_accel + steps_deccel > errorInt){
			steps_accel = (int)((fabsf(error)/2.f) - ((curr_velocity*curr_velocity)/(4.f*accel_ticks)));
			steps_deccel = errorInt - steps_accel;
		}
	}
	else{
//		int steps_zero = (int)((curr_velocity*curr_velocity)/(2.f*accel_ticks));
//		steps_accel = (int)((cruise_velocity*cruise_velocity)/(2.f*accel_ticks));
//		steps_deccel = steps_accel;//(int)((cruise_velocity*cruise_velocity)/(2.f*accel_ticks));
//
//		if(steps_accel + steps_deccel > errorInt){
//			steps_accel = (int)(((float) fabsf(error) )/2.f - (curr_velocity*curr_velocity)/(4.f*accel_ticks));
//			steps_deccel = errorInt - steps_accel;
//		}
//
//		steps_accel += steps_zero;
//
//		motor->profile->steps_total = errorInt + steps_zero;

	}

	motor->profile->steps_accel = abs(steps_accel);
	motor->profile->steps_decel = abs(steps_deccel);
}

int Degrees_to_Steps(float angle){
	return (int)((angle/360.)*3200.);
}

float Steps_to_Degrees(int steps){
	return (steps/3200.) * 360;
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
