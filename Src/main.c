/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "i2c-lcd.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN 0
#define MAX 1

#define OFF -1
#define CC 0
#define CV 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float current_A, current_ADC, voltage_V, voltage_ADC;
int accu_class, charge_mode;
double setpoint;

//PID Params
//format:
//			CC	| CV
//0=low,
//1=med,
//2=high
double KP_now, KI_now, KD_now;
const double KP[3][2]= {
    { 0.0065, 0.0065}, //CC, CV
    {0.007, 0.0092},
    {0.0005, 0.0078}
};
const double KI[3][2]= {
    { 0.03125,  0.03125},
    { 0.0375, 0.05},
    {0.05625, 0.075}
};
const double KD[3][2]= {
    {0, 0},
    {0, 0},
    {0, 0}
};

double Lerr = 0.0, Derr = 0.0, Ierr = 0.0, error=0, output = 0.0;
unsigned long prev_time, prev_lcd_time, prev_mode_time;

/* INISISALISASI USART */
unsigned int len;
int8_t buffer[100];

//Range identifikasi kelas
const double voltage_range[3][2]= {
    {3.75, 4.37},	//low
    {5.75, 6.37},	//med
    {11.51, 12.73}	//high
};

const double voltage_setpoint[3]= {
    4.8,
    7,
    13.8
};

const double voltage_syarat[3]= {
    4.5,
    6.7,
    13.5
};

const double current_setpoint[3]= {
    0.67,
    0.7,
    0.72
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
float getADCAvg(uint32_t channel, int sample, ADC_HandleTypeDef* hadc);
double PID(double input);
void reset_error();
void baca_sensor();
void print_lcd();
void drive_sepic(float percent);
void data_logger();
float adcToVolt(float adc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float pwmval=0;
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
    MX_ADC1_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    //start timer 4 (pwm)
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);

    //LCD
    HAL_Delay(50);
    lcd_init();
    HAL_Delay(50);
	
	lcd_clear();

    lcd_send_cmd(0x80 | 0x40); //  lcd_setCursor(0,1)
    len = snprintf((char *)buffer, 50, "Battery Charging");
    lcd_send_string((char *)buffer);
    HAL_Delay(2000);

    //start pwm
    drive_sepic(0);
    accu_class=-1;

    //stop sistem
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
    charge_mode = OFF;

    int len = snprintf((char*)buffer, 100,"CLEARDATA\r\nLABEL,TIME, Started Time,Data Register1, data2\r\nRESETTIMER");

    prev_lcd_time = HAL_GetTick();
    prev_mode_time = HAL_GetTick();

    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);//NYALA

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        baca_sensor();
        print_lcd();

        /* IDENTIFIKASI JENIS AKI*/
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==GPIO_PIN_RESET) {
//			pwmval+=0.1;
//			drive_sepic(pwmval);
			
			lcd_clear();
			//baris 1
			lcd_send_cmd(0x80 | 0x00); //  lcd_setCursor(0,0);
			len = snprintf((char *)buffer, 50, "Cek Jenis Baterai");
			lcd_send_string((char *)buffer);
			
            // Stop sistem
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
            pwmval=0;
            drive_sepic(pwmval);
            HAL_Delay(5000);
            charge_mode=OFF;
            baca_sensor();
            reset_error();

            if(voltage_V > voltage_range[0][MIN] && voltage_V < voltage_range[0][MAX]) accu_class=0;
            else if(voltage_V > voltage_range[1][MIN] && voltage_V < voltage_range[1][MAX]) accu_class=1;
            else if(voltage_V > voltage_range[2][MIN] && voltage_V < voltage_range[2][MAX]) accu_class=2;
            else accu_class=-1;
        }

        /* CHARGING AKI */
        else {
            /* JENIS AKI TELAH TERIDENTIFIKASI */
            if (accu_class != -1)
            {
                /* TENTUKAN MODE CHARGING*/
                // Tegangan belum memenuhi setpoint ==> CC Charging
                if(voltage_V < voltage_syarat[accu_class]) {
                    if(charge_mode!=CC) {
						//reset_error();
						charge_mode=CC;
						prev_time=HAL_GetTick();
						prev_mode_time=HAL_GetTick();
						//Ierr=7;

						setpoint=current_setpoint[accu_class];
						KP_now=KP[accu_class][CC];
						KI_now=KI[accu_class][CC];
						KD_now=KD[accu_class][CC];

						// Start sistem
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
                    }
					output = PID(current_A);
					pwmval=output;
					drive_sepic(output);
                }
                // Arus masih besar (diatas 10% dari setpoint) ==> CV Charging
                else if(current_A > current_setpoint[accu_class]*0.1) {
                    if(charge_mode!=CV) {
						//reset_error();
						prev_time=HAL_GetTick();
						prev_mode_time=HAL_GetTick();
						charge_mode=CV;
						//Ierr=13;

						setpoint=voltage_setpoint[accu_class];
						KP_now=KP[accu_class][CV];
						KI_now=KI[accu_class][CV];
						KD_now=KD[accu_class][CV];

						// Start sistem
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
                    }
					output = PID(voltage_V);
					pwmval=output;
					drive_sepic(output);
                }
                // Arus sudah kecil
                else {
                    // Stop sistem
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
                    charge_mode=OFF;
                    HAL_Delay(5000);
                }
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
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

    /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 2099;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 5000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

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
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                      |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                      |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                      |GPIO_PIN_7, GPIO_PIN_RESET);

    /*Configure GPIO pins : PE2 PE10 PE11 PE12
                             PE14 PE0 PE1 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : PA3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PC4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PE13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PD10 PD0 PD1 PD2
                             PD3 PD4 PD5 PD6
                             PD7 */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
double PID(double input)
{
    double output_pid;
    unsigned long now = HAL_GetTick();

    double delta_time = (now - prev_time) / 1000.0;
    if (delta_time == 0) {
        // Handle the case when delta_time is zero, for example, by setting it to a small value
        delta_time = 0.001;  // or any other small value that makes sense in your application
    }
    prev_time = now;

    error = setpoint - input;

    // Error
    Derr = (error - Lerr) / delta_time;
    Ierr += (error * delta_time);
    Lerr = error;  // Simpan error saat ini untuk  iterasi berikutnya

    // Anti-windup
    if(Ierr > 15) Ierr=15;

    output_pid = (KP_now * error) + (KI_now * Ierr) + (KD_now * Derr);

    // CLAMPING 0.1-0.55
    if (output_pid > 0.55) {
        output_pid = 0.55;
    } else if (output_pid < 0.1) {
        output_pid = 0.1;
    }
    return output_pid;
}

void reset_error() {
    Lerr = 0.0;
    Derr = 0.0;
    Ierr = 0.0;
}

void baca_sensor() {
    current_ADC= getADCAvg(ADC_CHANNEL_2, 50, &hadc1);
    current_A= adcToVolt(fabs(3696.0-current_ADC))/0.185;

    voltage_ADC= getADCAvg(ADC_CHANNEL_0, 50, &hadc1);
    //voltage_V= (0.0081*voltage_ADC) -0.0175 ;
    voltage_V=(0.008187*voltage_ADC) + 0.0004849;
}

float getADCAvg(uint32_t channel, int sample, ADC_HandleTypeDef* hadc)
{
    /*
     * channel = ADC_CHANNEL_0
     */
    //KONFIGURASI ADC
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(75);
    //HAL_ADCEx_Calibration_Start(hadc);

    //SAMPLING ADC
    float adcVal = 0;
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);

    for (int i = 0; i < sample; i++)
    {
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 100);
        adcVal += HAL_ADC_GetValue(hadc);
    }
    HAL_ADC_Stop(hadc);
    adcVal /= sample;
    return adcVal;
}
void print_lcd() {
    if(HAL_GetTick() - prev_lcd_time > 500) {
        lcd_clear();
        //baris 1
        lcd_send_cmd(0x80 | 0x00); //  lcd_setCursor(0,0);
        len = snprintf((char *)buffer, 50, "Teg:%.2fV  ADC:%.f", voltage_V, voltage_ADC);
        lcd_send_string((char *)buffer);

        //baris 2
        lcd_send_cmd(0x80 | 0x40); //  lcd_setCursor(0,1)
        len = snprintf((char *)buffer, 50, "Arus:%.2fA ADC:%.f", current_A, current_ADC);
        lcd_send_string((char *)buffer);

        float daya = current_A*voltage_V;
        //baris 3
        lcd_send_cmd(0x80 | 0x14); //	lcd_setCursor(0,2);
        len = snprintf((char *)buffer, 50, "Daya:%.2fW", daya);
        lcd_send_string((char *)buffer);

        lcd_send_cmd(0x80 | 0x54); //	lcd_setCursor(0,3);
		switch(accu_class){
			case 0:
				len = snprintf((char *)buffer, 50, "bt:4v");
				break;
				
			case 1:
				len = snprintf((char *)buffer, 50, "bt:7v");
				break;
			case 2:
				len = snprintf((char *)buffer, 50, "bt:12v");
				break;
			default:
				len = snprintf((char *)buffer, 50, "bt:???");
		}
        lcd_send_string((char *)buffer);
		
		switch(charge_mode){
			case 0:
				len = snprintf((char *)buffer, 50, " CC");
				break;
				
			case 1:
				len = snprintf((char *)buffer, 50, " CV");
				break;
			
			default:
				len = snprintf((char *)buffer, 50, " OFF");
		}
        lcd_send_string((char *)buffer);
		
		len = snprintf((char *)buffer, 50, " PID:%.2f", pwmval);
        lcd_send_string((char *)buffer);

        prev_lcd_time = HAL_GetTick();
    }
}

void drive_sepic(float percent) {
    float maxPWM=2100;
    TIM4->CCR1= maxPWM-(maxPWM * percent);
    //TIM4->CCR1= (maxPWM * percent);
}
void data_logger() {
    int len = snprintf((char*)buffer, 100,"");
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, 100);

}
float adcToVolt(float adc) {
    return (adc/4095.0)*3.0;
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
    while(1)
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
