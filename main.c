
#include "main.h"
#include "cmsis_os.h"

//____________________________________________________________________________________________________

#define LED7_DATA_HIGH		(GPIOB->BSRR = GPIO_PIN_5)
#define LED7_DATA_LOW		(GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16u);
#define LED7_CLOCK_HIGH		(GPIOB->BSRR = GPIO_PIN_3)
#define LED7_CLOCK_LOW		(GPIOB->BSRR = (uint32_t)GPIO_PIN_3 << 16u);
#define LED7_LATCH_HIGH		(GPIOD->BSRR = GPIO_PIN_2)
#define LED7_LATCH_LOW		(GPIOD->BSRR = (uint32_t)GPIO_PIN_2 << 16u);

#define LED1_POS 			0x10
#define LED2_POS 			0x20
#define LED3_POS 			0x40
#define LED4_POS 			0x80

uint32_t i = 0;
uint32_t num = 0;
unsigned char LED7SEGCA[11] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};

//____________________________________________________________________________________________________________

void HC595_Write(uint8_t data)
{
	for(int i=0; i<8; i++)
	{
		if(((data<<i)&0x80)==0x80)LED7_DATA_HIGH;
		else LED7_DATA_LOW;
		LED7_CLOCK_HIGH;
		LED7_CLOCK_LOW;
	}
}
void out_data()
{
	LED7_LATCH_HIGH;
	LED7_LATCH_LOW;
}

//_________________________________________________________________________________________________________________

osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

//__________________________________________________________________________________________________________________

osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

//___________________________________________________________________________________________________________________

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);

//____________________________________________________________________________________________________________________

int main(void)
{

  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();

  osKernelInitialize();


  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);


  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);


  osKernelStart();

  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; AFIO->MAPR = AFIO_MAPR_SWJ_CFG_1;

  while (1)
  {

  }

}

//___________________________________________________________________________________________________________

void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

//_________________________________________________________________________________________________________

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_4, GPIO_PIN_RESET);

    /*Configure GPIO pin : PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PB3 PB5 */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIOC->CRL |= 3UL<<0U;/* MODE */
    GPIOC->CRL &= ~(3UL<<2U); /*CNF*/
    GPIOC->CRL |= 3UL<<4U;/* MODE */
    GPIOC->CRL &= ~(3UL<<6U); /*CNF*/


}


void StartTask01(void *argument)
{

	for(;;)
	{
		HC595_Write(LED7SEGCA[num/1000]);
		HC595_Write(LED1_POS);
		out_data();

		HC595_Write(LED7SEGCA[(num%1000)/100]);
		HC595_Write(LED2_POS);
		out_data();

		HC595_Write(LED7SEGCA[((num%100)%100)/10]);
		HC595_Write(LED3_POS);
		out_data();

		HC595_Write(LED7SEGCA[((num%100)%100)%10]);
		HC595_Write(LED4_POS);
		out_data();
		i++;if(i>500)
		{
			i=0;num++;
			if(num>9999)num=0;
		}
    osDelay(1);
  }

}


void StartTask02(void *argument)
{

  for(;;)
  {
	  GPIOC->BSRR |= 1UL<<0U;
	  osDelay(200);
	  GPIOC->BRR |= 1UL<<0U;
	  osDelay(200);
  }

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }

}


void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif


