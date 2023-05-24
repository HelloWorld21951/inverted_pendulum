#include <math.h>

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

const double cart_ticks_to_m = 0.05 / 1024;
const double pend_ticks_to_rad = M_PI / 1024;

const double g = 9.81;
const double l = 0.617;
const double m_p = 0.355;
const double m_c = 0.216;
const double alpha = 10.0;

const double dt = 0.1;
const double max_velocity_error = 0.001;

double control_velocity = 0.0;

void SystemClock_Config(void);
//FIXME
int32_t convert_speed(double speed){
	return (int32_t)(speed / cart_ticks_to_m);
}

void move_carriage(double speed)
{
	if (fabs(speed) <= max_velocity_error)
	{
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1); // Stop moving
		LL_TIM_OC_SetCompareCH3(TIM2, 0);             // Stop moving
		LL_TIM_OC_SetCompareCH2(TIM2, 0);             // Stop moving
		return;
	}

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1); // Enable moving
	int32_t converted_speed = convert_speed(speed);

	if (converted_speed < 0)
	{
		converted_speed = fmax(converted_speed, -200);
		LL_TIM_OC_SetCompareCH3(TIM2, 0);
		LL_TIM_OC_SetCompareCH2(TIM2, (-((int32_t)converted_speed)));
		return;
	}

	if (converted_speed > 0)
	{
		converted_speed = fmin(converted_speed, 200);
		LL_TIM_OC_SetCompareCH2(TIM2, 0);
		LL_TIM_OC_SetCompareCH3(TIM2, (int32_t)(converted_speed));
		return;
	}
}
void start_buzzer(void)
{
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(100);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(100);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(100);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(100);
}
void buzzer_select_setpoint(void)
{
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(500);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(500);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(500);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(50);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(500);
}
void end_sensor_signal(void)
{
	LL_GPIO_SetOutputPin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin);
	LL_mDelay(300);
	LL_GPIO_ResetOutputPin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin);
	LL_mDelay(300);
	LL_GPIO_SetOutputPin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin);
	LL_mDelay(300);
	LL_GPIO_ResetOutputPin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin);
	LL_mDelay(300);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
	LL_mDelay(150);
	LL_GPIO_TogglePin(GPIOE, LL_GPIO_PIN_9);
}
void calibration(void)
{
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1); // Left moving
	LL_TIM_OC_SetCompareCH2(TIM2,0);            // Left moving
	LL_TIM_OC_SetCompareCH3(TIM2,40);           // Left moving
}

double get_cart_position(){
	return (TIM8->CNT - 32000) * cart_ticks_to_m;
}
double get_pendulum_position(){
	return (TIM4->CNT - 32000) * pend_ticks_to_rad;
}
// FIXME
double get_cart_speed(){
	TIM5->CNT = 0;
	uint16_t zeroPos = TIM8->CNT;
	while (zeroPos == TIM8->CNT);
	return TIM5->CNT * (TIM8->CNT - zeroPos) * cart_ticks_to_m;
}
// FIXME
double get_pendulum_speed(){
	TIM5->CNT = 0;
	uint16_t zeroPos = TIM4->CNT;
	while (zeroPos == TIM4->CNT);
	return TIM5->CNT * (TIM4->CNT - zeroPos) * pend_ticks_to_rad;
}
double get_force(double x, double theta, double x_dot, double theta_dot){
	double E = m_p * l * 2 * pow(theta_dot, 2) / 2 -
				m_p * g * l * (cos(theta) - 1);
	double F = m_p * g * cos(theta) * sin(theta) -
				m_p * l * pow(theta_dot, 2) * sin(theta) +
				(m_c + m_p * pow(sin(theta), 2) * 2 * g * sin(theta) / cos(theta)) +
				alpha * E * theta_dot * cos(theta);
	return F;
}
double get_control_velocity(double F){
	double x_ddot = F / (m_p + m_c);
	double v = control_velocity + x_ddot * dt;
	control_velocity = v;
	return v;
}

void send_data(int32_t data){
	while ((UART5->SR & USART_SR_RXNE) == 0) {}
	uint8_t flag = UART5->DR;
	while ((UART5->SR & USART_SR_TXE) == 0) {}
	UART5->DR = flag;
	while ((UART5->SR & USART_SR_TXE) == 0) {}
	UART5->DR = data & 0xFF;
	while ((UART5->SR & USART_SR_TXE) == 0) {}
	UART5->DR = (data>>8) & 0xFF;
	while ((UART5->SR & USART_SR_TXE) == 0) {}
	UART5->DR = (data>>16) & 0xFF;
	while ((UART5->SR & USART_SR_TXE) == 0) {}
	UART5->DR = (data>>24) & 0xFF;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* System interrupt init*/
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	MX_UART5_Init();
	MX_TIM5_Init();

	TIM4->CR1 |= (1<<0); // Start timer 4 to read encoder
	TIM8->CR1 |= (1<<0); // Start timer 8 to read encoder
	TIM5->CR1 |= (1<<0); // Start timer 5
	UART5->CR1 |= USART_CR1_TE | USART_CR1_RE ; // Enable transmission and receiving USART data
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
	LL_TIM_EnableCounter(TIM2);

	calibration();
	start_buzzer();

	double theta = 0.0;
	double x = 0.0;
	double theta_dot = 0.0;
	double x_dot = 0.0;
	double F = 0.0;
	double v = 0.0;

	while (1)
	{
		x = get_cart_position();
		theta = get_pendulum_position();
		x_dot = get_cart_speed();
		theta_dot = get_pendulum_speed();
		F = get_force(x, theta, x_dot, theta_dot);
		v = get_control_velocity(F);
		move_carriage(v);
		LL_mDelay((int32_t)(dt * 1000));
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
}

/* USER CODE BEGIN 4 */

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
