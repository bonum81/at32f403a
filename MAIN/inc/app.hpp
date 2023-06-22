#ifndef __APP_HPP
#define __APP_HPP

#include "at32f403a_407.h"

/* defines -------------------------------------------------------------------*/
#define USART1_RX_BUFFER_SIZE            32
#define MAX_RX_LENGHT                    8

#define LED_PIN       GPIO_PINS_15
#define LED_PORT      GPIOB
#define UART_TX_PIN   GPIO_PINS_9
#define UART_RX_PIN   GPIO_PINS_10
#define UART_PORT     GPIOA


/* variables ---------------------------------------------------------------- */
uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];


/* functions -----------------------------------------------------------------*/
void init_led(void);
void init_uart1(void);
void init_dma(void);
void uart_transmit_buffer(char* buf, int len);
void cmd_hadler(char *argv);


__INLINE void delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;        // разрешаем использовать DWT
}

__INLINE void delay_us(uint32_t us)
{
	uint32_t us_count_tic =  us * (system_core_clock / 1000000U);
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;    // запускаем счётчик
	while(DWT->CYCCNT < us_count_tic);
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;   // останавливаем счётчик
}

__INLINE void delay_ms(uint16_t ms)
{
    delay_us(ms*1000);
}


void system_clock_config(void)
{
  crm_reset();

  /* enable hext */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

   /* wait till hext is ready */
  while(crm_hext_stable_wait() == ERROR)
  {
  }

  /* config pll clock resource */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_16, CRM_PLL_OUTPUT_RANGE_GT72MHZ);

  /* enable pll */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

  /* wait till pll is ready */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* config apb2clk */
  crm_apb2_div_set(CRM_APB2_DIV_2);

  /* config apb1clk */
  crm_apb1_div_set(CRM_APB1_DIV_2);

  /* enable auto step mode */
  crm_auto_step_mode_enable(TRUE);

  /* select pll as system clock source */
  crm_sysclk_switch(CRM_SCLK_PLL);

  /* wait till pll is used as system clock source */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }

  /* disable auto step mode */
  crm_auto_step_mode_enable(FALSE);

  /* update system_core_clock global variable */
  system_core_clock_update();
}



#endif