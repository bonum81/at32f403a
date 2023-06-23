/**
  **************************************************************************
  * @file     app.cpp
  * @brief    example for at32f403AVGT
  **************************************************************************
  */


/* includes ------------------------------------------------------------------*/
#include "app.hpp"
#include <string>
#include <cstring>
#include <algorithm>
#include <ctype.h>
#include <stdio.h>

uint8_t dma_complete_tx = 0;


int main(void)
{
  system_clock_config();      // Тактируемся
  delay_init();               // Инициализация DWT для функций задержек
  init_uart1();               // Инициализация UART1
  init_led();                 // Инициализация LED порта
  init_dma();                 // Инициализация DMA Channel 5 для приема UART1

  uart_transmit_buffer_dma("I'm alive again!\n"); // Поздароваемся

  while (1)
  {

  }    
}

void init_led(void)
{
  gpio_init_type gpio_led;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_led);

  gpio_led.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_led.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_led.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_led.gpio_pins = LED_PIN;
  gpio_led.gpio_pull = GPIO_PULL_DOWN;

  gpio_init(LED_PORT, &gpio_led);
}

void init_uart1(void)
{
  gpio_init_type gpio_init_struct;
  
  crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);  
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  /* Конфигурация пина UART1 TX */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = UART_TX_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(UART_PORT, &gpio_init_struct);
  
   /* Конфигурация пина UART1 RX */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = UART_RX_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(UART_PORT, &gpio_init_struct);
  
  /* Конфигурируем параметры UART1
  Скорость: 115200 бод
  Бит: 8
  Стоп бит: 1
  без контроля четности
  */
  usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);

  /* Разрешим передатчик */
  usart_transmitter_enable(USART1, TRUE);
  /* Разрешим приемник */
  usart_receiver_enable(USART1, TRUE);
  /* Разрешим передачу для DMA */
  usart_dma_receiver_enable(USART1, TRUE);
  usart_dma_transmitter_enable(USART1, TRUE);  
  /* Разрешим флаг прерывания по IDLE */
  usart_interrupt_enable(USART1, USART_IDLE_INT, TRUE);
  /* Включим USART1*/
  usart_enable(USART1, TRUE);
  /* Разрешим прерывание для USART1 */
  nvic_irq_enable(USART1_IRQn, 0, 0);
}


void init_dma(void)
{
  dma_init_type dma_init_struct;  
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);    

  /* Сконфигурируем DMA1 Channel4 для передачи по USART1
    Длинну и передаваемый буфер не указываем, т.к незнаем
    заранее сколько символов нужно передать.
    Длинна и адрес буфера указываются в передающей фукнции
    void uart_transmit_buffer_dma(char* buf)
   */
  dma_reset(DMA1_CHANNEL4);
  dma_default_para_init(&dma_init_struct);  
  dma_init_struct.buffer_size = 0;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL4, &dma_init_struct);
  /* config flexible dma for usart1 tx */
  dma_flexible_config(DMA1, FLEX_CHANNEL4, DMA_FLEXIBLE_UART1_TX);



  dma_reset(DMA1_CHANNEL5);
  dma_default_para_init(&dma_init_struct);  
  /* Укажем размер приемного буфера */
  dma_init_struct.buffer_size = USART1_RX_BUFFER_SIZE;
  /* Укажем режим работы периферия в память */          
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  /* Укажем адрес приемного буфера */
  dma_init_struct.memory_base_addr = (uint32_t)usart1_rx_buffer;
  /* Укажем разрядность периферейного регистра */
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  /* Укажем адрес приемника USART1*/
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  /* Циклическая перезапись данных отключена*/
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL5, &dma_init_struct);
  
  /* Включим флаг прерывания по заполению буфера  */
  dma_interrupt_enable(DMA1_CHANNEL5, DMA_FDT_INT, TRUE);
  
  /* Разрешим прерывание от DMA1 Channel 5 */
  nvic_irq_enable(DMA1_Channel5_IRQn, 0, 0);
  
  /* DMA Channel 5 на прием от UART1 */
  dma_flexible_config(DMA1, FLEX_CHANNEL5, DMA_FLEXIBLE_UART1_RX);
  
  /* Включим DMA1 Channel5*/
  dma_channel_enable(DMA1_CHANNEL5, TRUE); 
}

uint8_t uart_transmit_buffer_dma(char* buf)
{
    uint32_t waiting = 0; 
  	strcat(buf,"\r");								                  // добавляем символ конца строки
    dma_channel_enable(DMA1_CHANNEL4, FALSE); 				// выключаем DMA
    DMA1_CHANNEL4->maddr = (uint32_t)buf;             // адрес на строки, которую нужно передать		
    DMA1_CHANNEL4->dtcnt = strlen(buf);				        // длина строки
    dma_flag_clear(DMA1_FDT4_FLAG);					          // сброс флага окончания передачи
    dma_channel_enable(DMA1_CHANNEL4, TRUE);    			// включить DMA
    while (!dma_flag_get(DMA1_FDT4_FLAG))             // Ждем флаг окончания передачи
    {
      waiting++;
      delay_ms(1);
      if (waiting > 10)
      {
        return 0;                                    // Не получили флаг окончания более чем 10мс, вернем 0
      }
    };

    return 1;
}

void uart_transmit_buffer(char* buf, int len)
{
  int counter = 0;
  while(len > 0)
  {
    while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART1, buf[counter++]);
    len--;
  }
}


void cmd_handler(char *argv)
{
  std::string command(argv);                                                  // приведем к строке
  std::transform(command.begin(), command.end(), command.begin(), ::tolower); // приведем к нижнему регистру
  if (command == "reset")
  {
    uart_transmit_buffer_dma("MCU reset now..\n");
    delay_ms(2000);
    NVIC_SystemReset();
    return;
  }
  if (command == "status")
  {
    if (gpio_output_data_bit_read(LED_PORT, LED_PIN))
    {
      uart_transmit_buffer_dma("led=on\n");
    }
    else
    {
      uart_transmit_buffer_dma("led=off\n");
    }
    return;
  }
  if (command == "led=on")
  {
    gpio_bits_set(LED_PORT, LED_PIN);
    uart_transmit_buffer_dma("Command led=on done..\n");
    return;
  }
  if (command == "led=off")
  {
    gpio_bits_reset(LED_PORT, LED_PIN);
    uart_transmit_buffer_dma("Command led=off done..\n");
    return;
  }

  uart_transmit_buffer_dma("Unknown command..\n");
}



#ifdef __cplusplus
extern "C" {
#endif


/* Прерывание от DMA по заполнению приемного буфера*/
void DMA1_Channel5_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT5_FLAG))   // проверим что прерывание сработало по заполнению буфера
  {
    dma_flag_clear(DMA1_FDT5_FLAG); //  сбросим флаг 
  }
}

/* Прерывание от USART1 по флагу IDLE (флаг взводится по превышению временного межбейтового временного окна) */
void USART1_IRQHandler(void)
{
  
  if(usart_flag_get(USART1, USART_IDLEF_FLAG))  
  {
    uint32_t length =  USART1_RX_BUFFER_SIZE - dma_data_number_get(DMA1_CHANNEL5);   // Определяем полученную длинну

    if (length < MAX_RX_LENGHT)                     // От переполения проверяем что длинна строки не вышла за максамально допустимую
    {
      usart1_rx_buffer[length] = 0;                 // Добавляем конец строки
      cmd_handler((char*)usart1_rx_buffer);         // Идем парсить и выполнять команды
    }
    else
    {
      uart_transmit_buffer_dma("Overflow rx data..\n");         // Переполнились, сообщим об этом
    }
    usart_data_receive(USART1);                                 // Пустое чтение нужно для очистки флага IDLE
    dma_channel_enable(DMA1_CHANNEL5, FALSE);                   // Отключим канал DMA для последующей операции
    dma_data_number_set(DMA1_CHANNEL5, USART1_RX_BUFFER_SIZE);  // Повторно запишем счетчик принимаемых данных в DMA
    dma_channel_enable(DMA1_CHANNEL5, TRUE);                    // Включим канал DMA обратно
  }
}


#ifdef __cplusplus
}                       //End of extern "C"
#endif
