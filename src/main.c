#include "stm32l0xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

USART_HandleTypeDef huart2;
UART_HandleTypeDef lpuart;
TIM_HandleTypeDef htim2;

/* System Init */
#define SYS_PIN                                GPIO_PIN_12
#define SYS_GPIO_PORT                          GPIOA
#define SYS_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()

/* GPS LDO */
#define GPS_PIN                                GPIO_PIN_14
#define GPS_GPIO_PORT                          GPIOB
#define GPS_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()

/* Blinker LEDs */
#define LED_PIN                                GPIO_PIN_14
#define LED_GPIO_PORT                          GPIOC
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()

/* GPS frame */
#define OFS           (0x02)
#define pos_GPSlat     0x05  // 4 byte
#define pos_GPSlon     0x09  // 4 byte
#define pos_GPSalt     0x0D  // 3 byte
#define pos_GPSvE      0x10  // 2 byte
#define pos_GPSvN      0x12  // 2 byte
#define pos_GPSvU      0x14  // 2 byte
#define pos_GPStime    0x16  // 3 byte
#define pos_GPSdate    0x19  // 3 byte
#define pos_GPSsats    0x03  // 1 byte
#define pos_GPSfix     0x04  // 1 byte

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define WRITE_PROTOTYPE int _write(int file, char *ptr, int len)

typedef struct {
    int32_t lat; int32_t lon; uint16_t alt;
    uint16_t vE; uint16_t vN; uint16_t vU;
    uint32_t date; uint32_t time;
    uint8_t sats; uint8_t fix;
} datum_t;

datum_t datum;

/* Single byte to store input */
volatile uint8_t UartBuffer[128];
volatile uint8_t UartByte = 0;
volatile uint8_t UartIndex = 0;
volatile uint32_t cnt = 0;
static void Error_Handler(void);

int get_GPSpos() {
    datum.lat = (UartBuffer[pos_GPSlat] << 24) + (UartBuffer[pos_GPSlat+1] << 16) + (UartBuffer[pos_GPSlat+2] << 8) + UartBuffer[pos_GPSlat+3];
    datum.lon = (UartBuffer[pos_GPSlon] << 24) + (UartBuffer[pos_GPSlon+1] << 16) + (UartBuffer[pos_GPSlon+2] << 8) + UartBuffer[pos_GPSlon+3];
    datum.alt = (UartBuffer[pos_GPSalt] << 16) + (UartBuffer[pos_GPSalt+1] << 8) + UartBuffer[pos_GPSalt+2];
    return 0;
}

int get_GPSinfo() {
    datum.fix = UartBuffer[pos_GPSfix];
    datum.sats = UartBuffer[pos_GPSsats];
    return 0;
}


/* UART2 Interrupt Service Routine */
void LPUART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&lpuart);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    HAL_Delay(50);
  }
}

/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == LPUART1)
  {
    UartBuffer[UartIndex++] = UartByte;
    cnt = 0;
  }
  HAL_UART_Receive_IT(&lpuart, (uint8_t *)&UartByte, 1);
  
}

void SYS_Init(void) {
  GPIO_InitTypeDef SYS_GPIO_InitStruct;
  SYS_GPIO_CLK_ENABLE();
  SYS_GPIO_InitStruct.Pin = SYS_PIN;
  SYS_GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  SYS_GPIO_InitStruct.Pull = GPIO_PULLUP;
  SYS_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SYS_GPIO_PORT, &SYS_GPIO_InitStruct);
  HAL_GPIO_WritePin(SYS_GPIO_PORT, SYS_PIN, GPIO_PIN_SET); 
}


void GPS_Init(void) {
  GPIO_InitTypeDef GPS_GPIO_InitStruct;
  GPS_GPIO_CLK_ENABLE();
  GPS_GPIO_InitStruct.Pin = GPS_PIN;
  GPS_GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPS_GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPS_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPS_GPIO_PORT, &GPS_GPIO_InitStruct); 
  HAL_GPIO_WritePin(GPS_GPIO_PORT, GPS_PIN, GPIO_PIN_SET);
}


void LED_Init(void) {
  GPIO_InitTypeDef LED_GPIO_InitStruct;
  LED_GPIO_CLK_ENABLE();
  LED_GPIO_InitStruct.Pin = LED_PIN;
  LED_GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  LED_GPIO_InitStruct.Pull = GPIO_PULLUP;
  LED_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &LED_GPIO_InitStruct); 
}

void usart_gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOA_CLK_ENABLE();

  /**USART2 GPIO Configuration
  PA10     ------> USART2_TX
  PA9     ------> USART2_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void usart_init()
{
  __USART1_CLK_ENABLE();

  huart2.Instance = USART1;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = USART_WORDLENGTH_8B;
  huart2.Init.StopBits = USART_STOPBITS_1;
  huart2.Init.Parity = USART_PARITY_NONE;
  huart2.Init.Mode = USART_MODE_TX_RX;
  
  
  if(HAL_USART_Init(&huart2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

void lpuart_gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOC_CLK_ENABLE();

  /**LPUART1 GPIO Configuration
  PC11     ------> USART2_TX
  PC10    ------> USART2_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void lpuart_init()
{
  __LPUART1_CLK_ENABLE();
  lpuart.Instance = LPUART1;
  lpuart.Init.BaudRate = 38400;
  lpuart.Init.WordLength = UART_WORDLENGTH_8B;
  lpuart.Init.StopBits = UART_STOPBITS_1;
  lpuart.Init.Parity = UART_PARITY_NONE;
  lpuart.Init.Mode = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&lpuart) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(LPUART1_IRQn);
}
int isFrameReady() {
  return UartBuffer[0] == 0xAA && UartBuffer[1] == 0xAA && UartBuffer[2] == 0xAA && UartBuffer[3] == 0x03;
}
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  SYS_Init();
  GPS_Init();
  LED_Init();
 
  usart_gpio_init();
  usart_init();
  lpuart_gpio_init();
  lpuart_init();

  HAL_UART_Receive_IT(&lpuart, (uint8_t*)&UartByte, 1);

  printf("System operational. Waiting for GPS fix.\r\n");

  while(1)
  {
    if (cnt < 75000) {
      cnt++;
      continue;
      
    }
    if (isFrameReady() == 1) {
      get_GPSinfo();
      if (datum.fix == 1) {
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
      }
      if (datum.fix > 1) {
        get_GPSpos();
        printf("GPS Lat: %ld.%06d, ", datum.lat/1000000, abs(datum.lat)%1000000);
        printf("Lon: %ld.%06d, ", datum.lon/1000000, abs(datum.lon)%1000000);
        printf("Alt: %d.%02d, ", datum.alt/100, abs(datum.alt)%100);
        printf("Sats: %d\r\n", datum.sats);
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
      }
    }
      // printf ("Frame DUMP: \r\n");
      // for (uint8_t i=0; i <63; i++ )
      // {
      //   printf("%02x ", UartBuffer[i]);
      // }
      // printf("\r\n");
      
      // clear buffer
    memset((uint8_t*)UartBuffer, 0, 128);
    UartIndex = 0;
    cnt = 0;
  }
  cnt++;
}


void SysTick_Handler(void)
{
  HAL_IncTick();
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_USART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF); 
  return ch;
}

WRITE_PROTOTYPE
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}