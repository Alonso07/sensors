 #include "funciones_mcu.h"
#include "built_in.h"
 
//Timer2 Prescaler :69; Preload = 59999; Actual Interrupt Time = 35 ms

//Place/Copy this part in declaration section
void InitTimer2(){
  RCC_APB1ENR.TIM2EN = 1;
  TIM2_CR1.CEN = 0;
  TIM2_PSC = 69;
  TIM2_ARR = 59999;
  NVIC_IntEnable(IVT_INT_TIM2);
  TIM2_DIER.UIE = 1;
  TIM2_CR1.CEN = 1;
}

void Timer2_interrupt() iv IVT_INT_TIM2 {
  TIM2_SR.UIF = 0;
  //Enter your code here
}

void initial_mcu(){
// configuracion de modulos de comunicacion serial
      I2C2_Init_Advanced(400000, &_GPIO_MODULE_I2C2_PB10_11); // configurar I2C
      Delay_ms(100);
      UART2_Init_Advanced(19200, _UART_8_BIT_DATA, _UART_NOPARITY, _UART_ONE_STOPBIT, &_GPIO_MODULE_USART2_PA23); // configurar UART
      Delay_ms(100);
      InitTimer2();
      EnableInterrupts();
      }
