#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t CCMR1,CCMR2,DCR,DMAR,EGR,CCR[4]; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct DMA_HandleTypeDef { void (*XferCpltCallback)(struct DMA_HandleTypeDef*); void (*XferErrorCallback)(struct DMA_HandleTypeDef*); } DMA_HandleTypeDef;
typedef struct { int gState,RxState; } UART_HandleTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
#define HAL_UART_STATE_READY 0
#define HAL_DMA_STATE_READY 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define TIM_DMA_UPDATE 1
#define TIM_FLAG_UPDATE 1
#define TIM_EGR_UG 1
#define TIM_CCMR1_OC1PE 1
#define TIM_CCMR1_OC2PE 2
#define TIM_CCMR2_OC3PE 1
#define TIM_CCMR2_OC4PE 2
#define TIM_DMABASE_CCR1 1
#define TIM_DMABURSTLENGTH_4TRANSFERS 2
#define UART_CLEAR_PEF 1
#define UART_CLEAR_FEF 2
#define UART_CLEAR_NEF 4
#define UART_CLEAR_OREF 8
#define __HAL_TIM_DISABLE_DMA(h,x) ((void)0)
#define __HAL_TIM_ENABLE_DMA(h,x) ((void)0)
#define __HAL_TIM_SET_COMPARE(h,ch,v) ((h)->Instance->CCR[ch]=(v))
#define __HAL_TIM_SET_PRESCALER(h,x) ((void)0)
#define __HAL_TIM_SET_AUTORELOAD(h,x) ((void)0)
#define __HAL_TIM_SET_COUNTER(h,x) ((void)0)
#define __HAL_TIM_CLEAR_FLAG(h,x) ((void)0)
#define __HAL_UART_CLEAR_FLAG(h,x) ((void)0)
#define SET_BIT(reg,bits) ((reg)|=(bits))
#define __DMB() ((void)0)
#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void HAL_GPIO_WritePin(int,int,int);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
int HAL_DMA_GetState(DMA_HandleTypeDef*);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef*);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef*,uint32_t,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t);
#endif
