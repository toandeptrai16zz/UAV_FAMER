#ifndef TEST_CONFIG_HAL_H
#define TEST_CONFIG_HAL_H
#include <stdint.h>
#include <stddef.h>
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_UART_STATE_READY 0
#define FLASH_OPTR_DBANK 1u
#define SYSCFG_MEMRMP_FB_MODE 1u
#define FLASH_BASE 0x08000000u
#define FLASH_BANK_SIZE 0x40000u
#define FLASH_PAGE_SIZE 0x800u
#define FLASH_BANK_2 2u
#define FLASH_TYPEERASE_PAGES 1u
#define FLASH_TYPEPROGRAM_DOUBLEWORD 1u
#define FLASH_FLAG_ALL_ERRORS 0u
typedef int HAL_StatusTypeDef;
typedef struct { int gState; int RxState; } UART_HandleTypeDef;
typedef struct { uint32_t TypeErase,Banks,Page,NbPages; } FLASH_EraseInitTypeDef;
typedef struct { uint32_t OPTR; } FLASH_TypeDef;
typedef struct { uint32_t MEMRMP; } SYSCFG_TypeDef;
extern FLASH_TypeDef flash_instance;
extern SYSCFG_TypeDef syscfg_instance;
extern uint16_t mock_flash_size;
#define FLASH (&flash_instance)
#define SYSCFG (&syscfg_instance)
#define FLASHSIZE_BASE (&mock_flash_size)
#define __HAL_FLASH_CLEAR_FLAG(x) ((void)0)
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *, uint32_t *);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t, uint32_t, uint64_t);
#endif
