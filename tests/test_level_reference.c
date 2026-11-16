#define _GNU_SOURCE
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include "../Core/Src/uav_config.c"
FLASH_TypeDef flash_instance = {.OPTR = FLASH_OPTR_DBANK};
SYSCFG_TypeDef syscfg_instance = {0};
uint16_t mock_flash_size = 512u;
static uint32_t tick;
static int cut_after = -1;
static uint8_t last_tx[100];
static uint16_t last_tx_len;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n) { (void)h;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)h; assert(n<=sizeof(last_tx)); memcpy(last_tx,p,n);last_tx_len=n;return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *error) {
  (void)error;
  assert(e->Banks==FLASH_BANK_2 && e->NbPages==1);
  uint32_t dst=FLASH_BASE+FLASH_BANK_SIZE+e->Page*FLASH_PAGE_SIZE;
  assert(dst==FLASH_CFG_PAGE0 || dst==FLASH_CFG_PAGE1);
  memset((void *)(uintptr_t)dst,0xff,FLASH_PAGE_SIZE);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type,uint32_t dst,uint64_t value) {
  assert(type == FLASH_TYPEPROGRAM_DOUBLEWORD && (dst%8)==0);
  if(cut_after==0)return HAL_ERROR;
  if(cut_after>0)cut_after--;
  uint64_t cur;
  memcpy(&cur,(void *)(uintptr_t)dst,8);
  assert(cur==UINT64_MAX);
  memcpy((void *)(uintptr_t)dst,&value,8);
  return HAL_OK;
}
static void command(UART_HandleTypeDef *uart, uint8_t type, bool safe)
{
    uint8_t packet[9]={SYNC_A,SYNC_B,1u,type,7u,0u,0u,0u,0u};
    uint16_t crc=crc16(packet,7u);
    packet[7]=(uint8_t)crc;packet[8]=(uint8_t)(crc>>8);
    for(unsigned i=0;i<sizeof(packet);i++){
        uart_rx_byte=packet[i];
        UAVConfig_OnRxComplete(uart);
    }
    last_tx_len=0;
    UAVConfig_Service(safe);
    assert(last_tx_len>0);
    assert(last_tx[0]==SYNC_A && last_tx[1]==SYNC_B && last_tx[3]==(type|0x80u));
    assert(crc16(last_tx,last_tx_len-2u)==(uint16_t)(last_tx[last_tx_len-2u]|(last_tx[last_tx_len-1u]<<8)));
    UAVConfig_OnTxComplete(uart);
}
static void samples(float r,float p) {
 for(int i=0;i<105;i++) {tick+=20;UAVConfig_LevelObserve(r,p,0.02f,-0.01f,0.03f,1.0f,true);}
}
int main(void) {
 void *p=mmap((void *)(uintptr_t)FLASH_CFG_PAGE0,0x1000u,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0);
 if(p==MAP_FAILED) { perror("mmap config flash"); return 2; }
 assert(p==(void *)(uintptr_t)FLASH_CFG_PAGE0);
 memset(p,0xff,0x1000u);
 UART_HandleTypeDef uart={0};
 FlightPIDGains gains;
 FlightPID_DefaultGains(&gains);
 assert(FlightPID_SetGains(&gains));
 UAVConfig_Init(&uart);
 assert(!reference_valid && UAVConfig_LevelRoll()==0);
 command(&uart,UAV_CONFIG_CMD_LEVEL_GET,false);
 assert(last_tx_len==31 && last_tx[6]==22 && last_tx[7]==0 && last_tx[8]==0 && last_tx[9]==0);
 command(&uart,UAV_CONFIG_CMD_LEVEL_CAPTURE,false);
 assert(last_tx[7]==UAV_CONFIG_STATUS_LOCKED);
 assert(level_capture()==UAV_CONFIG_STATUS_BUSY);
 /* Must be still, fresh and explicitly safe; arbitrary location is not auto captured. */
 for(int i=0;i<105;i++){tick+=20;UAVConfig_LevelObserve(3.4f,-0.2f,0,0,0,1.0f,false);}
 assert(level_capture()==UAV_CONFIG_STATUS_BUSY);
 samples(3.4f,-0.2f);
 /* End-to-end: UART CAPTURE request invokes level_capture and returns value. */
 command(&uart,UAV_CONFIG_CMD_LEVEL_CAPTURE,true);
 assert(last_tx[7]==UAV_CONFIG_STATUS_OK && last_tx[6]==22 && last_tx[9]==1);
 assert(reference_valid && fabsf(reference_roll_deg-3.4f)<0.0005f);
 assert(fabsf(reference_pitch_deg+0.2f)<0.0005f);
 command(&uart,UAV_CONFIG_CMD_LEVEL_GET,true);
 assert(last_tx_len==31 && last_tx[9]==1);
 float test_roll;
 memcpy(&test_roll,&last_tx[10],4);
 assert(fabsf(test_roll-3.4f)<0.0005f);
 assert(persist_config());
 assert(record_valid((const ConfigRecord *)FLASH_CFG_PAGE0));
 assert(level_tail_valid((const LevelTail *)(FLASH_CFG_PAGE0+72u)));
 /* PID and tare survive reboot; legacy config records still recognized. */
 UAVConfig_Init(&uart);
 assert(reference_valid && fabsf(UAVConfig_LevelRoll()-3.4f)<0.0005f);
 reference_valid=false;reference_roll_deg=reference_pitch_deg=0;
 assert(persist_config());
 assert(record_valid((const ConfigRecord *)FLASH_CFG_PAGE1));
 UAVConfig_Init(&uart);assert(!reference_valid && UAVConfig_LevelRoll()==0);
 /* Capture 2nd pose and simulate power fail before final commit. */
 samples(1.2f,0.4f);assert(level_capture()==UAV_CONFIG_STATUS_OK);
 cut_after=8; assert(!persist_config());cut_after=-1;
 UAVConfig_Init(&uart);assert(!reference_valid && UAVConfig_LevelRoll()==0);
 /* Stable but outside ±5° => range rejected, no accidental unsafe tare. */
 samples(5.5f,-0.1f);assert(level_capture()==UAV_CONFIG_STATUS_RANGE);
 command(&uart,UAV_CONFIG_CMD_LEVEL_GET,true);
 assert(last_tx[6]==22 && last_tx[26]==100 && last_tx[27]==0);
 float measured=0.0f;memcpy(&measured,&last_tx[18],sizeof(float));
 assert(fabsf(measured-5.5f)<0.001f);
 tick+=20;UAVConfig_LevelObserve(0,0,10,0,0,1.0f,true);
 assert(level_capture()==UAV_CONFIG_STATUS_BUSY);
 puts("PASS R5.6: 2s stationary gate, explicit capture, +/-5° range, PID-compatible dual-page journal, power-loss rollback");
 return 0;
}
