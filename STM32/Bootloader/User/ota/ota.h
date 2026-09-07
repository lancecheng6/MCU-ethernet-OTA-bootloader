/**
  * @file    ota.h
  * @brief   OTA 收檔核心（LwIP Raw API TCP Server :8000）
  *          NO_SYS / super-loop 架構：callback 收資料，狀態機在主迴圈處理。
  */
#ifndef __OTA_H
#define __OTA_H

#include "lwip/opt.h"
#include "lwip/tcp.h"

typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_RECEIVING,   /* TCP 收檔中 */
    OTA_STATE_VERIFY,      /* 收完，待 CRC 比對（主迴圈處理） */
    OTA_STATE_CONFIRM,     /* 已暫存 QSPI，等待使用者確認是否寫入 App */
    OTA_STATE_DONE,
    OTA_STATE_ERROR
} ota_state_t;

/* 512KB SRAM buffer（AXI SRAM 0x24000000，scatter .ota_buf section）*/
#define OTA_BUF_SIZE  (512 * 1024)
extern uint8_t ota_buf[OTA_BUF_SIZE];

void OTA_Server_Start(void);
void OTA_SendTrigger(void);   /* 按 y 後發 UDP 觸發封包到 PC，通知 ota_send.py 連回來 */
void OTA_Server_Stop(void);
void OTA_Poll(void);               /* main loop 呼叫：處理 state 轉移 */
ota_state_t OTA_GetState(void);
uint32_t OTA_GetReceived(void);
uint32_t OTA_GetExpected(void);
int      OTA_IsReceiving(void);
int      OTA_IsConfirm(void);
int      OTA_Flash_Write(void);
void     OTA_Abort(void);
void     OTA_ResetState(void);
void     OTA_SetExpectedCRC(uint32_t crc);

/* 供 TCP callback 用 */
struct tcp_pcb *OTA_GetClientPcb(void);

#endif /* __OTA_H */