/**
  * @file    ota.h
  * @brief   OTA firmware receive core (LwIP Raw API TCP Server :8000)
  *          NO_SYS / super-loop architecture: callback receives data, state machine runs in main loop.
  */
#ifndef __OTA_H
#define __OTA_H

#include "lwip/opt.h"
#include "lwip/tcp.h"

typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_RECEIVING,   /* TCP receiving firmware */
    OTA_STATE_VERIFY,      /* Receive complete, waiting for CRC check (handled in main loop) */
    OTA_STATE_CONFIRM,     /* Staged in QSPI, waiting for user confirmation to write to App */
    OTA_STATE_DONE,
    OTA_STATE_ERROR
} ota_state_t;

/* 512KB SRAM buffer (AXI SRAM 0x24000000, scatter .ota_buf section) */
#define OTA_BUF_SIZE  (512 * 1024)
extern uint8_t ota_buf[OTA_BUF_SIZE];

void OTA_Server_Start(void);
void OTA_Server_Stop(void);
void OTA_Poll(void);               /* Called from main loop: handle state transitions */
ota_state_t OTA_GetState(void);
uint32_t OTA_GetReceived(void);
uint32_t OTA_GetExpected(void);
int      OTA_IsReceiving(void);
int      OTA_IsConfirm(void);
int      OTA_Flash_Write(void);
void     OTA_Abort(void);
void     OTA_ResetState(void);
void     OTA_SetExpectedCRC(uint32_t crc);

/* For TCP callback use */
struct tcp_pcb *OTA_GetClientPcb(void);

#endif /* __OTA_H */