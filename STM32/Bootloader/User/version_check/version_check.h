/**
  * @file    version_check.h
  * @brief   TCP Client version query to server (PC :8001, LwIP Raw API)
  */
#ifndef __VERSION_CHECK_H
#define __VERSION_CHECK_H

#include <stdint.h>

/* Start non-blocking version query; result available via Version_Check_Result() */
void     Version_Check_Start(void);
int      Version_Check_IsDone(void);
int      Version_Check_IsOK(void);
int      Version_Check_IsConnected(void);
void     Version_Check_Get(char *version, uint32_t *length, uint32_t *crc);

#endif /* __VERSION_CHECK_H */