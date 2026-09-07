/**
  * @file    menu.h
  * @brief   OTA Bootloader interactive menu (UART 0-6 keys)
  */

#ifndef __MENU_H
#define __MENU_H

#include "main.h"

void Display_Menu(void);
void Show_Info(void);
void Show_Help(void);
int  Firmware_Update(void);
void Backup_SlotA(void);
void Restore_SlotA(void);
void JumpToApp(void);
int  Boot_Metadata_Step(void);   /* 0=選單 1=開狗跳 2=直接跳 */
void Iwdg_Enable(uint32_t timeout_ms);
int  Menu_HandleKey(int key);

#endif /* __MENU_H */