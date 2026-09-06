/**
  * @file    menu.c
  * @brief   OTA Bootloader interactive menu (UART 0-6 keys)
  *          Non-blocking: LwIP polling continues in main loop.
  */

#include "menu.h"
#include "bsp_debug_usart.h"
#include "ota.h"
#include "version_check.h"
#include "metadata.h"
#include "bsp_qspi_flash.h"
#include "lwip/init.h"
#include "lwip/ip_addr.h"
#include "app_ethernet.h"
#include "ethernetif.h"
#include <stdio.h>

extern int eth_inited;

#define APP_RUN_ADDR  0x08100000UL   /* Bank2 = App execution area */

void Display_Menu(void)
{
    printf("\r\n");
    printf("===== OTA Bootloader v1.0 (Bare-metal) =====\r\n");
    printf("  [1] Info              [2] Firmware Update\r\n");
    printf("  [3] Backup            [4] Restore\r\n");
    printf("  [5] Run App           [6] Reboot\r\n");
    printf("  [9] QSPI Erase Test   [0] Help\r\n");
    printf("============================================\r\n");
    printf("Select (0-9): ");
}

void Show_Info(void)
{
    Metadata meta;
    uint32_t rsr = RCC->RSR;

    printf("\r\n--- System Info ---\r\n");
    printf("  MCU        : STM32H743XIH6 @ 480MHz\r\n");
    printf("  IP         : %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    printf("  Slot A     : 0x08100000\r\n");
    printf("  NOR staging: 0x90000000 (offset 0x000000)\r\n");
    printf("  NOR backup : 0x90200000 (offset 0x200000)\r\n");

    /* RCC reset reason */
    printf("  Reset reason: 0x%08lX\r\n", (unsigned long)rsr);
    if (rsr & (1U << 26)) printf("                IWDG1 reset\r\n");
    if (rsr & (1U << 22)) printf("                PIN reset\r\n");
    if (rsr & (1U << 21)) printf("                BOR reset\r\n");
    if (rsr & (1U << 20)) printf("                D2 reset\r\n");
    if (rsr & (1U << 19)) printf("                D1 reset\r\n");
    if (rsr & (1U << 17)) printf("                CPU reset\r\n");
    if (rsr & (1U << 24)) printf("                Software reset\r\n");

    /* IWDG status (watchdog active or not) */
    printf("  IWDG1      : LSI_RDY=%d SR=0x%04lX PR=0x%08lX RLR=0x%08lX\r\n",
           (int)((RCC->CSR >> 1) & 1U),
           (unsigned long)IWDG1->SR,
           (unsigned long)IWDG1->PR,
           (unsigned long)IWDG1->RLR);
    printf("  RCC->CSR   : 0x%08lX (LSION=%d LSIRDY=%d)\r\n",
           (unsigned long)RCC->CSR,
           (int)(RCC->CSR & RCC_CSR_LSION) ? 1 : 0,
           (int)(RCC->CSR & RCC_CSR_LSIRDY) ? 1 : 0);

    /* Metadata diagnostic info */
    Metadata_Read(&meta);
    printf("\r\n--- Metadata ---\r\n");
    printf("  magic       : 0x%08lX%s\r\n",
           (unsigned long)meta.magic,
           (meta.magic == METADATA_MAGIC) ? " (OK)" : " (INVALID)");
    printf("  state       : 0x%08lX (%s)\r\n",
           (unsigned long)meta.state, Metadata_StateStr(meta.state));
    printf("  version     : %lu.%lu.%lu\r\n",
           (unsigned long)meta.version_major,
           (unsigned long)meta.version_minor,
           (unsigned long)meta.version_patch);
    printf("  crc32       : 0x%08lX\r\n", (unsigned long)meta.crc32);
    printf("  fw_size     : %lu bytes\r\n", (unsigned long)meta.fw_size);

    /* Metadata RAW: read flash raw bytes directly (no fallback), print separately */
    {
        volatile uint32_t *raw = (volatile uint32_t *)METADATA_ADDR;
        printf("\r\n--- Metadata RAW (0x%08lX) ---\r\n", (unsigned long)(uint32_t)METADATA_ADDR);
        for (int i = 0; i < 16; i++) {   /* 64 bytes / 4 = 16 words */
            printf("  [%02d] 0x%08lX\r\n", i, (unsigned long)raw[i]);
        }
    }
}

void Show_Help(void)
{
    printf("\r\n--- Help ---\r\n");
    printf("  [1] Info         - Show system info\r\n");
    printf("  [2] Firmware     - OTA update from PC server (TODO)\r\n");
    printf("  [3] Backup       - Copy Slot A to NOR backup (TODO)\r\n");
    printf("  [4] Restore      - Copy NOR backup to Slot A (TODO)\r\n");
    printf("  [5] Run App      - Jump to Slot A / App (TODO)\r\n");
    printf("  [6] Reboot       - Software reset\r\n");
    printf("  [9] QSPI Erase   - Erase QSPI flash block-by-block test\r\n");
    printf("  ESC / other     - Cancel or ignore\r\n");
}

/* --- Backup / Restore ([3]/[4]) --- */
#define APP_SIZE_C      (1UL * 1024UL * 1024UL)   /* Bank2 App area = 1MB */
#define BACKUP_OFFSET   0x200000UL                /* External NOR backup area offset (relative to 0x90000000) */

/* Wait for QUADSPI peripheral engine idle (check QSPI_FLAG_BUSY) */
static void QSPI_WaitIdle(void)
{
    uint32_t guard = 1000000;
    while (__HAL_QSPI_GET_FLAG(&QSPIHandle, QSPI_FLAG_BUSY) && guard--) { }
}

/* [3] Backup: copy Bank2(0x08100000) -> external NOR backup(0x200000), full 1MB */
void Backup_SlotA(void)
{
    uint8_t buf[4096];
    uint32_t off;

    printf("\r\n[Backup] Copying Bank2(0x08100000,1MB) -> NOR backup(0x200000)...\r\n");

    /* 1. Verify source is valid (Bank2 header vector not 0xFF/0) */
    uint32_t msp = *(uint32_t *)APP_RUN_ADDR;
    if (msp == 0xFFFFFFFFU || msp == 0U) {
        printf("[Backup] No valid App (header=0x%08lX) -> cancelled\r\n", (unsigned long)msp);
        Display_Menu();
        return;
    }

    /* 2. Erase backup area 1MB in batches (16 x 64KB blocks, reset QSPI every 8) */
    if (QSPI_Erase_Blocks(BACKUP_OFFSET, APP_SIZE_C) != 0) {
        printf("[Backup] Backup area erase failed\r\n");
        Display_Menu();
        return;
    }
    printf("[Backup] Backup area erase complete\r\n");

    /* 3. Read Bank2 -> write NOR backup (4096 chunks) */
    for (off = 0; off < APP_SIZE_C; off += 4096) {
        QSPI_WaitIdle();
        memcpy(buf, (void *)(APP_RUN_ADDR + off), 4096);
        if (BSP_QSPI_Write(buf, BACKUP_OFFSET + off, 4096) != QSPI_OK) {
            printf("[Backup] Write backup FAIL @0x%08lX\r\n", (unsigned long)off);
            Display_Menu();
            return;
        }
        if ((off % (512UL * 1024UL)) == 0)
            printf("\r[Backup] %lu%%", (unsigned long)(100 * off / APP_SIZE_C));
    }
    printf("\r[Backup] 100%%\r\n");
    printf("[Backup] Done\r\n");
    Display_Menu();
}

/* [4] Restore: copy external NOR backup(0x200000) -> Bank2(0x08100000), full 1MB */
void Restore_SlotA(void)
{
    uint8_t hdr[4];
    uint8_t wbuf[32];
    uint32_t off, remain;

    printf("\r\n[Restore] Copying NOR backup(0x200000) -> Bank2(0x08100000,1MB)...\r\n");

    /* 1. Verify backup area is valid (header not 0xFF) */
    QSPI_WaitIdle();
    if (BSP_QSPI_Read(hdr, BACKUP_OFFSET, 4) != QSPI_OK) {
        printf("[Restore] Read backup failed\r\n");
        Display_Menu();
        return;
    }
    if (hdr[0] == 0xFF && hdr[1] == 0xFF && hdr[2] == 0xFF && hdr[3] == 0xFF) {
        printf("[Restore] Backup area empty -> cancelled\r\n");
        Display_Menu();
        return;
    }

    /* 2. Erase entire Bank2 (8 sectors, one by one; wait for SR2(BSY/QW) clear before next; retry on failure) */
    HAL_FLASH_Unlock();
    for (uint32_t s = 0; s < 8; s++) {
        FLASH_EraseInitTypeDef erase;
        uint32_t SectorError = 0;
        uint32_t guard;
        int ok = 0;
        erase.TypeErase     = FLASH_TYPEERASE_SECTORS;
        erase.Banks         = FLASH_BANK_2;
        erase.Sector        = s;
        erase.NbSectors     = 1;
        erase.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
        for (int attempt = 0; attempt < 4 && !ok; attempt++) {
            SectorError = 0;
            /* Wait for Bank2 idle */
            guard = 1000000;
            while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
            if (HAL_FLASHEx_Erase(&erase, &SectorError) != HAL_OK) {
                printf("[Restore] sector %lu erase retry %d SR2=0x%08lX\r\n",
                       (unsigned long)s, attempt, (unsigned long)FLASH->SR2);
                /* Wait a bit more (BSY/QW residual) */
                guard = 3000000;
                while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
            } else {
                ok = 1;
            }
        }
        if (!ok) {
            printf("[Restore] Bank2 erase FAILED (sector %lu) SR2=0x%08lX\r\n",
                   (unsigned long)SectorError, (unsigned long)FLASH->SR2);
            HAL_FLASH_Lock();
            Display_Menu();
            return;
        }
        /* Wait for this erase to complete (SR2 BSY/QW clear) */
        guard = 1000000;
        while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
        printf("[Restore] Erase Bank2 sector %lu/7 OK\r\n", (unsigned long)s);
    }

    /* 3. Read NOR backup -> FLASH_Program write back to Bank2 (32B FLASHWORD) */
    printf("[Restore] Writing to Bank2...\r\n");
    remain = APP_SIZE_C;
    off = 0;
    while (remain > 0) {
        uint32_t chunk = (remain >= 32) ? 32 : remain;
        memset(wbuf, 0xFF, sizeof(wbuf));
        QSPI_WaitIdle();
        if (BSP_QSPI_Read(wbuf, BACKUP_OFFSET + off, chunk) != QSPI_OK) {
            printf("[Restore] Read backup FAIL @0x%08lX\r\n", (unsigned long)off);
            HAL_FLASH_Lock();
            Display_Menu();
            return;
        }
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
        int ok = 0;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                  APP_RUN_ADDR + off, (uint32_t)wbuf) == HAL_OK)
                ok = 1;
            else {
                printf("[Restore] prog retry %d SR2=0x%08lX\r\n",
                       attempt, (unsigned long)FLASH->SR2);
                __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
            }
        }
        if (!ok) {
            printf("[Restore] Flash program FAIL @0x%08lX\r\n", (unsigned long)off);
            HAL_FLASH_Lock();
            Display_Menu();
            return;
        }
        off    += chunk;
        remain -= chunk;
        if ((off % (512UL * 1024UL)) == 0)
            printf("\r[Restore] Writing %lu%%", (unsigned long)(100 * off / APP_SIZE_C));
    }
    printf("\r[Restore] Writing 100%%\r\n");
    HAL_FLASH_Lock();

    /* Manual restore = confirm old version is usable -> Metadata mark CONFIRMED
       (Metadata_Write internally: copy to SRAM -> erase sector -> update state -> write back) */
    {
        Metadata meta;
        Metadata_Read(&meta);
        meta.state = STATE_CONFIRMED;
        if (Metadata_Write(&meta) != 0)
            printf("[Restore] Metadata write failed!\r\n");
        else
            printf("[Restore] Metadata → CONFIRMED\r\n");
    }

    printf("[Restore] Done\r\n");
    Display_Menu();
}
/**
  * @brief  Normal boot: read Metadata, run IWDG health check state machine.
  *         - PENDING_UPDATE -> mark TESTING -> enable IWDG(5s) -> jump to App
  *         - TESTING -> based on reset reason: IWDG->rollback; non-IWDG->CONFIRMED(version=new)
  *         - CONFIRMED -> jump directly to App
  *         - EMPTY/invalid -> interactive menu
  */

/* ---- IWDG (Independent Watchdog) direct register access ---- */
#define IWDG_KR_RELOAD   0xAAAAU   /* Reload key value */
#define IWDG_KR_UNLOCK   0x5555U   /* Unlock PR/RLR */
#define IWDG_KR_START    0xCCCCU   /* Start */
/* RCC_RSR IWDG1 reset flag (bit 26; official CMSIS: RCC_RSR_IWDG1RSTF=0x04000000) */
#define RCC_RSR_IWDG1RSTF_Msk   (0x1UL << 26)

/* Enable IWDG, timeout in ms (LSI 32kHz: reload=(ms*32)-1)
   Not static: used by main() boot flow when "KEY1 not pressed and state==TESTING" to enable watchdog. */
void Iwdg_Enable(uint32_t timeout_ms)
{
    /* 1. First enable LSI (watchdog clock source) and wait for it to be ready; otherwise IWDG won't count */
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }

    /* 2. Unlock + set PR/RLR + start IWDG
          PR: clock prescaler 64 (LSI 32k/64=500Hz -> decrement every 2ms) */
    IWDG1->KR = IWDG_KR_UNLOCK;
    IWDG1->PR  = 3;   /* 0b011: /64 */
    IWDG1->RLR = (timeout_ms / 2) - 1;   /* 500Hz -> every 2ms, reload count */
    IWDG1->KR = IWDG_KR_START;
}

/* Reload/feed watchdog */
static void Iwdg_Refresh(void)
{
    IWDG1->KR = IWDG_KR_RELOAD;
}

/* Check if reset was caused by IWDG (read RCC->RSR and clear) */
static int Reset_Reason_Is_IWDG(void)
{
    if (RCC->RSR & RCC_RSR_IWDG1RSTF_Msk) {
        RCC->RSR |= RCC_RSR_RMVF;   /* Clear all reset flags */
        return 1;
    }
    RCC->RSR |= RCC_RSR_RMVF;
    return 0;
}

/* Rollback: copy NOR backup(0x200000) -> Bank2(0x08100000). Return 0=success */
static int Boot_DoRollback(void)
{
    uint8_t hdr[4];
    /* FLASHWORD DataAddress requires 32B alignment; stack variables not guaranteed -> use static aligned buffer */
    static uint8_t wbuf[32] __attribute__((aligned(32)));
    uint32_t off, remain;
    extern int qspi_inited;

    printf("[ROLLBACK] Restoring Bank2 from NOR backup...\r\n");

    /* Ensure QSPI is initialized (normal boot path won't init, must do it here) */
    if (!qspi_inited) {
        printf("[ROLLBACK] QSPI init...\r\n");
        if (BSP_QSPI_Init() != QSPI_OK) {
            printf("[ROLLBACK] QSPI init failed, cannot rollback\r\n");
            return -1;
        }
        qspi_inited = 1;
    }

    QSPI_WaitIdle();
    if (BSP_QSPI_Read(hdr, BACKUP_OFFSET, 4) != QSPI_OK) { printf("[ROLLBACK] Read failed\r\n"); return -1; }
    if (hdr[0]==0xFF && hdr[1]==0xFF && hdr[2]==0xFF && hdr[3]==0xFF) {
        printf("[ROLLBACK] Backup area empty, cannot rollback\r\n"); return -1;
    }

    /* Erase Bank2 (8 sectors, one by one + retry) */
    HAL_FLASH_Unlock();
    for (uint32_t s = 0; s < 8; s++) {
        FLASH_EraseInitTypeDef erase;
        uint32_t SectorError = 0, guard;
        int ok = 0;
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.Banks = FLASH_BANK_2;
        erase.Sector = s;
        erase.NbSectors = 1;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        for (int attempt = 0; attempt < 4 && !ok; attempt++) {
            SectorError = 0;
            guard = 1000000;
            while ((FLASH->SR2 & (FLASH_SR_BSY|FLASH_SR_QW)) && guard--) { }
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1|FLASH_FLAG_ALL_ERRORS_BANK2);
            if (HAL_FLASHEx_Erase(&erase, &SectorError) != HAL_OK) {
                guard = 3000000;
                while ((FLASH->SR2 & (FLASH_SR_BSY|FLASH_SR_QW)) && guard--) { }
            } else ok = 1;
        }
        if (!ok) { printf("[ROLLBACK] Erase failed s%lu\r\n",(unsigned long)s); HAL_FLASH_Lock(); return -1; }
        guard = 1000000;
        while ((FLASH->SR2 & (FLASH_SR_BSY|FLASH_SR_QW)) && guard--) { }
    }

    /* Write back to Bank2 */
    remain = APP_SIZE_C; off = 0;
    while (remain > 0) {
        uint32_t chunk = (remain >= 32) ? 32 : remain;
        memset(wbuf, 0xFF, sizeof(wbuf));
        QSPI_WaitIdle();
        if (BSP_QSPI_Read(wbuf, BACKUP_OFFSET + off, chunk) != QSPI_OK) {
            printf("[ROLLBACK] Read backup failed\r\n"); HAL_FLASH_Lock(); return -1;
        }
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1|FLASH_FLAG_ALL_ERRORS_BANK2);
        int ok = 0;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, APP_RUN_ADDR+off, (uint32_t)wbuf)==HAL_OK) ok=1;
            else __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1|FLASH_FLAG_ALL_ERRORS_BANK2);
        }
        if (!ok) { printf("[ROLLBACK] Write back failed %08lX\r\n", (unsigned long)off); HAL_FLASH_Lock(); return -1; }
        off += chunk; remain -= chunk;
    }
    HAL_FLASH_Lock();
    printf("[ROLLBACK] Complete\r\n");
    return 0;
}

/* [9] QSPI erase test: two test points -- erase 8 Block64K at each.
   Test point 1: block starting at 0x000000 (includes sector 128)
   Test point 2: block starting at 0x2A0000 (includes sector 700)
   Uses 64KB block erase (BSP_QSPI_Erase_Block64K), prints OK / FAIL for each. */
static void QSPI_Erase_Test(void)
{
    uint32_t fail_cnt = 0;
    uint32_t base, addr;

    printf("\r\n=== QSPI Erase Test (Block64K x8 @0x000000 & 0x2A0000) ===\r\n");

    /* Ensure QSPI is initialized (interactive menu inits at start, but just in case) */
    extern int qspi_inited;
    if (!qspi_inited) {
        if (BSP_QSPI_Init() != QSPI_OK) { printf("[QSPI] init failed\r\n"); return; }
        qspi_inited = 1;
    }

    /* Test point 1: block 0x000000 x8 */
    base = 0x000000UL;
    printf("[QSPI] --- block64K %06lX x8 ---\r\n", (unsigned long)base);
    for (uint32_t i = 0; i < 8; i++) {
        addr = base + i * QSPI_BLOCK_64K_SIZE;
        if (BSP_QSPI_Erase_Block64K(addr) == QSPI_OK)
            printf("[QSPI] %06lX : OK\r\n", (unsigned long)addr);
        else { printf("[QSPI] %06lX : FAIL\r\n", (unsigned long)addr); fail_cnt++; }
    }

    /* Test point 2: block 0x2A0000 x8 */
    base = 0x2A0000UL;
    printf("[QSPI] --- block64K %06lX x8 ---\r\n", (unsigned long)base);
    for (uint32_t i = 0; i < 8; i++) {
        addr = base + i * QSPI_BLOCK_64K_SIZE;
        if (BSP_QSPI_Erase_Block64K(addr) == QSPI_OK)
            printf("[QSPI] %06lX : OK\r\n", (unsigned long)addr);
        else { printf("[QSPI] %06lX : FAIL\r\n", (unsigned long)addr); fail_cnt++; }
    }

    printf("=== Erase Test Done: %lu FAIL ===\r\n", (unsigned long)fail_cnt);
}

/**
  * @brief  Normal boot: read Metadata, determine boot behavior (Plan A).
  *         - CONFIRMED / PENDING_UPDATE -> jump directly to App (Bank2 already contains latest firmware)
  *           (During PENDING, firmware was directly written to Bank2 during OTA, no need to copy from NOR)
  *         - TESTING -> Step 2 implements IWDG/health check; for now just jump to App
  *         - EMPTY / invalid -> interactive_mode
  */
void normal_boot(void)
{
    Metadata meta;
    Metadata_Read(&meta);

    /* Diagnostic: print Metadata state + reset reason at boot */
    printf("\r\n[BOOT] meta.magic=0x%08lX state=0x%08lX(%s) ver=%lu.%lu.%lu\r\n",
           (unsigned long)meta.magic, (unsigned long)meta.state,
           Metadata_StateStr(meta.state),
           (unsigned long)meta.version_major,
           (unsigned long)meta.version_minor,
           (unsigned long)meta.version_patch);
    printf("[BOOT] RSR=0x%02lX IWDG1RSTF=%d\r\n",
           (unsigned long)(RCC->RSR & 0x0FF00000),
           (RCC->RSR & RCC_RSR_IWDG1RSTF_Msk) ? 1 : 0);

    if (meta.magic != METADATA_MAGIC || meta.state == STATE_EMPTY)
    {
        printf("\r\n[BOOT] No valid firmware -> entering interactive menu\r\n");
        Display_Menu();
        extern void interactive_mode(void);
        interactive_mode();
        return;
    }

    /* PENDING_UPDATE: new firmware written to Bank2, enter health check -> enable IWDG jump to App */
    if (meta.state == STATE_PENDING_UPDATE)
    {
        printf("[BOOT] PENDING_UPDATE -> mark TESTING, enable IWDG(5s) jump to App\r\n");
        meta.state = STATE_TESTING;
        if (Metadata_Write(&meta) != 0)
            printf("[BOOT] Metadata write failed (TESTING)\r\n");
        Iwdg_Enable(5000);   /* App must feed watchdog, otherwise 5s IWDG resets */
        JumpToApp();
        return;
    }

    /* TESTING: determine App success based on reset reason */
    if (meta.state == STATE_TESTING)
    {
        if (Reset_Reason_Is_IWDG())
        {
            /* App failed (reset by IWDG within 5s without feeding) -> rollback to old version */
            printf("[BOOT] TESTING + IWDG reset -> App failed -> rollback\r\n");
            if (Boot_DoRollback() == 0) {
                meta.state = STATE_CONFIRMED;   /* Restore success, mark CONFIRMED (version=old version unchanged) */
                if (Metadata_Write(&meta) != 0)
                    printf("[BOOT] Metadata write failed (after rollback)\r\n");
                printf("[BOOT] Rollback complete, jumping to old App\r\n");
            } else {
                printf("[BOOT] Rollback failed -> interactive menu\r\n");
                Display_Menu();
                extern void interactive_mode(void);
                interactive_mode();
                return;
            }
        }
        else
        {
            /* App normal (non-IWDG reset = last boot's App survived) -> mark CONFIRMED (version=new) */
            printf("[BOOT] TESTING + non-IWDG reset -> App normal -> CONFIRMED\r\n");
            meta.state = STATE_CONFIRMED;
            if (Metadata_Write(&meta) != 0)
                printf("[BOOT] Metadata write failed (CONFIRMED)\r\n");
        }
        JumpToApp();
        return;
    }

    /* CONFIRMED: jump directly */
    printf("[BOOT] State=%s -> jumping to App @0x08100000\r\n",
           Metadata_StateStr(meta.state));
    JumpToApp();
}

/* Metadata state transition + return boot requirement (executed before KEY1 check).
   Return values:
     0 -> invalid/EMPTY or rollback failed -> enter menu
     1 -> state==TESTING -> need to enable IWDG before jumping to App (health check start point)
     2 -> CONFIRMED (or rollback complete) -> jump directly to App
   State transitions:
     PENDING -> TESTING (write, no watchdog)
     TESTING + IWDG triggered -> rollback -> CONFIRMED (write)
     TESTING + non-IWDG -> still TESTING (don't directly CONFIRMED, let "enable watchdog + jump" happen)
     CONFIRMED -> no change */
int Boot_Metadata_Step(void)
{
    Metadata meta;
    Metadata_Read(&meta);

    if (meta.magic != METADATA_MAGIC || meta.state == STATE_EMPTY)
        return 0;   /* invalid/EMPTY -> enter menu */

    if (meta.state == STATE_PENDING_UPDATE)
    {
        meta.state = STATE_TESTING;
        if (Metadata_Write(&meta) != 0)
            printf("[BOOT] Metadata write failed (PENDING->TESTING)\r\n");
        printf("[BOOT] PENDING_UPDATE -> TESTING (pre-transition)\r\n");
        return 1;   /* Transitioned to TESTING -> need watchdog jump */
    }
    else if (meta.state == STATE_TESTING)
    {
        if (Reset_Reason_Is_IWDG())
        {
            /* App reset by watchdog -> rollback */
            printf("[BOOT] TESTING + IWDG -> rollback\r\n");
            if (Boot_DoRollback() == 0)
            {
                meta.state = STATE_CONFIRMED;
                if (Metadata_Write(&meta) != 0)
                    printf("[BOOT] Metadata write failed (rollback CONFIRMED)\r\n");
                printf("[BOOT] Rollback complete -> CONFIRMED\r\n");
                return 2;
            }
            else
            {
                printf("[BOOT] Rollback failed -> stay in TESTING (enter menu)\r\n");
                return 0;
            }
        }
        else
        {
            /* Non-IWDG reset: still TESTING, let "enable watchdog + jump to App" happen (health check start point) */
            printf("[BOOT] TESTING + non-IWDG -> stay TESTING, enable watchdog jump to App\r\n");
            return 1;
        }
    }

    /* CONFIRMED: jump directly */
    return 2;
}
void JumpToApp(void)
{
    uint32_t app_addr = APP_RUN_ADDR;

    printf("\r\n[BOOT] Jumping to App @0x%08lX\r\n", (unsigned long)app_addr);
    HAL_Delay(50);

    /* 1. Disable global interrupts + stop SysTick */
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* 2. Stop Ethernet DMA + QSPI (avoid background access; Ethernet only needs stop if previously init'd) */
    if (eth_inited) {
        extern ETH_HandleTypeDef EthHandle;
        HAL_ETH_Stop(&EthHandle);
        HAL_ETH_DeInit(&EthHandle);
        eth_inited = 0;
    }
    extern QSPI_HandleTypeDef QSPIHandle;
    HAL_QSPI_DeInit(&QSPIHandle);

    /* 3. Clear all NVIC interrupts */
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* 4. Clear Cache */
    SCB_CleanDCache();
    SCB_InvalidateDCache();
    SCB_InvalidateICache();

    /* 5. Set VTOR + MSP + jump */
    SCB->VTOR = app_addr;
    __set_MSP(*(uint32_t *)app_addr);
    void (*app_reset)(void) = (void (*)(void))(*(uint32_t *)(app_addr + 4));
    app_reset();
    while (1) { }   /* never reached */
}

/* Ethernet deferred initialization (only once, init when entering [2] Firmware Update) */
extern struct netif gnetif;
extern int flag;

static void Eth_Init_Once(void)
{
    if (eth_inited) return;
    printf("\r\n[ETH] Initializing Ethernet...\r\n");
    lwip_init();
    Netif_Config();
    User_notification(&gnetif);
    printf("LAN8720A Ethernet Demo\n");
    printf("LwIP version: %s\n", LWIP_VERSION_STRING);
    printf("Board IP: %d.%d.%d.%d\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    eth_inited = 1;
}

/**
  * @brief  Wait for a key press while continuously running LwIP (avoid TCP closed/receive callback hang).
  *         Returns key ASCII, or 0 (no key, just ran LwIP once).
  */
static int WaitKey_WithLwIP(void)
{
    if (flag) { flag = 0; ethernetif_input(&gnetif); }
    sys_check_timeouts();
    return UART_ReadKey_NonBlock();
}

/**
  * @brief  [2] Firmware Update.
  *         Initialize Ethernet first; then query version -> three-way branch.
  *         Each confirmation uses "non-blocking key read + LwIP simultaneously", no timeout set.
  */
int Firmware_Update(void)
{
    printf("\r\n--- Firmware Update ---\r\n");
    Eth_Init_Once();

    /* Query version, print "server connected" on success */
    char ver[16]; uint32_t len, crc;
    int c;

retry:
    printf("\r\n[OTA] Connecting to server...\r\n");
    Version_Check_Start();

    uint32_t t0 = HAL_GetTick();
    while (!Version_Check_IsDone())
    {
        if (flag) { flag = 0; ethernetif_input(&gnetif); }
        sys_check_timeouts();
        if ((HAL_GetTick() - t0) > 6000) break;   /* timeout */
    }

    if (Version_Check_IsOK())
    {
        /* New version available -> continue after connection success */
        printf("[OTA] Server connected\r\n");
        Version_Check_Get(ver, &len, &crc);
        printf("[OTA] New version %s found, length %lu bytes, CRC 0x%08lX\r\n",
               ver, (unsigned long)len, (unsigned long)crc);

        printf("\r\n[OTA] Update available? (y/n): ");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == 'y' || c == 'Y') {
                printf(" y\r\n");
                OTA_SetExpectedCRC(crc);
                OTA_Server_Start();
                return 0;   /* Enter receive mode */
            }
            if (c == 'n' || c == 'N' || c == 27) {   /* n / ESC */
                printf(" n\r\n[OTA] Update cancelled -> returning to menu\r\n");
                return 1;
            }
            /* Other keys / no key: ignore, continue waiting (LwIP processed synchronously) */
        }
    }
    else if (Version_Check_IsConnected())
    {
        /* Connected but already latest */
        printf("[OTA] Server connected, already on latest firmware\r\n");
        printf("[OTA] Press ESC to return to main screen\r\n");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == 27) { printf(" ESC\r\n"); return 1; }
        }
    }
    else
    {
        /* Cannot connect / timeout */
        printf("[OTA] Server connection failed\r\n");
        printf("[OTA] (1)Return to main screen   (2)Retry connection\r\n");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == '1') return 1;          /* Return to main screen */
            if (c == '2') goto retry;        /* Retry connection */
        }
    }
}

/**
  * @brief  Process one menu key. Returns 1 if menu should stay, 0 to exit loop.
  */
int Menu_HandleKey(int key)
{
    int reprint_menu = 0;   /* Whether to reprint entire menu (when returning from case '2') */

    switch (key)
    {
    case '0': Show_Help();            break;
    case '1': Show_Info();            break;
    case '2':
        /* If already receiving or confirming, avoid restarting server (freeze: won't bind again) */
        if (OTA_IsReceiving() || OTA_IsConfirm()) {
            printf("\r\n[OTA] OTA process already in progress\r\n");
            break;
        }
        if (Firmware_Update() == 0) {
            printf("[OTA] Receiving... (ESC to abort)\r\n");
            /* Entering OTA receive mode; main loop will call OTA_Poll */
        } else {
            Display_Menu();   /* Return to main screen: reprint entire menu */
            reprint_menu = 1;
        }
        break;
    case '3': Backup_SlotA();         break;
    case '4': Restore_SlotA();        break;
    case '9': QSPI_Erase_Test();      reprint_menu = 1;  Display_Menu();  break;
    case '6': printf("\r\n[BOOT] System reset...\r\n");
              HAL_Delay(50);
              NVIC_SystemReset();
              break;                  /* never returns */
    case '5': JumpToApp();            break;
    case 27:
        /* During receive/confirm: ESC = abort; main menu ESC = no action */
        if (OTA_IsReceiving() || OTA_IsConfirm())
            OTA_Abort();
        break;
    default:
        /* Unimplemented keys: don't display (avoid noise), can be extended */
        break;
    }

    /* If menu wasn't fully reprinted, append "Select (0-6):" prompt */
    if (!reprint_menu)
        printf("\r\nSelect (0-6): ");
    return 1;
}