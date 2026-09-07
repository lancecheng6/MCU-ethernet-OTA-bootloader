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

#define APP_RUN_ADDR  0x08100000UL   /* Bank2 = App 執行區 */

void Display_Menu(void)
{
    printf("\r\n");
    printf("===== OTA Bootloader v1.0 (Bare-metal) =====\r\n");
    printf("  [1] Info              [2] Firmware Update\r\n");
    printf("  [3] Backup            [4] Restore\r\n");
    printf("  [5] Run App           [6] Reboot\r\n");
    printf("  [0] Help\r\n");
    printf("============================================\r\n");
    printf("Select (0-6): ");
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

    /* RCC reset 原因 */
    printf("  Reset reason: 0x%08lX\r\n", (unsigned long)rsr);
    if (rsr & (1U << 26)) printf("                IWDG1 reset\r\n");
    if (rsr & (1U << 22)) printf("                PIN reset\r\n");
    if (rsr & (1U << 21)) printf("                BOR reset\r\n");
    if (rsr & (1U << 20)) printf("                D2 reset\r\n");
    if (rsr & (1U << 19)) printf("                D1 reset\r\n");
    if (rsr & (1U << 17)) printf("                CPU reset\r\n");
    if (rsr & (1U << 24)) printf("                Software reset\r\n");

    /* Metadata 診斷資訊 */
    Metadata_Read(&meta);
    printf("\r\n--- Metadata ---\r\n");
    printf("  magic       : 0x%08lX%s\r\n",
           (unsigned long)meta.magic,
           (meta.magic == METADATA_MAGIC) ? " (OK)" : " (INVALID)");
    printf("  state       : 0x%08lX (%s)\r\n",
           (unsigned long)meta.state, Metadata_StateStr(meta.state));
    printf("  current     : %lu.%lu.%lu  CRC=0x%08lX size=%lu\r\n",
           (unsigned long)meta.cur_major,
           (unsigned long)meta.cur_minor,
           (unsigned long)meta.cur_patch,
           (unsigned long)meta.crc32,
           (unsigned long)meta.fw_size);
    printf("  backup      : %lu.%lu.%lu  CRC=0x%08lX size=%lu\r\n",
           (unsigned long)meta.bak_major,
           (unsigned long)meta.bak_minor,
           (unsigned long)meta.bak_patch,
           (unsigned long)meta.bak_crc32,
           (unsigned long)meta.bak_fw_size);
    printf("  ota         : %lu.%lu.%lu\r\n",
           (unsigned long)meta.ota_major,
           (unsigned long)meta.ota_minor,
           (unsigned long)meta.ota_patch);
}

void Show_Help(void)
{
    printf("\r\n--- Help ---\r\n");
    printf("  [1] Info         - Show system info\r\n");
    printf("  [2] Firmware     - OTA update from PC server\r\n");
    printf("  [3] Backup       - Copy Slot A to NOR backup\r\n");
    printf("  [4] Restore      - Copy NOR backup to Slot A\r\n");
    printf("  [5] Run App      - Jump to Slot A / App\r\n");
    printf("  [6] Reboot       - Software reset\r\n");
    printf("  ESC / other     - Cancel or ignore\r\n");
}

/* --- Backup / Restore（[3]/[4]）--- */
#define APP_SIZE_C      (1UL * 1024UL * 1024UL)   /* Bank2 App 區域 = 1MB */
#define BACKUP_OFFSET   0x200000UL                /* 外部 NOR 備份區偏移（相對 0x90000000）*/

/* 等 QUADSPI 外設引擎空閒（檢查 QSPI_FLAG_BUSY） */
static void QSPI_WaitIdle(void)
{
    uint32_t guard = 1000000;
    while (__HAL_QSPI_GET_FLAG(&QSPIHandle, QSPI_FLAG_BUSY) && guard--) { }
}

/* [3] Backup：複製 Bank2(0x08100000) → 外部 NOR 備份(0x200000)，整 1MB */
void Backup_SlotA(void)
{
    uint8_t buf[4096];
    uint32_t off;

    printf("\r\n[Backup] Copy Bank2(0x08100000,1MB) -> NOR backup(0x200000)...\r\n");

    /* 1. 確認來源有效（Bank2 開頭向量非灰/0） */
    uint32_t msp = *(uint32_t *)APP_RUN_ADDR;
    if (msp == 0xFFFFFFFFU || msp == 0U) {
        printf("[Backup] No valid App (header=0x%08lX) -> abort\r\n", (unsigned long)msp);
        Display_Menu();
        return;
    }

    /* 2. 分批抹除備份區 1MB（16 個 64KB block，每 8 個重置 QSPI） */
    if (QSPI_Erase_Blocks(BACKUP_OFFSET, APP_SIZE_C) != 0) {
        printf("[Backup] Erase backup area FAILED\r\n");
        Display_Menu();
        return;
    }
    printf("[Backup] Backup area erased\r\n");

    /* 3. 讀 Bank2 → 寫 NOR 備份（4096 分塊） */
    for (off = 0; off < APP_SIZE_C; off += 4096) {
        QSPI_WaitIdle();
        memcpy(buf, (void *)(APP_RUN_ADDR + off), 4096);
        if (BSP_QSPI_Write(buf, BACKUP_OFFSET + off, 4096) != QSPI_OK) {
            printf("[Backup] Write backup FAILED @0x%08lX\r\n", (unsigned long)off);
            Display_Menu();
            return;
        }
        if ((off % (512UL * 1024UL)) == 0)
            printf("\r[Backup] %lu%%", (unsigned long)(100 * off / APP_SIZE_C));
    }
    printf("\r[Backup] 100%%\r\n");
    printf("[Backup] Done\r\n");

    /* 手動備份 = 目前版本記錄為可回滾版本（bak_*）*/
    {
        Metadata meta;
        Metadata_Read(&meta);
        meta.bak_major = meta.cur_major;
        meta.bak_minor = meta.cur_minor;
        meta.bak_patch = meta.cur_patch;
        meta.bak_crc32   = meta.crc32;
        meta.bak_fw_size = meta.fw_size;
        Metadata_Write(&meta);
    }

    Display_Menu();
}

/* [4] Restore：複製 外部 NOR 備份(0x200000) → Bank2(0x08100000)，整 1MB */
void Restore_SlotA(void)
{
    uint8_t hdr[4];
    uint8_t wbuf[32];
    uint32_t off, remain;

    printf("\r\n[Restore] Copy NOR backup(0x200000) -> Bank2(0x08100000,1MB)...\r\n");

    /* 1. 確認備份區有效（開頭非灰） */
    QSPI_WaitIdle();
    if (BSP_QSPI_Read(hdr, BACKUP_OFFSET, 4) != QSPI_OK) {
        printf("[Restore] Read backup area FAILED\r\n");
        Display_Menu();
        return;
    }
    if (hdr[0] == 0xFF && hdr[1] == 0xFF && hdr[2] == 0xFF && hdr[3] == 0xFF) {
        printf("[Restore] Backup area blank -> abort\r\n");
        Display_Menu();
        return;
    }

    /* 2. 抹除整個 Bank2（8 個 sector，逐一；每次等 SR2(BSY/QW) 清再下一個；失敗重試） */
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
            /* 等 Bank2 空閒 */
            guard = 1000000;
            while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
            if (HAL_FLASHEx_Erase(&erase, &SectorError) != HAL_OK) {
                printf("[Restore] sector %lu erase retry %d SR2=0x%08lX\r\n",
                       (unsigned long)s, attempt, (unsigned long)FLASH->SR2);
                /* 多等一點（BSY/QW 殘留） */
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
        /* 等本次抹除完成（SR2 BSY/QW 清） */
        guard = 1000000;
        while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
        printf("[Restore] Erase Bank2 sector %lu/7 OK\r\n", (unsigned long)s);
    }

    /* 3. 讀 NOR 備份 → FLASH_Program 寫回 Bank2（32B FLASHWORD） */
    printf("[Restore] Writing Bank2...\r\n");
    remain = APP_SIZE_C;
    off = 0;
    while (remain > 0) {
        uint32_t chunk = (remain >= 32) ? 32 : remain;
        memset(wbuf, 0xFF, sizeof(wbuf));
        QSPI_WaitIdle();
        if (BSP_QSPI_Read(wbuf, BACKUP_OFFSET + off, chunk) != QSPI_OK) {
            printf("[Restore] Read backup FAILED @0x%08lX\r\n", (unsigned long)off);
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

    /* 手動還原 = 確認舊版可用 → Metadata 標 CONFIRMED
       （Metadata_Write 內部：先複製到 SRAM → 抹 sector → 更新 state → 寫回）*/
    {
        Metadata meta;
        Metadata_Read(&meta);
        /* 手動還原 = 備份版 → 目前版本 = 備份版本 */
        meta.cur_major = meta.bak_major;
        meta.cur_minor = meta.bak_minor;
        meta.cur_patch = meta.bak_patch;
        meta.crc32   = meta.bak_crc32;
        meta.fw_size = meta.bak_fw_size;
        meta.state = STATE_CONFIRMED;
        if (Metadata_Write(&meta) != 0)
            printf("[Restore] Metadata write FAILED!\r\n");
        else
            printf("[Restore] Metadata -> CONFIRMED (version=backup)\r\n");
    }

    printf("[Restore] Done\r\n");
    Display_Menu();
}
/**
  * @brief  Normal boot: read Metadata, run IWDG health-check state machine.
  *         - PENDING_UPDATE -> mark TESTING -> enable IWDG(5s) -> jump to App
  *         - TESTING -> by reset reason: IWDG->rollback; non-IWDG->CONFIRMED(version=new)
  *         - CONFIRMED -> jump to App directly
  *         - EMPTY/invalid -> interactive menu
  */

/* ---- IWDG (independent watchdog) direct register writes ---- */
#define IWDG_KR_RELOAD   0xAAAAU   /* reload key */
#define IWDG_KR_UNLOCK   0x5555U   /* unlock PR/RLR */
#define IWDG_KR_START    0xCCCCU   /* start */
/* IWDG1 reset flag in RCC_RSR (bit 26; official CMSIS: RCC_RSR_IWDG1RSTF=0x04000000) */
#define RCC_RSR_IWDG1RSTF_Msk   (0x1UL << 26)

/* Enable IWDG, timeout timeout_ms (LSI 32kHz: reload=(ms*32)-1)
   Non-static: main() boot flow enables the dog when "KEY1 not pressed and state==TESTING". */
void Iwdg_Enable(uint32_t timeout_ms)
{
    /* 1. Start LSI (watchdog clock source) first and wait ready; else IWDG never ticks */
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }

    /* 2. Unlock + set PR/RLR + start IWDG
          PR: clock prescaler 64 (LSI 32k/64=500Hz -> decrement every 2ms) */
    IWDG1->KR = IWDG_KR_UNLOCK;
    IWDG1->PR  = 3;   /* 0b011: /64 */
    IWDG1->RLR = (timeout_ms / 2) - 1;   /* 500Hz -> every 2ms, reload count */
    IWDG1->KR = IWDG_KR_START;
}

/* Reload/feed the dog */
static void Iwdg_Refresh(void)
{
    IWDG1->KR = IWDG_KR_RELOAD;
}

/* Whether reset was caused by IWDG (read RCC->RSR, then clear) */
static int Reset_Reason_Is_IWDG(void)
{
    if (RCC->RSR & RCC_RSR_IWDG1RSTF_Msk) {
        RCC->RSR |= RCC_RSR_RMVF;   /* clear all reset flags */
        return 1;
    }
    RCC->RSR |= RCC_RSR_RMVF;
    return 0;
}

/* Rollback: copy NOR backup(0x200000) -> Bank2(0x08100000). Returns 0=OK */
static int Boot_DoRollback(void)
{
    uint8_t hdr[4];
    /* FLASHWORD DataAddress needs 32B alignment; stack vars not guaranteed -> use static aligned buffer */
    static uint8_t wbuf[32] __attribute__((aligned(32)));
    uint32_t off, remain;
    extern int qspi_inited;

    printf("[ROLLBACK] Restore NOR backup -> Bank2...\r\n");

    /* Ensure QSPI is inited (normal boot path never inits it, do it here) */
    if (!qspi_inited) {
        printf("[ROLLBACK] QSPI init...\r\n");
        if (BSP_QSPI_Init() != QSPI_OK) {
            printf("[ROLLBACK] QSPI init FAILED, cannot rollback\r\n");
            return -1;
        }
        qspi_inited = 1;
    }

    QSPI_WaitIdle();
    if (BSP_QSPI_Read(hdr, BACKUP_OFFSET, 4) != QSPI_OK) { printf("[ROLLBACK] Read FAILED\r\n"); return -1; }
    if (hdr[0]==0xFF && hdr[1]==0xFF && hdr[2]==0xFF && hdr[3]==0xFF) {
        printf("[ROLLBACK] Backup area blank, cannot rollback\r\n"); return -1;
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
        if (!ok) { printf("[ROLLBACK] Erase FAILED s%lu\r\n",(unsigned long)s); HAL_FLASH_Lock(); return -1; }
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
            printf("[ROLLBACK] Read backup FAILED\r\n"); HAL_FLASH_Lock(); return -1;
        }
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1|FLASH_FLAG_ALL_ERRORS_BANK2);
        int ok = 0;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, APP_RUN_ADDR+off, (uint32_t)wbuf)==HAL_OK) ok=1;
            else __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1|FLASH_FLAG_ALL_ERRORS_BANK2);
        }
        if (!ok) { printf("[ROLLBACK] Write back FAILED %08lX\r\n", (unsigned long)off); HAL_FLASH_Lock(); return -1; }
        off += chunk; remain -= chunk;
    }
    HAL_FLASH_Lock();
    printf("[ROLLBACK] Done\r\n");
    return 0;
}

/* Metadata state transitions + return boot requirement (runs before KEY1 check).
   Return values:
     0 -> invalid/EMPTY or rollback FAILED -> enter menu
     1 -> state==TESTING -> must enable IWDG before jumping to App (health-check start)
     2 -> CONFIRMED (or rollback done) -> jump to App directly
   State transitions:
     PENDING -> TESTING (write, no watchdog yet)
     TESTING + IWDG triggered -> rollback -> CONFIRMED (write)
     TESTING + non-IWDG -> stay TESTING (no direct CONFIRMED, let "enable-dog+jump" happen)
     CONFIRMED -> unchanged */
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
            printf("[BOOT] Metadata write FAILED (PENDING->TESTING)\r\n");
        printf("[BOOT] PENDING_UPDATE -> TESTING (pre-transition)\r\n");
        return 1;   /* now TESTING -> need watchdog jump */
    }
    else if (meta.state == STATE_TESTING)
    {
        if (Reset_Reason_Is_IWDG())
        {
            /* App rebooted by watchdog -> rollback */
            printf("[BOOT] TESTING + IWDG -> rollback\r\n");
            if (Boot_DoRollback() == 0)
            {
                /* Rollback = backup becomes current version */
                meta.cur_major = meta.bak_major;
                meta.cur_minor = meta.bak_minor;
                meta.cur_patch = meta.bak_patch;
                meta.crc32   = meta.bak_crc32;
                meta.fw_size = meta.bak_fw_size;
                meta.state = STATE_CONFIRMED;
                if (Metadata_Write(&meta) != 0)
                    printf("[BOOT] Metadata write FAILED (after rollback CONFIRMED)\r\n");
                printf("[BOOT] Rollback done -> CONFIRMED (version=backup)\r\n");
                return 2;
            }
            else
            {
                printf("[BOOT] Rollback FAILED -> stay TESTING (enter menu)\r\n");
                return 0;
            }
        }
        else
        {
            /* Non-IWDG reboot: stay TESTING so "enable-dog + jump to App" happens (health-check start) */
            printf("[BOOT] TESTING + non-IWDG -> keep TESTING, enable watchdog and jump to App\r\n");
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

    /* 1. Disable global IRQs + stop SysTick */
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* 2. Stop Ethernet DMA + QSPI (avoid background access; stop Ethernet only if inited) */
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

    /* 4. Flush caches */
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

/* Deferred Ethernet init (once only, on entering [2] Firmware Update) */
extern struct netif gnetif;
extern int flag;

static void Eth_Init_Once(void)
{
    if (eth_inited) return;
    printf("\r\n[ETH] Init Ethernet...\r\n");
    lwip_init();
    Netif_Config();
    User_notification(&gnetif);
    printf("LAN8720A Ethernet Demo\r\n");
    printf("LwIP version: %s\r\n", LWIP_VERSION_STRING);
    printf("Board IP : %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    eth_inited = 1;
}

/**
  * @brief  Wait for one key while pumping LwIP (avoid TCP closed/rx callback stall).
  *         Returns key ASCII, or 0 (no key, just pumped LwIP once).
  */
static int WaitKey_WithLwIP(void)
{
    if (flag) { flag = 0; ethernetif_input(&gnetif); }
    sys_check_timeouts();
    return UART_ReadKey_NonBlock();
}

/**
  * @brief  [2] Firmware Update.
  *         Init Ethernet on entry; then check version -> three-way branch.
  *         Each confirm wait uses "non-blocking key read + pump LwIP", no timeout.
  */
int Firmware_Update(void)
{
    printf("\r\n--- Firmware Update ---\r\n");
    Eth_Init_Once();

    /* Check version, on success print "server connected" first */
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
        /* New version available -> continue after connect OK */
        printf("[OTA] Server connected\r\n");
        Version_Check_Get(ver, &len, &crc);
        printf("[OTA] New version %s, length %lu bytes, CRC 0x%08lX\r\n",
               ver, (unsigned long)len, (unsigned long)crc);

        printf("\r\n[OTA] Update? (y/n): ");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == 'y' || c == 'Y') {
                printf(" y\r\n");
                OTA_SetExpectedCRC(crc);
                OTA_Server_Start();
                OTA_SendTrigger();
                printf("[OTA] Trigger packet sent, waiting for PC...\r\n");
                return 0;   /* enter receive mode */
            }
            if (c == 'n' || c == 'N' || c == 27) {   /* n / ESC */
                printf(" n\r\n[OTA] Update aborted -> back to menu\r\n");
                return 1;
            }
            /* Other keys / no key: ignore, keep waiting (LwIP already pumped) */
        }
    }
    else if (Version_Check_IsConnected())
    {
        /* Connected but already latest */
        printf("[OTA] Server connected, firmware already latest\r\n");
        printf("[OTA] Press ESC to return\r\n");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == 27) { printf(" ESC\r\n"); return 1; }
        }
    }
    else
    {
        /* Connect FAILED / timeout */
        printf("[OTA] Server connect FAILED\r\n");
        printf("[OTA] (1)Main menu   (2)Retry\r\n");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == '1') return 1;          /* back to main menu */
            if (c == '2') goto retry;        /* retry */
        }
    }
}

/**
  * @brief  Process one menu key. Returns 1 if menu should stay, 0 to exit loop.
  */
int Menu_HandleKey(int key)
{
    int reprint_menu = 0;   /* whether full menu was reprinted (case '2' return) */

    switch (key)
    {
    case '0': Show_Help();            break;
    case '1': Show_Info();            break;
    case '2':
        /* If already receiving/confirming, avoid restarting server (freeze: no re-bind) */
        if (OTA_IsReceiving() || OTA_IsConfirm()) {
            printf("\r\n[OTA] OTA already in progress\r\n");
            break;
        }
        if (Firmware_Update() == 0) {
            printf("[OTA] Receiving... (ESC to abort)\r\n");
            /* Enter OTA receive; main loop will call OTA_Poll */
        } else {
            Display_Menu();   /* back to main screen: reprint full menu */
            reprint_menu = 1;
        }
        break;
    case '3': Backup_SlotA();         break;
    case '4': Restore_SlotA();        break;
    case '6': printf("\r\n[BOOT] System reset...\r\n");
              HAL_Delay(50);
              NVIC_SystemReset();
              break;                  /* never returns */
    case '5': JumpToApp();            break;
    case 27:
        /* ESC during rx/confirm = abort; ESC in main menu = no-op */
        if (OTA_IsReceiving() || OTA_IsConfirm())
            OTA_Abort();
        break;
    default:
        /* Unimplemented key: silent (avoid noise), extensible */
        break;
    }

    /* If full menu not reprinted, print "Select (0-6):" prompt */
    if (!reprint_menu)
        printf("\r\nSelect (0-6): ");
    return 1;
}