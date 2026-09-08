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

    /* RCC reset reason */
    printf("  Reset reason: 0x%08lX\r\n", (unsigned long)rsr);
    if (rsr & (1U << 26)) printf("                IWDG1 reset\r\n");
    if (rsr & (1U << 22)) printf("                PIN reset\r\n");
    if (rsr & (1U << 21)) printf("                BOR reset\r\n");
    if (rsr & (1U << 20)) printf("                D2 reset\r\n");
    if (rsr & (1U << 19)) printf("                D1 reset\r\n");
    if (rsr & (1U << 17)) printf("                CPU reset\r\n");
    if (rsr & (1U << 24)) printf("                Software reset\r\n");

    /* Metadata diagnostic info */
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
#define APP_SIZE_C      (1UL * 1024UL * 1024UL)   /* Bank2 App area = 1MB */
#define BACKUP_OFFSET   0x200000UL                /* External NOR backup area offset (relative to 0x90000000) */

/* Wait for QUADSPI peripheral engine idle (check QSPI_FLAG_BUSY) */
static void QSPI_WaitIdle(void)
{
    uint32_t guard = 1000000;
    while (__HAL_QSPI_GET_FLAG(&QSPIHandle, QSPI_FLAG_BUSY) && guard--) { }
}

/* [3] Backup: Copy Bank2(0x08100000) -> External NOR backup(0x200000), full 1MB */
void Backup_SlotA(void)
{
    uint8_t buf[4096];
    uint32_t off;

    printf("\r\n[Backup] Copying Bank2(0x08100000,1MB) → NOR backup(0x200000)...\r\n");

    /* 1. Verify source valid (Bank2 start vector not 0xFF/0) */
    uint32_t msp = *(uint32_t *)APP_RUN_ADDR;
    if (msp == 0xFFFFFFFFU || msp == 0U) {
        printf("[Backup] No valid App (start=0x%08lX) -> Aborted\r\n", (unsigned long)msp);
        return;
    }

    /* 2. Erase backup area 1MB in batches (16 x 64KB blocks, reset QSPI every 8) */
    if (QSPI_Erase_Blocks(BACKUP_OFFSET, APP_SIZE_C) != 0) {
        printf("[Backup] Erase backup area failed\r\n");
        return;
    }
    printf("[Backup] Backup area erase complete\r\n");

    /* 3. Read Bank2 -> Write NOR backup (4096-byte chunks) */
    for (off = 0; off < APP_SIZE_C; off += 4096) {
        QSPI_WaitIdle();
        memcpy(buf, (void *)(APP_RUN_ADDR + off), 4096);
        if (BSP_QSPI_Write(buf, BACKUP_OFFSET + off, 4096) != QSPI_OK) {
            printf("[Backup] Write backup failed @0x%08lX\r\n", (unsigned long)off);
            return;
        }
        if ((off % (512UL * 1024UL)) == 0)
            printf("\r[Backup] %lu%%", (unsigned long)(100 * off / APP_SIZE_C));
    }
    printf("\r[Backup] 100%%\r\n");
    printf("[Backup] Done\r\n");

    /* Manual backup = record current version as rollback version（bak_*）*/
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
}

/* [4] Restore: Copy External NOR backup(0x200000) -> Bank2(0x08100000), full 1MB */
void Restore_SlotA(void)
{
    uint8_t hdr[4];
    uint8_t wbuf[32];
    uint32_t off, remain;

    printf("\r\n[Restore] Copying NOR backup(0x200000) → Bank2(0x08100000,1MB)...\r\n");

    /* 1. Verify backup area valid (start not 0xFF) */
    QSPI_WaitIdle();
    if (BSP_QSPI_Read(hdr, BACKUP_OFFSET, 4) != QSPI_OK) {
        printf("[Restore] Read backup area failed\r\n");
        return;
    }
    if (hdr[0] == 0xFF && hdr[1] == 0xFF && hdr[2] == 0xFF && hdr[3] == 0xFF) {
        printf("[Restore] Backup area empty -> Aborted\r\n");
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
                /* Wait longer (BSY/QW residual) */
                guard = 3000000;
                while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
            } else {
                ok = 1;
            }
        }
        if (!ok) {
            printf("[Restore] Bank2 erase failed (sector %lu) SR2=0x%08lX\r\n",
                   (unsigned long)SectorError, (unsigned long)FLASH->SR2);
            HAL_FLASH_Lock();
            return;
        }
        /* Wait for current erase to complete (SR2 BSY/QW clear) */
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
            printf("[Restore] Read backup failed @0x%08lX\r\n", (unsigned long)off);
            HAL_FLASH_Lock();
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
            return;
        }
        off    += chunk;
        remain -= chunk;
        if ((off % (512UL * 1024UL)) == 0)
            printf("\r[Restore] Writing %lu%%", (unsigned long)(100 * off / APP_SIZE_C));
    }
    printf("\r[Restore] Writing 100%%\r\n");
    HAL_FLASH_Lock();

    /* Manual restore = verify old version available -> Mark Metadata CONFIRMED
       (Metadata_Write internal: copy to SRAM -> erase sector -> update state -> write back) */
    {
        Metadata meta;
        Metadata_Read(&meta);
        /* Manual restore = backup version -> Current version = backup version */
        meta.cur_major = meta.bak_major;
        meta.cur_minor = meta.bak_minor;
        meta.cur_patch = meta.bak_patch;
        meta.crc32   = meta.bak_crc32;
        meta.fw_size = meta.bak_fw_size;
        meta.state = STATE_CONFIRMED;
        if (Metadata_Write(&meta) != 0)
            printf("[Restore] Metadata write failed!\r\n");
        else
            printf("[Restore] Metadata -> CONFIRMED (version=backup)\r\n");
    }

    printf("[Restore] Done\r\n");
}
/**
  * @brief  Normal boot: read Metadata, run IWDG health-check state machine.
  *         - PENDING_UPDATE → mark TESTING -> enable IWDG(5s) -> jump to App
  *         - TESTING → by reset reason: IWDG->rollback; non-IWDG->CONFIRMED(version=new)
  *         - CONFIRMED → jump to App directly
  *         - EMPTY/invalid -> interactive menu
  */

/* ---- IWDG (independent watchdog) direct register write ---- */
#define IWDG_KR_RELOAD   0xAAAAU   /* Reload key value */
#define IWDG_KR_UNLOCK   0x5555U   /* Unlock PR/RLR */
#define IWDG_KR_START    0xCCCCU   /* Start */
/* RCC_RSR IWDG1 reset flag (bit 26; official CMSIS: RCC_RSR_IWDG1RSTF=0x04000000) */
#define RCC_RSR_IWDG1RSTF_Msk   (0x1UL << 26)

/* Enable IWDG, timeout timeout_ms (LSI 32kHz: reload=(ms*32)-1)
   Non-static: used by main() boot flow to enable watchdog when "KEY1 not pressed and state==TESTING". */
void Iwdg_Enable(uint32_t timeout_ms)
{
    /* 1. Start LSI (watchdog clock source) and wait for ready; otherwise IWDG will not count */
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }

    /* 2. Unlock + set PR/RLR + start IWDG
          PR: clock divider 64 (LSI 32k/64=500Hz -> decrement every 2ms) */
    IWDG1->KR = IWDG_KR_UNLOCK;
    IWDG1->PR  = 3;   /* 0b011: /64 */
    IWDG1->RLR = (timeout_ms / 2) - 1;   /* 500Hz → every 2ms, reload count */
    IWDG1->KR = IWDG_KR_START;
}

/* Reload/feed watchdog */
static void Iwdg_Refresh(void)
{
    IWDG1->KR = IWDG_KR_RELOAD;
}

/* Check if IWDG reset (read RCC->RSR and clear) */
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

    printf("[ROLLBACK] from NOR backup→restoring Bank2...\r\n");

    /* Ensure QSPI initialized (normal boot path does not init, must do here) */
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
        if (!ok) { printf("[ROLLBACK] Write-back failed %08lX\r\n", (unsigned long)off); HAL_FLASH_Lock(); return -1; }
        off += chunk; remain -= chunk;
    }
    HAL_FLASH_Lock();
    printf("[ROLLBACK] Complete\r\n");
    return 0;
}

/* Metadata state transition + return boot requirement (executed before KEY1 check).
   Return values:
     0 -> invalid/EMPTY or rollback failed -> enter menu
     1 -> state==TESTING -> need to enable IWDG before jumping to App (health-check start)
     2 -> CONFIRMED (or rollback complete) -> jump to App directly
   State transitions:
     PENDING -> TESTING (write, no watchdog)
     TESTING + IWDG triggered -> rollback -> CONFIRMED (write)
     TESTING + non-IWDG -> still TESTING (do not directly CONFIRMED, let "enable watchdog + jump" happen)
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
        printf("[BOOT] PENDING_UPDATE -> TESTING (pre-transfer)\r\n");
        return 1;   /* become TESTING -> need watchdog jump */
    }
    else if (meta.state == STATE_TESTING)
    {
        if (Reset_Reason_Is_IWDG())
        {
            /* App reset by watchdog -> rollback */
            printf("[BOOT] TESTING + IWDG -> Rollback\r\n");
            if (Boot_DoRollback() == 0)
            {
                /* Rollback = backup version -> Current version = backup version */
                meta.cur_major = meta.bak_major;
                meta.cur_minor = meta.bak_minor;
                meta.cur_patch = meta.bak_patch;
                meta.crc32   = meta.bak_crc32;
                meta.fw_size = meta.bak_fw_size;
                meta.state = STATE_CONFIRMED;
                if (Metadata_Write(&meta) != 0)
                    printf("[BOOT] Metadata write failed (post-rollback CONFIRMED)\r\n");
                printf("[BOOT] Rollback complete -> CONFIRMED (version=backup)\r\n");
                return 2;
            }
            else
            {
                printf("[BOOT] Rollback failed -> Stay in TESTING (enter menu)\r\n");
                return 0;
            }
        }
        else
        {
            /* Non-IWDG reset: still TESTING, let "enable watchdog + jump to App" happen (health-check start) */
            printf("[BOOT] TESTING + non-IWDG -> Stay TESTING, enable watchdog and jump to App\r\n");
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

    /* 2. Stop Ethernet DMA + QSPI (prevent background access; Ethernet only needs stop if initialized) */
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

/* Ethernet deferred init (once only, init when entering [2] Firmware Update) */
extern struct netif gnetif;
extern int flag;

static void Eth_Init_Once(void)
{
    if (eth_inited) return;
    printf("\r\n[ETH] Initializing Ethernet...\r\n");
    lwip_init();
    Netif_Config();
    User_notification(&gnetif);
    printf("LAN8720A Ethernet Demo\r\n");
    printf("LwIP version: %s\r\n", LWIP_VERSION_STRING);
    printf("Board IP: %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    eth_inited = 1;
}

/**
  * @brief  Wait for a key while continuously running LwIP (prevent TCP closed/receive callback hang).
  *         Return key ASCII, or 0 (no key, just run LwIP once).
  */
static int WaitKey_WithLwIP(void)
{
    if (flag) { flag = 0; ethernetif_input(&gnetif); }
    sys_check_timeouts();
    return UART_ReadKey_NonBlock();
}

/**
  * @brief  [2] Firmware Update.
  *         Initialize Ethernet on entry；then checkversion → three-way branch。
  *         All confirm waits use "non-blocking key read + run LwIP simultaneously", no timeout.
  */
int Firmware_Update(void)
{
    printf("\r\n--- Firmware Update ---\r\n");
    Eth_Init_Once();

    /* Check version, on success print "Server connected" */
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
        /* New version found -> continue after connection success */
        printf("[OTA] Server connected\r\n");
        Version_Check_Get(ver, &len, &crc);
        printf("[OTA] New version found: %s, length %lu bytes, CRC 0x%08lX\r\n",
               ver, (unsigned long)len, (unsigned long)crc);

        printf("\r\n[OTA] Update now? (y/n): ");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == 'y' || c == 'Y') {
                printf(" y\r\n");
                OTA_SetExpectedCRC(crc);
                if (OTA_Server_Start() != 0) {
                    printf("[OTA] :8000 listen failed -> Update aborted\r\n");
                    OTA_Server_Stop();
                    OTA_ResetState();
                    printf("[OTA] Update cancelled -> Back to menu\r\n");
                    return 1;           /* return to menu (caller will Display_Menu) */
                }
                /* send once -> wait 2 seconds -> retry on no response, max 3 rounds */
                int connected = 0;
                for (int t = 0; t < 3; t++) {
                    OTA_SendTrigger();   /* send 1 packet */
                    printf("[OTA] Trigger packet sent (%d/3), waiting for PC connection...\r\n", t + 1);
                    uint32_t tw0 = HAL_GetTick();
                    while ((HAL_GetTick() - tw0) < 2000) {
                        if (flag) { flag = 0; ethernetif_input(&gnetif); }
                        sys_check_timeouts();
                        if (OTA_GetClientPcb() != NULL) { connected = 1; break; }
                    }
                    if (connected) break;
                }
                if (!connected) {
                    printf("[OTA] 3 triggers with no response, PC not connected -> Update aborted\r\n");
                    OTA_Server_Stop();
                    OTA_ResetState();   /* A2：keep original call */
                    printf("[OTA] Update cancelled -> Back to menu\r\n");
                    return 1;           /* return to menu (caller will Display_Menu) */
                }
                printf("[OTA] PC connected, starting firmware receive...\r\n");
                return 0;   /* enter receive mode */
            }
            if (c == 'n' || c == 'N' || c == 27) {   /* n / ESC */
                printf(" n\r\n[OTA] Update cancelled -> Back to menu\r\n");
                return 1;
            }
            /* Other key / no key: ignore, continue waiting (LwIP handled synchronously) */
        }
    }
    else if (Version_Check_IsConnected())
    {
        /* Connected but already up-to-date */
        printf("[OTA] Server connected, firmware is already up-to-date\r\n");
        printf("[OTA] Press ESC to return to main menu\r\n");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == 27) { printf(" ESC\r\n"); return 1; }
        }
    }
    else
    {
        /* Cannot connect / timeout */
        printf("[OTA] Server connection failed\r\n");
        printf("[OTA] (1)Return to menu   (2)Retry connection\r\n");
        for (;;) {
            c = WaitKey_WithLwIP();
            if (c == '1') return 1;          /* return to main screen */
            if (c == '2') goto retry;        /* retry connection */
        }
    }
}

/**
  * @brief  Process one menu key. Returns 1 if menu should stay, 0 to exit loop.
  */
int Menu_HandleKey(int key)
{
    int reprint_menu = 0;   /* Whether entire menu was reprinted (on case '2' return) */

    switch (key)
    {
    case '0': Show_Help(); Display_Menu(); reprint_menu = 1; break;
    case '1': Show_Info(); Display_Menu(); reprint_menu = 1; break;
    case '2':
        /* If already receiving or confirming, avoid starting server again (freeze: no more bind) */
        if (OTA_IsReceiving() || OTA_IsConfirm()) {
            printf("\r\n[OTA] OTA process already in progress\r\n");
            break;
        }
        if (Firmware_Update() == 0) {
            printf("[OTA] Receiving... (ESC to abort)\r\n");
            /* Enter OTA receive; main loop will call OTA_Poll */
        } else {
            Display_Menu();   /* Return to main screen: reprint entire menu */
            reprint_menu = 1;
        }
        break;
    case '3': Backup_SlotA(); Display_Menu(); reprint_menu = 1; break;
    case '4': Restore_SlotA(); Display_Menu(); reprint_menu = 1; break;
    case '6': printf("\r\n[BOOT] System reset...\r\n");
              HAL_Delay(50);
              NVIC_SystemReset();
              break;                  /* never returns */
    case '5': JumpToApp();            break;
    case 27:
        /* ESC during receive/confirm = abort; ESC in main menu = no action */
        if (OTA_IsReceiving() || OTA_IsConfirm())
            OTA_Abort();
        break;
    default:
        /* Unimplemented key: no display (avoid noise), extensible */
        break;
    }

    /* If entire menu was not reprinted, print the "Select (0-6):" prompt */
    if (!reprint_menu)
        printf("\r\nSelect (0-6): ");
    return 1;
}