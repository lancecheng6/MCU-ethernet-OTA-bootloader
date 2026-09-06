/**
  * @file    ota.c
  * @brief   OTA firmware receive core (LwIP Raw API TCP Server :8000)
  *
  *
  * NO_SYS architecture: callback receives data and writes to QSPI; state machine OTA_Poll() runs in main loop.
  */
#include "ota.h"
#include "crc32.h"
#include "metadata.h"
#include "bsp_debug_usart.h"
#include "bsp_qspi_flash.h"
#include "main.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdio.h>

#define OTA_PORT             8000
#define OTA_MAGIC            0x0041544F   /* "OTA\0" little-endian as u32 */
#define OTA_STAGING_ADDR     0           /* QSPI offset (memory-mapped 0x90000000+) */

/* 512KB SRAM buffer: received firmware stored here first (AXI SRAM via scatter) */
__attribute__((section(".ota_buf"))) uint8_t ota_buf[OTA_BUF_SIZE];

static struct tcp_pcb *ota_server_pcb = NULL;
static struct tcp_pcb *ota_client_pcb = NULL;
static volatile ota_state_t ota_state = OTA_STATE_IDLE;
static volatile int  ota_header_parsed = 0;
static volatile uint32_t ota_expected = 0;
static volatile uint32_t ota_received = 0;
static volatile uint32_t ota_crc_expected = 0;
static volatile uint8_t ota_stage[8];   /* First pbuf may contain header + partial data */

/* ===== Old: streaming write to QSPI (replaced by SRAM buffer, kept as comment for reference) =====
static uint8_t ota_page_buf[256];
static volatile uint32_t ota_page_off = 0;
static volatile uint32_t ota_cur_page_addr = 0;
static volatile uint32_t ota_flush_addr = 0;
static err_t ota_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t ota_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void   ota_err_cb(void *arg, err_t err);
static void ota_flush_page(void)
{
    if (ota_page_off > 0) {
        BSP_QSPI_Write(ota_page_buf, ota_cur_page_addr, ota_page_off);
        ota_flush_addr = ota_cur_page_addr + ota_page_off;
        ota_page_off = 0;
    }
}
static void ota_write_data(const uint8_t *data, uint32_t len)
{
    uint32_t i = 0;
    while (i < len) {
        uint32_t page_start = (ota_received + i) & ~(uint32_t)255;
        if (ota_cur_page_addr != page_start) {
            ota_flush_page();
            ota_cur_page_addr = page_start;
        }
        ota_page_buf[ota_page_off++] = data[i++];
        if (ota_page_off == 256) ota_flush_page();
    }
}
===== Old: streaming write to QSPI end ===== */

static err_t ota_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t ota_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void   ota_err_cb(void *arg, err_t err);

void OTA_Server_Start(void)
{
    err_t err;

    OTA_Server_Stop();
    ota_state = OTA_STATE_RECEIVING;
    ota_header_parsed = 0;
    ota_expected = 0;
    ota_received = 0;
    /* ota_page_off = 0; ota_cur_page_addr = 0; ota_flush_addr = 0; */   /* Old: streaming write */

    printf("[OTA] Waiting for firmware on :%d\r\n", OTA_PORT);

    ota_server_pcb = tcp_new();
    if (ota_server_pcb == NULL) { ota_state = OTA_STATE_ERROR; return; }
    /* Allow binding to local port still in TIME_WAIT (leftover from previous ESC abort) */
    ota_server_pcb->so_options |= SOF_REUSEADDR;
    err = tcp_bind(ota_server_pcb, IP_ADDR_ANY, OTA_PORT);
    if (err != ERR_OK) { printf("[OTA] bind fail %d\r\n", err); OTA_Server_Stop(); return; }
    ota_server_pcb = tcp_listen(ota_server_pcb);
    tcp_accept(ota_server_pcb, ota_accept_cb);
}

void OTA_Server_Stop(void)
{
    if (ota_client_pcb) { tcp_abort(ota_client_pcb); ota_client_pcb = NULL; }
    if (ota_server_pcb) { tcp_abort(ota_server_pcb); ota_server_pcb = NULL; }
    ota_state = OTA_STATE_IDLE;
}

void OTA_Abort(void)
{
    printf("\r\n[OTA] Aborted by user\r\n");
    OTA_Server_Stop();
}

/* Only reset state (don't clear TCP pcb) -- used for "staged, cancelled write" back to menu */
void OTA_ResetState(void)
{
    ota_state = OTA_STATE_IDLE;
}

static err_t ota_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    LWIP_UNUSED_ARG(arg); LWIP_UNUSED_ARG(err);
    if (ota_client_pcb != NULL) {
        printf("[OTA] Busy, rejecting client\r\n");
        tcp_abort(newpcb);
        return ERR_ABRT;
    }
    ota_client_pcb = newpcb;
    tcp_recv(ota_client_pcb, ota_recv_cb);
    tcp_err(ota_client_pcb, ota_err_cb);
    printf("[OTA] Client connected\r\n");
    return ERR_OK;
}

static err_t ota_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    LWIP_UNUSED_ARG(arg); LWIP_UNUSED_ARG(err);
    if (p == NULL) {
        printf("[OTA] Client disconnected (got %lu/%lu)\r\n",
               (unsigned long)ota_received, (unsigned long)ota_expected);
        ota_client_pcb = NULL;
        if (ota_state == OTA_STATE_RECEIVING) {
            if (ota_expected != 0 && ota_received >= ota_expected) {
                ota_state = OTA_STATE_VERIFY;   /* Handle in main loop */
            } else {
                printf("[OTA] Incomplete! Expected %lu got %lu\r\n",
                       (unsigned long)ota_expected, (unsigned long)ota_received);
                ota_state = OTA_STATE_ERROR;
            }
        }
        return ERR_OK;
    }

    struct pbuf *q;
    for (q = p; q != NULL; q = q->next) {
        uint8_t *data = (uint8_t *)q->payload;
        uint32_t len = q->len;

        /* Check if exceeding SRAM buffer size (512KB) */
        if (ota_received + len > OTA_BUF_SIZE) {
            printf("[OTA] Overflow!\r\n");
            ota_state = OTA_STATE_ERROR;
            tcp_recved(pcb, p->tot_len);
            pbuf_free(p);
            return ERR_OK;
        }

        /* Parse header (each pbuf may span header/data boundary) */
        while (len > 0) {
            if (!ota_header_parsed) {
                int need = 8 - (int)ota_received;
                int take = (len < (uint32_t)need) ? (int)len : need;
                memcpy((uint8_t*)ota_stage + ota_received, data, take);
                ota_received += take;
                data += take; len -= take;

                if (ota_received == 8) {
                    uint32_t magic = *(uint32_t*)ota_stage;
                    if (magic != OTA_MAGIC) {
                        printf("[OTA] Bad magic 0x%08X\r\n", (unsigned)magic);
                        ota_state = OTA_STATE_ERROR;
                        tcp_recved(pcb, p->tot_len);
                        pbuf_free(p);
                        return ERR_OK;
                    }
                    ota_expected = *(uint32_t*)(ota_stage + 4);
                    ota_received = 0;   /* Only count payload from now on */
                    ota_header_parsed = 1;
                    printf("[OTA] Header OK, size=%lu bytes\r\n", (unsigned long)ota_expected);

                    /* Old: erase entire QSPI before receiving (changed to erase after CRC verify, kept as comment) */
                    // if (BSP_QSPI_Erase_Chip() != QSPI_OK)
                    //     printf("[OTA] WARN: QSPI erase failed/slow\r\n");
                    // ota_page_off = 0; ota_cur_page_addr = 0; ota_flush_addr = 0;
                }
            } else {
                /* Receive into SRAM buffer (no longer streaming write to QSPI) */
                memcpy(ota_buf + ota_received, data, len);
                ota_received += len;
                data += len; len = 0;
            }
        }
    }

    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static void ota_err_cb(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    printf("[OTA] TCP err %d\r\n", err);
    ota_client_pcb = NULL;
    if (ota_state == OTA_STATE_RECEIVING) ota_state = OTA_STATE_ERROR;
}

/* ---- Main loop polling: handle VERIFY -> write to Slot A -> PENDING -> Reset ---- */
void OTA_Poll(void)
{
    if (ota_state != OTA_STATE_VERIFY) return;

    // /* flush last incomplete page */  (Old: streaming write to QSPI)
    // ota_flush_page();

    printf("[OTA] Received %lu bytes, computing CRC32...\r\n", (unsigned long)ota_received);
    /* Compute CRC directly in SRAM buffer (fast, no need to read back from QSPI) */
    uint32_t crc = CRC32_Calculate(ota_buf, ota_received);

    /* ===== Old: read back from QSPI to compute CRC (replaced by SRAM RAM CRC) =====
    uint8_t tmp[64];
    uint32_t crc = 0xFFFFFFFF;
    uint32_t off = 0;
    while (off < ota_received) {
        uint32_t chunk = (ota_received - off < sizeof(tmp)) ? (ota_received - off) : sizeof(tmp);
        BSP_QSPI_Read(tmp, OTA_STAGING_ADDR + off, chunk);
        for (uint32_t i = 0; i < chunk; i++) {
            uint8_t b = tmp[i];
            crc = crc ^ b;
            for (int bit = 0; bit < 8; bit++)
                crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
        off += chunk;
    }
    crc ^= 0xFFFFFFFF;
    ===== Old CRC end ===== */

    printf("[OTA] Calculated: 0x%08lX  Expected: 0x%08lX\r\n",
           (unsigned long)crc, (unsigned long)ota_crc_expected);

    if (crc != ota_crc_expected) {
        printf("[OTA] *** Checksum error! Aborting update ***\r\n");
        // BSP_QSPI_Erase_Sector(OTA_STAGING_ADDR);   /* Staging not yet erased, restart is fine, changed to comment */
        ota_state = OTA_STATE_ERROR;
        return;
    }

    printf("[OTA] CRC32 verified OK\r\n");

    /* Erase staging area 2MB (erase in 64KB blocks, avoid continuous erase causing hang) */
    printf("[OTA] Erasing QSPI staging (0x000000..0x1FFFFF)...\r\n");
    if (QSPI_Erase_Blocks(0, 2 * 1024 * 1024) != 0)
        printf("[OTA] WARN: some staging blocks erase failed (readback CRC will catch errors)\r\n");
    printf("[OTA] Erase staging done\r\n");

    /* Re-init QSPI after erase to clear driver hang from consecutive commands */
    if (BSP_QSPI_Init() != QSPI_OK) {
        printf("[OTA] QSPI re-init FAILED\r\n");
        ota_state = OTA_STATE_ERROR;
        return;
    }

    /* Write to staging: SRAM buffer -> QSPI (retry on failure) */
    printf("[OTA] Writing firmware to QSPI staging (0x%06lX, %lu bytes)...\r\n",
           (unsigned long)OTA_STAGING_ADDR, (unsigned long)ota_received);
    if (BSP_QSPI_Write(ota_buf, OTA_STAGING_ADDR, ota_received) != QSPI_OK) {
        printf("[OTA] Write staging FAILED, retry...\r\n");
        /* Retry: re-init + write again */
        BSP_QSPI_Init();
        if (BSP_QSPI_Write(ota_buf, OTA_STAGING_ADDR, ota_received) != QSPI_OK) {
            printf("[OTA] Write staging FAILED (retry)\r\n");
            ota_state = OTA_STATE_ERROR;
            return;
        }
    }
    printf("[OTA] Write staging OK (%lu bytes)\r\n", (unsigned long)ota_received);

    /* Read back staging and compare to verify QSPI write is correct */
    {
        uint8_t tmp[64];
        uint32_t crc_rb = 0xFFFFFFFF;
        uint32_t off = 0;
        while (off < ota_received) {
            uint32_t chunk = (ota_received - off < sizeof(tmp)) ? (ota_received - off) : sizeof(tmp);
            if (BSP_QSPI_Read(tmp, OTA_STAGING_ADDR + off, chunk) != QSPI_OK) {
                printf("[OTA] Readback FAILED @0x%08lX\r\n", (unsigned long)off);
                ota_state = OTA_STATE_ERROR;
                return;
            }
            for (uint32_t i = 0; i < chunk; i++) {
                uint8_t b = tmp[i];
                crc_rb ^= b;
                for (int bit = 0; bit < 8; bit++)
                    crc_rb = (crc_rb >> 1) ^ (0xEDB88320 & -(crc_rb & 1));
            }
            off += chunk;
        }
        crc_rb ^= 0xFFFFFFFF;
        printf("[OTA] Staging readback CRC32: 0x%08lX\r\n", (unsigned long)crc_rb);
        if (crc_rb == ota_crc_expected)
            printf("[OTA] Staging readback MATCH\r\n");
        else {
            printf("[OTA] Staging readback MISMATCH!\r\n");
            ota_state = OTA_STATE_ERROR;
            return;
        }
    }

    printf("[OTA] Firmware staged to QSPI (0x000000) staging area\r\n");
    ota_state = OTA_STATE_CONFIRM;   /* Waiting for user confirmation to write to App */
}

/**
  * @brief  Check if in "staged, waiting for user confirmation to write" state
  */
int OTA_IsConfirm(void)
{
    return (ota_state == OTA_STATE_CONFIRM);
}

/**
  * @brief  Write to App flow:
  *         1) Backup: read internal Flash Bank2 old version (0x08100000) -> external NOR backup (0x200000)
  *         2) Write: external NOR staging (0x000000) -> internal Flash Bank2 (0x08100000)
  *         3) Metadata write PENDING_UPDATE
  *         4) Caller handles Reset / jump
  * @retval 0 = success; 1 = failure
  */
#define APP_ADDR      0x08100000UL   /* Bank2 start = App execution area */
#define APP_SIZE      0x00100000UL   /* 1MB = sector8~15 (8 x 128KB sectors) */
#define BACKUP_OFFSET 0x200000UL     /* External NOR backup area */
#define FCACHE_LINE   32UL           /* FLASHWORD = 32 bytes */

int OTA_Flash_Write(void)
{
    uint32_t i;
    uint8_t tmp[64];

    printf("\r\n[OTA] Starting write to App at 0x08100000 (Bank2)...\r\n");

    /* ---- Step 1: Backup old Slot A -> external NOR backup (0x200000) ---- */
    printf("[OTA] Step 1: Backup old 0x08100000 -> NOR backup (0x200000)...\r\n");
    /* Erase backup area 1MB in batches (16 x 64KB blocks, reset QSPI every 8) */
    if (QSPI_Erase_Blocks(BACKUP_OFFSET, APP_SIZE) != 0) {
        printf("[OTA] Backup area erase failed, aborting OTA\r\n");
        return 1;
    }
    /* Read internal Flash Bank2 old version (1MB, 64B chunks) -> write to external NOR */
    for (i = 0; i < APP_SIZE; i += sizeof(tmp)) {
        memcpy(tmp, (void *)(APP_ADDR + i), sizeof(tmp));
        if (BSP_QSPI_Write(tmp, BACKUP_OFFSET + i, sizeof(tmp)) != QSPI_OK) {
            printf("[OTA] Backup write FAIL @0x%04lX\r\n", (unsigned long)i);
            return 1;
        }
    }
    printf("[OTA] Backup OK\r\n");

    /* ---- Step 2: Write QSPI staging (0x000000) -> internal Flash Bank2 (0x08100000) ---- */
    printf("[OTA] Step 2: Writing to 0x08100000 (Bank2)...\r\n");
    HAL_FLASH_Unlock();

    /* Erase Bank2 sectors 8~15 (erase one at a time to avoid timeout from erasing all 8 at once) */
    {
        /* Bank2 erase: sector uses "relative 0~7 within Bank";
           SNB field is only 3-bit (0~7), passing 8~15 overflows to 0~7.
           Erase one sector at a time + clear error flag before each write to avoid error accumulation. */
        FLASH_EraseInitTypeDef erase;
        uint32_t SectorError = 0;
        for (uint32_t s = 0; s < 8; s++) {
            /* Only erase sectors actually needed by App (128KB/sector) */
            if ((s * 128UL * 1024UL) >= ota_received) break;
            erase.TypeErase     = FLASH_TYPEERASE_SECTORS;
            erase.Banks         = FLASH_BANK_2;
            erase.Sector        = s;               /* 0~7 (relative) */
            erase.NbSectors     = 1;
            erase.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
            int ok = 0;
            for (int attempt = 0; attempt < 4 && !ok; attempt++) {
                SectorError = 0;
                /* Wait for Bank2 idle (previous operation may still be busy/queued) */
                uint32_t guard = 1000000;
                while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
                __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
                if (HAL_FLASHEx_Erase(&erase, &SectorError) != HAL_OK) {
                    printf("[OTA] Bank2 erase retry %d (sector %lu) SR2=0x%08lX\r\n",
                           attempt, (unsigned long)s, (unsigned long)FLASH->SR2);
                    /* Wait a bit more (BSY/QW residual) */
                    guard = 3000000;
                    while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
                } else {
                    ok = 1;
                }
            }
            if (!ok) {
                printf("[OTA] Bank2 erase FAILED (rel sector %lu)\r\n", (unsigned long)s);
                HAL_FLASH_Lock();
                return 1;
            }
            /* Wait for this erase to complete (BSY/QW clear) */
            uint32_t guard = 1000000;
            while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
            printf("[OTA] Erase Bank2 sector %lu/7 OK\r\n", (unsigned long)s);
        }
    }
    printf("[OTA] Bank2 erase done\r\n");

    /* Read back from QSPI staging -> write to Bank2 (FLASHWORD 32B, 32B aligned) */
    {
        /* FLASHWORD DataAddress requires 32B alignment; stack variables not guaranteed -> use static aligned buffer */
        static uint8_t wbuf[32] __attribute__((aligned(32)));
        uint32_t addr = APP_ADDR;
        uint32_t remain = ota_received;   /* Only write received length */
        printf("[OTA] Programming %lu bytes...\r\n", (unsigned long)remain);
        while (remain > 0) {
            uint32_t chunk = (remain >= sizeof(wbuf)) ? sizeof(wbuf) : remain;
            memset(wbuf, 0xFF, sizeof(wbuf));
            if (BSP_QSPI_Read(wbuf, OTA_STAGING_ADDR + (addr - APP_ADDR), chunk) != QSPI_OK) {
                printf("[OTA] QSPI read FAIL @0x%08lX\r\n", (unsigned long)addr);
                HAL_FLASH_Lock();
                return 1;
            }
            /* Clear flags before write; retry up to 3 times on failure */
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
            int ok = 0;
            for (int attempt = 0; attempt < 3 && !ok; attempt++) {
                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr,
                                      (uint32_t)wbuf) == HAL_OK) {
                    ok = 1;
                } else {
                    printf("[OTA] prog retry %d SR2=0x%08lX\r\n",
                           attempt, (unsigned long)FLASH->SR2);
                    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
                }
            }
            if (!ok) {
                printf("[OTA] Flash program FAIL @0x%08lX\r\n", (unsigned long)addr);
                HAL_FLASH_Lock();
                return 1;
            }
            addr   += chunk;
            remain -= chunk;
            if (((addr - APP_ADDR) % (64*1024)) == 0)
                printf("\r[OTA] Writing %lu%%",
                       (unsigned long)(100 * (addr - APP_ADDR) / ota_received));
        }
        printf("\r\n");
    }

    HAL_FLASH_Lock();

    /* Read back and verify Bank2 header */
    memcpy(tmp, (void *)APP_ADDR, 4);
    printf("[OTA] Bank2 header: 0x%02X%02X%02X%02X\r\n",
           tmp[0], tmp[1], tmp[2], tmp[3]);

    /* ---- Step 3: Metadata PENDING_UPDATE ---- */
    printf("[OTA] Step 3: Metadata PENDING_UPDATE...\r\n");
    {
        Metadata meta;
        Metadata_Read(&meta);
        meta.state = STATE_PENDING_UPDATE;
        /* Version field not updated yet (only update after CONFIRMED) */
        if (Metadata_Write(&meta) != 0) {
            printf("[OTA] Metadata write failed, aborting OTA (Bank2 written but state not updated)\r\n");
            return 1;
        }
    }
    printf("[OTA] Metadata OK\r\n");

    printf("[OTA] Step 4: Write complete -> caller will Reset and jump\r\n");
    ota_state = OTA_STATE_DONE;
    return 0;
}

ota_state_t OTA_GetState(void)   { return ota_state; }
uint32_t OTA_GetReceived(void)   { return ota_received; }
uint32_t OTA_GetExpected(void)   { return ota_expected; }
int OTA_IsReceiving(void)        { return (ota_state == OTA_STATE_RECEIVING); }
struct tcp_pcb *OTA_GetClientPcb(void) { return ota_client_pcb; }

/* ---- For Firmware_Update to set expected CRC -- */
void OTA_SetExpectedCRC(uint32_t crc) { ota_crc_expected = crc; }