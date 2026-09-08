/**
  * @file    ota.c
  * @brief   OTA receive core (LwIP Raw API TCP Server :8000)
  *
  * Flow (refer to pseudocode.md):
  *   TCP receive header (magic+len) -> write while receiving QSPI staging area -> receive complete -> CRC32 comparison
  *   → Backup Slot A → Write Slot A -> Metadata PENDING_UPDATE → Reset
  *
  * NO_SYS architecture: callback receives data and writes QSPI; state machine OTA_Poll() runs in main loop.
  */
#include "ota.h"
#include "crc32.h"
#include "metadata.h"
#include "version_check.h"
#include "bsp_debug_usart.h"
#include "bsp_qspi_flash.h"
#include "main.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include <string.h>
#include <stdio.h>

#define OTA_PORT             8000
#define OTA_MAGIC            0x0041544F   /* "OTA\0" little-endian as u32 */
#define OTA_STAGING_ADDR     0           /* QSPI offset (memory-mapped 0x90000000+) */

/* UDP trigger packet: notify PC ota_send.py to connect back to :8000 after pressing y */
/* Single send + wait for retry: send 1 -> wait 2s no response then retry, max 3 rounds (replaces old burst-3) */
#define OTA_TRIGGER_PORT     8002
#define OTA_TRIGGER_MAGIC    "OTA_TRIGGER"
#define OTA_TRIGGER_RETRY_MAX 3         /* Retry limit (rounds) */
#define OTA_TRIGGER_WAIT_MS  2000        /* Wait window per round */

/* 512KB SRAM buffer: store received server firmware here first (AXI SRAM via scatter) */
__attribute__((section(".ota_buf"))) uint8_t ota_buf[OTA_BUF_SIZE];

static struct tcp_pcb *ota_server_pcb = NULL;
static struct tcp_pcb *ota_client_pcb = NULL;
static volatile ota_state_t ota_state = OTA_STATE_IDLE;
static volatile int  ota_header_parsed = 0;
static volatile uint32_t ota_expected = 0;
static volatile uint32_t ota_received = 0;
static volatile uint32_t ota_crc_expected = 0;
static volatile uint8_t ota_stage[8];   /* First pbuf may contain file header + partial data */

/* ===== Old: write to QSPI while receiving (now using SRAM buffer, kept as comment for reference) =====
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
===== Old: write to QSPI while receiving end ===== */

static err_t ota_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t ota_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void   ota_err_cb(void *arg, err_t err);

/* Return 0=listen success, non-0=failure (tcp_new/bind failed, caller should return to menu, do not send trigger) */
int OTA_Server_Start(void)
{
    err_t err;

    OTA_Server_Stop();
    ota_state = OTA_STATE_RECEIVING;
    ota_header_parsed = 0;
    ota_expected = 0;
    ota_received = 0;
    /* ota_page_off = 0; ota_cur_page_addr = 0; ota_flush_addr = 0; */   /* Old: write while receiving */

    printf("[OTA] Waiting for firmware on :%d\r\n", OTA_PORT);

    ota_server_pcb = tcp_new();
    if (ota_server_pcb == NULL) { ota_state = OTA_STATE_ERROR; return -1; }
    /* Allow binding to local port still in TIME_WAIT (leftover from previous ESC abort) */
    ota_server_pcb->so_options |= SOF_REUSEADDR;
    err = tcp_bind(ota_server_pcb, IP_ADDR_ANY, OTA_PORT);
    if (err != ERR_OK) { printf("[OTA] bind fail %d\r\n", err); OTA_Server_Stop(); return -1; }
    ota_server_pcb = tcp_listen(ota_server_pcb);
    tcp_accept(ota_server_pcb, ota_accept_cb);
    return 0;
}

/* Send UDP trigger packet to PC after pressing y (ota_send.py --listen connects back to :8000).
   :8000 already listening (done by OTA_Server_Start), so no race condition.
   Send 1 packet: retry decided by caller (menu.c y-branch) based on response.
   fire-and-forget: on packet loss user can still run manually (ota_send.py without arguments). */
void OTA_SendTrigger(void)
{
    struct udp_pcb *pcb;
    ip_addr_t pc_ip;
    struct pbuf *p;
    const char *magic = OTA_TRIGGER_MAGIC;
    u16_t magic_len = (u16_t)strlen(magic);

    IP4_ADDR(&pc_ip, IP_ADDR0, IP_ADDR1, IP_ADDR2, 1); /* gateway = PC (same version check) */

    pcb = udp_new();
    if (pcb == NULL) { printf("[OTA] trigger: udp_new fail\r\n"); return; }

    p = pbuf_alloc(PBUF_TRANSPORT, magic_len, PBUF_RAM);
    if (p == NULL) { printf("[OTA] trigger: pbuf fail\r\n"); udp_remove(pcb); return; }
    memcpy(p->payload, magic, magic_len);
    udp_sendto(pcb, p, &pc_ip, OTA_TRIGGER_PORT);
    pbuf_free(p);
    udp_remove(pcb);
}

void OTA_Server_Stop(void)
{
    if (ota_client_pcb) {
        tcp_abort(ota_client_pcb); ota_client_pcb = NULL;
    }
    if (ota_server_pcb) {
        /* LISTEN pcb use close (release only, no RST, no err callback) safer than abort */
        if (tcp_close(ota_server_pcb) != ERR_OK) {
            printf("[OTA] listener close fail, abort\r\n");
            tcp_abort(ota_server_pcb);
        }
        ota_server_pcb = NULL;
    }
    ota_state = OTA_STATE_IDLE;
}

void OTA_Abort(void)
{
    printf("\r\n[OTA] Aborted by user\r\n");
    OTA_Server_Stop();
}

/* Only reset state (do not clear TCP pcb) -- used after "staging complete, cancel write" to return to menu */
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
        /* Passive close: complete FIN handshake and release pcb, otherwise :8000 stays occupied and next bind gets -8(ERR_USE) */
        if (tcp_close(pcb) != ERR_OK) {
            printf("[OTA] tcp_close fail, abort\r\n");   /* TODO-debug: for debugging, remove after stable */
            tcp_abort(pcb);   /* force abort if cannot close, do not leak pcb */
        }
        if (ota_state == OTA_STATE_RECEIVING) {
            if (ota_expected != 0 && ota_received >= ota_expected) {
                ota_state = OTA_STATE_VERIFY;   /* handled by main loop */
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

        /* Parse file header (each pbuf may span header/data boundary) */
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
                    ota_received = 0;   /* Only count as payload after this */
                    ota_header_parsed = 1;
                    printf("[OTA] Header OK, size=%lu bytes\r\n", (unsigned long)ota_expected);

                    /* Old: erase entire QSPI before receiving (changed to erase after CRC, kept as comment) */
                    // if (BSP_QSPI_Erase_Chip() != QSPI_OK)
                    //     printf("[OTA] WARN: QSPI erase failed/slow\r\n");
                    // ota_page_off = 0; ota_cur_page_addr = 0; ota_flush_addr = 0;
                }
            } else {
                /* Receive into SRAM buffer (no longer writing to QSPI while receiving) */
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

/* ---- Main loop poll: handle VERIFY -> Write Slot A -> PENDING -> Reset ---- */
void OTA_Poll(void)
{
    if (ota_state != OTA_STATE_VERIFY) return;

    // /* flush last incomplete page */  (Old: write to QSPI while receiving)
    // ota_flush_page();

    printf("[OTA] Received %lu bytes, computing CRC32...\r\n", (unsigned long)ota_received);
    /* Calculate CRC directly in SRAM buffer (fast, no need to read back from QSPI) */
    uint32_t crc = CRC32_Calculate(ota_buf, ota_received);

    /* ===== Old: read back from QSPI to calculate CRC (now using SRAM CRC) =====
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
        // BSP_QSPI_Erase_Sector(OTA_STAGING_ADDR);   /* staging area not yet erased, reboot is fine, changed to comment */
        ota_state = OTA_STATE_ERROR;
        return;
    }

    printf("[OTA] CRC32 verified OK\r\n");

    /* Erase staging area 2MB (erase 64KB blocks in batches, avoid too many consecutive erases causing hang) */
    printf("[OTA] Erasing QSPI staging (0x000000..0x1FFFFF)...\r\n");
    if (QSPI_Erase_Blocks(0, 2 * 1024 * 1024) != 0)
        printf("[OTA] WARN: some staging blocks erase failed (readback CRC will catch it)\r\n");
    printf("[OTA] Erase staging done\r\n");

    /* Re-initialize QSPI after erase to clear potential driver hang from consecutive bulk commands */
    if (BSP_QSPI_Init() != QSPI_OK) {
        printf("[OTA] QSPI re-init FAILED\r\n");
        ota_state = OTA_STATE_ERROR;
        return;
    }

    /* Write to staging area: SRAM buffer -> QSPI (retry on failure) */
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

    /* Read back staging area and compare to verify QSPI write correctness */
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
    ota_state = OTA_STATE_CONFIRM;   /* waiting for user to confirm whether to write App */
}

/**
  * @brief  Whether in "staged, waiting for user to confirm write" state
  */
int OTA_IsConfirm(void)
{
    return (ota_state == OTA_STATE_CONFIRM);
}

/**
  * @brief  Write App flow:
  *         1) Backup: read old version from internal Flash Bank2(0x08100000) -> external NOR backup(0x200000)
  *         2) Write: external NOR staging(0x000000) -> internal Flash Bank2(0x08100000)
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

    printf("\r\n[OTA] Starting App write to 0x08100000 (Bank2)...\r\n");

    /* ---- Step 1: Backup old Slot A -> External NOR backup(0x200000) ---- */
    printf("[OTA] Step 1: Backup old 0x08100000 -> NOR backup(0x200000)...\r\n");
    /* Erase backup area 1MB in batches (16 x 64KB blocks, reset QSPI every 8) */
    if (QSPI_Erase_Blocks(BACKUP_OFFSET, APP_SIZE) != 0) {
        printf("[OTA] Backup area erase failed, OTA aborted\r\n");
        return 1;
    }
    /* Read old version from internal Flash Bank2 (1MB, 64B chunks) -> write to external NOR
       Reset QSPI every 128KB (same as erase every-8-block reset pattern, prevent cumulative hang);
       retry each chunk 3 times (reset before retry). */
    for (i = 0; i < APP_SIZE; i += sizeof(tmp)) {
        if (i != 0 && (i % (128UL * 1024UL)) == 0)
            BSP_QSPI_Init();   /* controller idle reset, safe */
        memcpy(tmp, (void *)(APP_ADDR + i), sizeof(tmp));
        int wok = 0;
        for (int wa = 0; wa < 3 && !wok; wa++) {
            if (BSP_QSPI_Write(tmp, BACKUP_OFFSET + i, sizeof(tmp)) == QSPI_OK) {
                wok = 1;
            } else {
                BSP_QSPI_Init();
            }
        }
        if (!wok) {
            printf("[OTA] Backup write FAIL @0x%04lX\r\n", (unsigned long)i);
            BSP_QSPI_Init();   /* restore QSPI on failure path first, ensure print/menu return not affected by sick QSPI */
            return 1;
        }
    }
    printf("[OTA] Backup OK\r\n");

    /* Record "backed-up old version" to bak field (corresponds to old App just backed up to NOR) */
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

    /* ---- Step 2: Write QSPI staging(0x000000) -> Internal Flash Bank2(0x08100000) ---- */
    printf("[OTA] Step 2: Writing to 0x08100000 (Bank2)...\r\n");
    HAL_FLASH_Unlock();

    /* Erase Bank2 sector 8~15 (one by one, avoid timeout from erasing 8 at once) */
    {
        /* Bank2 erase: sector uses "relative 0~7 within Bank";
           SNB field is only 3-bit (0~7), passing 8~15 overflows to 0~7.
           One sector at a time + clear error flag before each write to prevent error accumulation. */
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
            for (int attempt = 0; attempt < 8 && !ok; attempt++) {
                SectorError = 0;
                /* Wait for Bank2 idle (previous operation may still be busy/queued) */
                uint32_t guard = 1000000;
                while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
                __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
                if (HAL_FLASHEx_Erase(&erase, &SectorError) != HAL_OK) {
                    printf("[OTA] Bank2 erase retry %d (sector %lu) SR2=0x%08lX\r\n",
                           attempt, (unsigned long)s, (unsigned long)FLASH->SR2);
                    /* Wait longer (BSY/QW residual) */
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
            /* Wait for current erase to complete (BSY/QW clear) */
            uint32_t guard = 1000000;
            while ((FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) && guard--) { }
            printf("[OTA] Erase Bank2 sector %lu/7 OK\r\n", (unsigned long)s);
        }
    }
    printf("[OTA] Bank2 erase done\r\n");

    /* Read back from QSPI staging -> Write to Bank2 (FLASHWORD 32B, 32B aligned) */
    {
        /* FLASHWORD DataAddress requires 32B alignment; stack variables not guaranteed -> static aligned buffer */
        static uint8_t wbuf[32] __attribute__((aligned(32)));
        uint32_t addr = APP_ADDR;
        uint32_t remain = ota_received;   /* only write received length */
        printf("[OTA] Programming %lu bytes...\r\n", (unsigned long)remain);
        while (remain > 0) {
            uint32_t chunk = (remain >= sizeof(wbuf)) ? sizeof(wbuf) : remain;
            memset(wbuf, 0xFF, sizeof(wbuf));
            if (BSP_QSPI_Read(wbuf, OTA_STAGING_ADDR + (addr - APP_ADDR), chunk) != QSPI_OK) {
                printf("[OTA] QSPI read FAIL @0x%08lX\r\n", (unsigned long)addr);
                HAL_FLASH_Lock();
                return 1;
            }
            /* Clear flag before write; retry up to 3 times on failure */
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

    /* read back to verify Bank2 header */
    memcpy(tmp, (void *)APP_ADDR, 4);
    printf("[OTA] Bank2 header: 0x%02X%02X%02X%02X\r\n",
           tmp[0], tmp[1], tmp[2], tmp[3]);

    /* ---- Step 3: Metadata PENDING_UPDATE ---- */
    printf("[OTA] Step 3: Metadata PENDING_UPDATE...\r\n");
    {
        Metadata meta;
        Metadata_Read(&meta);
        /* Fill new version numbers (from version server RESP) into ota + cur */
        uint32_t vm, vn, vp;
        Version_Check_GetVersionNumbers(&vm, &vn, &vp);
        meta.ota_major = meta.cur_major = vm;
        meta.ota_minor = meta.cur_minor = vn;
        meta.ota_patch = meta.cur_patch = vp;
        /* New CRC / size (corresponds to PENDING) */
        meta.crc32   = ota_crc_expected;
        meta.fw_size = ota_received;
        meta.state = STATE_PENDING_UPDATE;
        if (Metadata_Write(&meta) != 0) {
            printf("[OTA] Metadata write failed, OTA aborted (Bank2 written but state not updated)\r\n");
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

/* ---- for Firmware_Update to set expected CRC -- */
void OTA_SetExpectedCRC(uint32_t crc) { ota_crc_expected = crc; }