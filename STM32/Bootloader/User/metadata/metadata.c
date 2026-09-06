/**
  * @file    metadata.c
  * @brief   Metadata read/write (internal Flash 0x08020000, sector 1)
  *          H7 write uses FLASHWORD (32 bytes / Program FLASHWORD).
  */
#include "metadata.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

void Metadata_Read(Metadata *meta)
{
    uint8_t *p = (uint8_t *)METADATA_ADDR;
    memcpy(meta, p, sizeof(*meta));

    if (meta->magic != METADATA_MAGIC) {
        /* Flash uninitialized / invalid content -> assign defaults */
        memset(meta, 0, sizeof(*meta));
        meta->magic = METADATA_MAGIC;
        meta->state = STATE_EMPTY;
        meta->version_major = 0;
        meta->version_minor = 0;
        meta->version_patch = 0;
        meta->crc32 = 0;
        meta->fw_size = 0;
    }
}

int Metadata_Write(Metadata *meta)
{
    HAL_StatusTypeDef st;
    uint32_t sector_to_erase = 1;   /* sector 1 = 0x08020000 */
    FLASH_EraseInitTypeDef erase;
    uint32_t SectorError = 0;
    uint32_t addr;
    int ok;
    /* FLASHWORD requires 32B aligned data source; stack variables not guaranteed -> use static aligned buffer */
    static uint8_t aligned[sizeof(Metadata)] __attribute__((aligned(32)));

    /* Ensure magic is correct */
    meta->magic = METADATA_MAGIC;

    memcpy(aligned, meta, sizeof(Metadata));

    HAL_FLASH_Unlock();

    /* Erase sector 1 (retry up to 10 times; clear error flag + short delay each time) */
    erase.TypeErase   = FLASH_TYPEERASE_SECTORS;
    erase.Banks       = FLASH_BANK_1;
    erase.Sector      = sector_to_erase;
    erase.NbSectors   = 1;
    erase.VoltageRange = 0;         /* H7 no such field (compatibility) */
    ok = 0;
    for (int attempt = 0; attempt < 10 && !ok; attempt++) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
        SectorError = 0;
        st = HAL_FLASHEx_Erase(&erase, &SectorError);
        printf("[META] Erase[%d] st=%d SectorError=%lu\r\n",
               attempt, (int)st, (unsigned long)SectorError);
        if (st == HAL_OK) ok = 1;
        else {
            printf("[META] Erase retry SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)FLASH->SR1, (unsigned long)FLASH->SR2);
            for (volatile uint32_t g = 0; g < 100000; g++) { }   /* Short delay */
        }
    }
    if (!ok) {
        printf("[META] Erase FAILED (10x)\r\n");
        HAL_FLASH_Lock();
        return -1;
    }

    /* Write 64 bytes = 2 FLASHWORDs (each 32 bytes; retry up to 3 times on failure) */
    for (uint32_t i = 0; i < (sizeof(*meta) / 32); i++) {
        int prog_ok = 0;
        addr = METADATA_ADDR + i * 32;
        for (int attempt = 0; attempt < 3 && !prog_ok; attempt++) {
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
            st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   addr, (uint32_t)(aligned + i * 32));
            printf("[META] Program[%d] i=%lu st=%d @0x%08lX\r\n",
                   attempt, (unsigned long)i, (int)st, (unsigned long)addr);
            if (st == HAL_OK) prog_ok = 1;
            else {
                printf("[META] Program retry SR1=0x%08lX SR2=0x%08lX\r\n",
                       (unsigned long)FLASH->SR1, (unsigned long)FLASH->SR2);
                for (volatile uint32_t g = 0; g < 100000; g++) { }
            }
        }
        if (!prog_ok) {
            printf("[META] Program FAILED\r\n");
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock();

    /* Read back and verify (clear DCache first to avoid reading stale cached values) */
    SCB_InvalidateDCache();
    uint8_t *p = (uint8_t *)METADATA_ADDR;
    if (memcmp(p, meta, sizeof(*meta)) != 0) {
        printf("[META] Write verify FAILED\r\n");
        printf("[META] flash: magic=0x%08lX state=0x%08lX\r\n",
               (unsigned long)((uint32_t*)p)[0], (unsigned long)((uint32_t*)p)[1]);
        return -1;
    } else {
        printf("[META] Write verify OK\r\n");
    }
    return 0;
}

void Metadata_ReadVersion(char *buf)
{
    Metadata meta;
    Metadata_Read(&meta);
    sprintf(buf, "v%u.%u.%u",
            (unsigned)meta.version_major,
            (unsigned)meta.version_minor,
            (unsigned)meta.version_patch);
}

const char *Metadata_StateStr(uint32_t state)
{
    switch (state) {
    case STATE_PENDING_UPDATE: return "PENDING_UPDATE";
    case STATE_TESTING:        return "TESTING";
    case STATE_CONFIRMED:      return "CONFIRMED";
    default:                   return "EMPTY";
    }
}

int Metadata_IsValid(Metadata *meta)
{
    return (meta->magic == METADATA_MAGIC);
}

void Metadata_SetMagic(uint32_t magic) { (void)magic; }