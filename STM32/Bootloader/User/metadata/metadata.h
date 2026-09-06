/**
  * @file    metadata.h
  * @brief   OTA Metadata (shared between App and Bootloader)
  *          Stored in internal Flash 0x08020000 (sector 1)
  */
#ifndef __METADATA_H
#define __METADATA_H

#include <stdint.h>

#define METADATA_ADDR   0x08020000
#define METADATA_MAGIC  0x4D455441   /* "META" */

typedef enum {
    STATE_EMPTY          = 0xFFFFFFFF,  /* Flash uninitialized */
    STATE_PENDING_UPDATE = 0x50454E44,  /* "PEND" new firmware received, awaiting verification */
    STATE_TESTING        = 0x54455354,  /* "TEST" health check in progress */
    STATE_CONFIRMED      = 0x434F4E46   /* "CONF" App confirmed */
} MetadataState;

/* 64 bytes aligned Flash word (multiple of 32 bytes) */
typedef struct {
    uint32_t magic;          /* METADATA_MAGIC */
    uint32_t state;          /* MetadataState */
    uint32_t version_major;
    uint32_t version_minor;
    uint32_t version_patch;
    uint32_t crc32;          /* App firmware CRC32 */
    uint32_t fw_size;        /* App firmware size in bytes */
    uint32_t reserved[9];    /* Padding to 64 bytes */
} Metadata;

void Metadata_Read(Metadata *meta);
int  Metadata_Write(Metadata *meta);   /* 0=success, -1=failure (erase/program/verify all retry) */
void Metadata_ReadVersion(char *buf);          /* "vX.Y.Z" */
const char *Metadata_StateStr(uint32_t state);
int  Metadata_IsValid(Metadata *meta);
void Metadata_SetMagic(uint32_t magic);        /* For test override */

#endif /* __METADATA_H */