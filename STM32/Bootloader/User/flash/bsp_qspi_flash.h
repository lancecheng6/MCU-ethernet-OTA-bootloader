#ifndef __BSP_QSPI_FLASH_H
#define __BSP_QSPI_FLASH_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* W25Q256JV Specifications --------------------------------------------------- */
#define QSPI_FLASH_SIZE          0x2000000   /* 32MB = 256Mbit */
#define QSPI_PAGE_SIZE           256         /* 256 bytes per page */
#define QSPI_SECTOR_SIZE         4096        /* 4KB per sector */
#define QSPI_BLOCK_32K_SIZE      32768       /* 32KB block */
#define QSPI_BLOCK_64K_SIZE      65536       /* 64KB block */

/* Memory-mapped base address (D2 QSPI memory-mapped) ------------------------ */
#define QSPI_FLASH_BASE_ADDR     0x90000000

/* OTA area offsets (design document) ----------------------------------------- */
#define OTA_STAGING_OFFSET       0x000000    /* OTA staging area 2MB */
#define OTA_BACKUP_OFFSET        0x200000    /* Backup area 2MB */

/* Return values ------------------------------------------------------------- */
#define QSPI_OK                  0
#define QSPI_ERROR               1

/* Function prototypes ------------------------------------------------------- */
uint8_t  BSP_QSPI_Init(void);
uint8_t  BSP_QSPI_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
uint8_t  BSP_QSPI_Write(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
uint8_t  BSP_QSPI_Erase_Sector(uint32_t SectorAddress);
uint8_t  BSP_QSPI_Erase_Block64K(uint32_t BlockAddress);
uint8_t  BSP_QSPI_Erase_Chip(void);
int      QSPI_Erase_Blocks(uint32_t start, uint32_t size);   /* Erase 64KB blocks in batches (reset QSPI every 8) */
uint32_t BSP_QSPI_ReadJedecID(void);
uint8_t  BSP_QSPI_EnterMemoryMapped(void);
uint8_t  BSP_QSPI_ExitMemoryMapped(void);

/* External QSPI Handle (for other modules to access) ----------------------- */
extern QSPI_HandleTypeDef QSPIHandle;

#endif /* __BSP_QSPI_FLASH_H */
