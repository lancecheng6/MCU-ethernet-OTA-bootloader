/**
  ******************************************************************************
  * @file    bsp_qspi_flash.c
  * @brief   W25Q256JV QSPI NOR Flash driver
  *          Wildfire STM32H743 Pro onboard external NOR Flash
  *
  * Interface: QUADSPI (QSPI)
  * Mode: SPI mode (not QPI), 4-byte address mode
  * Memory-mapped base: 0x90000000
  ******************************************************************************
  */

#include "bsp_qspi_flash.h"
#include <string.h>
#include <stdio.h>

/* QSPI Handle -------------------------------------------------------------- */
QSPI_HandleTypeDef QSPIHandle;

/* W25Q256JV command codes --------------------------------------------------- */
#define W25Q_CMD_WRITE_ENABLE         0x06
#define W25Q_CMD_WRITE_DISABLE        0x04
#define W25Q_CMD_READ_STATUS_REG1     0x05
#define W25Q_CMD_READ_STATUS_REG2     0x35
#define W25Q_CMD_WRITE_STATUS_REG     0x01
#define W25Q_CMD_READ_DATA            0x03    /* 3-byte addr */
#define W25Q_CMD_READ_DATA_4B         0x13    /* 4-byte addr */
#define W25Q_CMD_FAST_READ            0x0B
#define W25Q_CMD_FAST_READ_4B         0x0C
#define W25Q_CMD_QUAD_READ            0xEB    /* 3-byte addr */
#define W25Q_CMD_QUAD_READ_4B         0xEC    /* 4-byte addr */
#define W25Q_CMD_PAGE_PROGRAM         0x02    /* 3-byte addr */
#define W25Q_CMD_PAGE_PROGRAM_4B      0x12    /* 4-byte addr */
#define W25Q_CMD_QUAD_PAGE_PROGRAM    0x32
#define W25Q_CMD_QUAD_PAGE_PROGRAM_4B 0x34
#define W25Q_CMD_SECTOR_ERASE_4KB     0x20    /* 3-byte addr */
#define W25Q_CMD_SECTOR_ERASE_4KB_4B  0x21    /* 4-byte addr */
#define W25Q_CMD_BLOCK_ERASE_32KB     0x52
#define W25Q_CMD_BLOCK_ERASE_32KB_4B  0x5C
#define W25Q_CMD_BLOCK_ERASE_64KB     0xD8
#define W25Q_CMD_BLOCK_ERASE_64KB_4B  0xDC
#define W25Q_CMD_CHIP_ERASE           0xC7
#define W25Q_CMD_ERASE_SUSPEND        0x75
#define W25Q_CMD_ERASE_RESUME         0x7A
#define W25Q_CMD_POWER_DOWN           0xB9
#define W25Q_CMD_RELEASE_PD           0xAB
#define W25Q_CMD_RESET_ENABLE         0x66
#define W25Q_CMD_RESET_MEMORY         0x99
#define W25Q_CMD_READ_JEDEC_ID        0x9F
#define W25Q_CMD_ENTER_4B_ADDR        0xB7
#define W25Q_CMD_EXIT_4B_ADDR         0xE9

/* Status Register bits ----------------------------------------------------- */
#define W25Q_SR1_WIP                 0x01    /* Write In Progress */
#define W25Q_SR1_WEL                 0x02    /* Write Enable Latch */

/* Private function prototypes -----------------------------------------------*/
static uint8_t QSPI_WriteEnable(void);
static uint8_t QSPI_AutoPollingMemReady(void);
static uint8_t QSPI_EnterFourBytesAddress(void);

/**
  * @brief  QSPI initialization — W25Q256JV
  *         1. QSPI peripheral initialization
  *         2. Reset flash
  *         3. Enter 4-byte address mode
  *         4. Enter memory-mapped mode
  * @retval QSPI_OK or QSPI_ERROR
  */
uint8_t BSP_QSPI_Init(void)
{
    /* QSPI peripheral initialization */
    QSPIHandle.Instance = QUADSPI;

    if (HAL_QSPI_DeInit(&QSPIHandle) != HAL_OK)
        return QSPI_ERROR;

    /* QSPI MSP initialization (GPIO + Clock) */
    /* Clock: AHB3, user must ensure __HAL_RCC_QSPI_CLK_ENABLE() has been called */
    __HAL_RCC_QSPI_CLK_ENABLE();

    /* QSPI initialization parameters
     * Clock: HCLK(200MHz) / (Prescaler+1) = 200MHz / 8 = 25MHz (reduced for testing, Prescaler=7)
     * FlashSize: 32MB = 2^25, so FlashSize = 25 - 1 = 24
     */
    QSPIHandle.Init.ClockPrescaler     = 7;    /* /8 -> 25MHz QSPI clock (reduced for testing) */
    QSPIHandle.Init.FifoThreshold      = 1;
    QSPIHandle.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    QSPIHandle.Init.FlashSize          = 24;   /* 32MB = 2^(24+1) */
    QSPIHandle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
    QSPIHandle.Init.ClockMode          = QSPI_CLOCK_MODE_0;
    QSPIHandle.Init.FlashID            = QSPI_FLASH_ID_1;
    QSPIHandle.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;

    if (HAL_QSPI_Init(&QSPIHandle) != HAL_OK)
        return QSPI_ERROR;

    /* GPIO QSPI pin initialization (CLK=PB2, NCS=PG6, D0=PF8, D1=PF9, D2=PF7, D3=PF6) */
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();

        /* CLK (PB2) — AF9 */
        GPIO_InitStruct.Pin       = GPIO_PIN_2;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* D0 (PF8), D1 (PF9) — AF10 */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* D2 (PF7), D3 (PF6) — AF9 */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* NCS (PG6) — AF10, Pull-up */
        GPIO_InitStruct.Pin       = GPIO_PIN_6;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    }

    /* Reset flash */
    {
        QSPI_CommandTypeDef s_command = {0};

        s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        s_command.AddressMode       = QSPI_ADDRESS_NONE;
        s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        s_command.DataMode          = QSPI_DATA_NONE;
        s_command.DummyCycles       = 0;
        s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
        s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

        /* Reset Enable */
        s_command.Instruction = W25Q_CMD_RESET_ENABLE;
        if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
            return QSPI_ERROR;

        /* Reset Memory */
        s_command.Instruction = W25Q_CMD_RESET_MEMORY;
        if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
            return QSPI_ERROR;
    }

    /* Wait for reset to complete */
    if (QSPI_AutoPollingMemReady() != QSPI_OK)
        return QSPI_ERROR;

    /* Enter 4-byte address mode */
    if (QSPI_EnterFourBytesAddress() != QSPI_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}

/**
  * @brief  Read data (memory-mapped mode, direct read)
  */
uint8_t BSP_QSPI_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
    QSPI_CommandTypeDef s_command = {0};

    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = W25Q_CMD_READ_DATA_4B;
    s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
    s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
    s_command.Address           = ReadAddr;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_1_LINE;
    s_command.DummyCycles       = 0;
    s_command.NbData            = Size;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return QSPI_ERROR;

    if (HAL_QSPI_Receive(&QSPIHandle, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}

/**
  * @brief  Write data (auto-handles page boundary, 256 bytes/page)
  *         Exits memory-mapped mode before write, re-enters after
  */
uint8_t BSP_QSPI_Write(uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
    uint32_t remaining = Size;
    uint32_t offset = 0;

    while (remaining > 0)
    {
        /* Calculate writable length for this page (page boundary 256 bytes) */
        uint32_t page_avail = QSPI_PAGE_SIZE - (WriteAddr % QSPI_PAGE_SIZE);
        uint32_t chunk = (remaining < page_avail) ? remaining : page_avail;

        /* Write Enable */
        if (QSPI_WriteEnable() != QSPI_OK)
            return QSPI_ERROR;

        /* Page Program (4-byte address) */
        {
            QSPI_CommandTypeDef s_command = {0};

            s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
            s_command.Instruction       = W25Q_CMD_PAGE_PROGRAM_4B;
            s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
            s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
            s_command.Address           = WriteAddr;
            s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
            s_command.DataMode          = QSPI_DATA_1_LINE;
            s_command.DummyCycles       = 0;
            s_command.NbData            = chunk;
            s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
            s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
            s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

            if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
                return QSPI_ERROR;

            if (HAL_QSPI_Transmit(&QSPIHandle, pData + offset, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
                return QSPI_ERROR;
        }

        /* Wait for write to complete */
        if (QSPI_AutoPollingMemReady() != QSPI_OK)
            return QSPI_ERROR;

        WriteAddr += chunk;
        offset    += chunk;
        remaining -= chunk;
    }

    return QSPI_OK;
}

/**
  * @brief  Erase one sector (4KB)
  * @param  SectorAddress: sector start address (must be 4KB aligned)
  */
uint8_t BSP_QSPI_Erase_Sector(uint32_t SectorAddress)
{
    if (QSPI_WriteEnable() != QSPI_OK)
        return QSPI_ERROR;

    {
        QSPI_CommandTypeDef s_command = {0};

        s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        s_command.Instruction       = W25Q_CMD_SECTOR_ERASE_4KB_4B;
        s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
        s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
        s_command.Address           = SectorAddress;
        s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        s_command.DataMode          = QSPI_DATA_NONE;
        s_command.DummyCycles       = 0;
        s_command.NbData            = 0;
        s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
        s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

        if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
            return QSPI_ERROR;
    }

    /* Wait for erase to complete (sector erase max 400ms) */
    if (QSPI_AutoPollingMemReady() != QSPI_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}

/**
  * @brief  Erase one 64KB block
  * @param  BlockAddress: block start address (must be 64KB aligned)
  */
uint8_t BSP_QSPI_Erase_Block64K(uint32_t BlockAddress)
{
    /* Retry 3 times: W25Q occasional erase failures. Key: must Abort + clear flags after each failure,
       otherwise QUADSPI controller gets stuck in Error/Busy state, causing subsequent retries to fail. */
    for (int attempt = 0; attempt < 3; attempt++)
    {
        if (QSPI_WriteEnable() != QSPI_OK) { goto retry; }

        QSPI_CommandTypeDef s_command = {0};

        s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        s_command.Instruction       = W25Q_CMD_BLOCK_ERASE_64KB_4B;
        s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
        s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
        s_command.Address           = BlockAddress;
        s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        s_command.DataMode          = QSPI_DATA_NONE;
        s_command.DummyCycles       = 0;
        s_command.NbData            = 0;
        s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
        s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

        if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) { goto retry; }

        /* Wait for erase to complete (64KB block erase max 2s) */
        if (QSPI_AutoPollingMemReady() != QSPI_OK) { goto retry; }

        return QSPI_OK;

retry:
        /* Before retry: force reset QSPI controller state and flags (otherwise timeout causes all subsequent retries to fail) */
        HAL_QSPI_Abort(&QSPIHandle);   /* Abort current operation, clear BUSY */
        __HAL_QSPI_CLEAR_FLAG(&QSPIHandle, QSPI_FLAG_TE | QSPI_FLAG_TC);  /* Clear error/complete flags */
        HAL_Delay(10);                 /* Allow flash internal state machine to stabilize */
    }

    return QSPI_ERROR;
}

/* Batch erase consecutive 64KB blocks.
   Experience: erasing too many (>8) consecutive 64KB blocks causes W25Q cumulative hang -> false erase failures.
   Strategy: re-init BSP_QSPI_Init() every 8 blocks to reset QSPI/flash; single failure also resets before retry.
   @param start   start address (must be 64KB aligned)
   @param size    total size (bytes, must be multiple of 64KB)
   @retval 0 = all success; non-zero = block failure */
int QSPI_Erase_Blocks(uint32_t start, uint32_t size)
{
    uint32_t cnt = 0;
    uint32_t addr;

    for (addr = start; addr < start + size; addr += QSPI_BLOCK_64K_SIZE) {
        if (BSP_QSPI_Erase_Block64K(addr) != QSPI_OK) {
            /* Failure: reset QSPI/flash then retry this block once */
            BSP_QSPI_Init();
            if (BSP_QSPI_Erase_Block64K(addr) != QSPI_OK) {
                printf("[QSPI] erase FAIL @0x%06lX\r\n", (unsigned long)addr);
                return 1;
            }
            printf("[QSPI] erase retry OK @0x%06lX\r\n", (unsigned long)addr);
        }
        if ((++cnt % 8) == 0) {
            /* Reset every 8 blocks = 512KB to avoid cumulative hang */
            BSP_QSPI_Init();
        }
    }
    return 0;
}

/**
  * @brief  Chip erase (max 100s)
  */
uint8_t BSP_QSPI_Erase_Chip(void)
{
    if (QSPI_WriteEnable() != QSPI_OK)
        return QSPI_ERROR;

    {
        QSPI_CommandTypeDef s_command = {0};

        s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        s_command.Instruction       = W25Q_CMD_CHIP_ERASE;
        s_command.AddressMode       = QSPI_ADDRESS_NONE;
        s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        s_command.DataMode          = QSPI_DATA_NONE;
        s_command.DummyCycles       = 0;
        s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
        s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

        if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
            return QSPI_ERROR;
    }

    if (QSPI_AutoPollingMemReady() != QSPI_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}

/**
  * @brief  Read JEDEC ID (3 bytes: Manufacturer + Memory Type + Capacity)
  */
uint32_t BSP_QSPI_ReadJedecID(void)
{
    uint32_t id = 0;
    uint8_t data[3];

    {
        QSPI_CommandTypeDef s_command = {0};

        s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        s_command.Instruction       = W25Q_CMD_READ_JEDEC_ID;
        s_command.AddressMode       = QSPI_ADDRESS_NONE;
        s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        s_command.DataMode          = QSPI_DATA_1_LINE;
        s_command.DummyCycles       = 0;
        s_command.NbData            = 3;
        s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
        s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

        if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
            return 0;

        if (HAL_QSPI_Receive(&QSPIHandle, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
            return 0;
    }

    id = (data[0] << 16) | (data[1] << 8) | data[2];

    return id;
}

/**
  * @brief  Enter memory-mapped mode
  *         Read access directly via 0x90000000 + offset
  */
uint8_t BSP_QSPI_EnterMemoryMapped(void)
{
    QSPI_CommandTypeDef      s_command = {0};
    QSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

    /* Quad Fast Read (4-byte address, 4-line) */
    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = W25Q_CMD_QUAD_READ_4B;  /* 0xEC */
    s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
    s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_4_LINES;
    s_command.DummyCycles       = 6;    /* W25Q256JV quad read needs 6 dummy cycles */
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
    s_mem_mapped_cfg.TimeOutPeriod     = 0;

    if (HAL_QSPI_MemoryMapped(&QSPIHandle, &s_command, &s_mem_mapped_cfg) != HAL_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}

/**
  * @brief  Exit memory-mapped mode (required before write/erase)
  */
uint8_t BSP_QSPI_ExitMemoryMapped(void)
{
    if (HAL_QSPI_Abort(&QSPIHandle) != HAL_OK)
        return QSPI_ERROR;
    return QSPI_OK;
}

/* ========================================================================= */
/*                             Private Functions                             */
/* ========================================================================= */

/**
  * @brief  Write Enable
  */
static uint8_t QSPI_WriteEnable(void)
{
    QSPI_CommandTypeDef s_command = {0};
    QSPI_AutoPollingTypeDef s_config = {0};

    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = W25Q_CMD_WRITE_ENABLE;
    s_command.AddressMode       = QSPI_ADDRESS_NONE;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_NONE;
    s_command.DummyCycles       = 0;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return QSPI_ERROR;

    /* Auto-polling wait for WEL=1 */
    s_config.Match           = W25Q_SR1_WEL;
    s_config.Mask            = W25Q_SR1_WEL;
    s_config.MatchMode       = QSPI_MATCH_MODE_AND;
    s_config.StatusBytesSize = 2;              /* Same as AutoPollingMemReady: SR 2 bytes */
    s_config.Interval        = 0x10;
    s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    s_command.Instruction    = W25Q_CMD_READ_STATUS_REG1;
    s_command.DataMode       = QSPI_DATA_1_LINE;

    if (HAL_QSPI_AutoPolling(&QSPIHandle, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}

/**
  * @brief  Auto-polling wait for flash ready (WIP=0)
  */
static uint8_t QSPI_AutoPollingMemReady(void)
{
    QSPI_CommandTypeDef s_command = {0};
    QSPI_AutoPollingTypeDef s_config = {0};

    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = W25Q_CMD_READ_STATUS_REG1;
    s_command.AddressMode       = QSPI_ADDRESS_NONE;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_1_LINE;
    s_command.DummyCycles       = 0;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    s_config.Match           = 0x0000;
    s_config.Mask            = W25Q_SR1_WIP;   /* Wait for WIP=0 (only compare WIP bit) */
    s_config.MatchMode       = QSPI_MATCH_MODE_AND;
    s_config.StatusBytesSize = 2;              /* W25Q SR is 2 bytes (SR1+SR2); set 2 to avoid FIFO alignment mismatch */
    s_config.Interval        = 0x10;
    s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_QSPI_AutoPolling(&QSPIHandle, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}

/**
  * @brief  Enter 4-byte address mode (required for W25Q256JV > 16MB)
  */
static uint8_t QSPI_EnterFourBytesAddress(void)
{
    QSPI_CommandTypeDef s_command = {0};

    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = W25Q_CMD_ENTER_4B_ADDR;
    s_command.AddressMode       = QSPI_ADDRESS_NONE;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_NONE;
    s_command.DummyCycles       = 0;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return QSPI_ERROR;

    return QSPI_OK;
}
