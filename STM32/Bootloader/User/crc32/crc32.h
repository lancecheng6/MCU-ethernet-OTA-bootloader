/**
  * @file    crc32.h
  * @brief   Standard zlib CRC32 (table lookup method), compatible with Python zlib.crc32
  */
#ifndef __CRC32_H
#define __CRC32_H

#include <stdint.h>

uint32_t CRC32_Calculate(const void *data, uint32_t length);

#endif /* __CRC32_H */