#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

#include <stdint.h>
#include <sys/types.h>

#define __autoalign__ __attribute__((packed))

#pragma pack(push, 1)

typedef struct
{
    unsigned short Dist_1;
    unsigned char  RSSI_1;
    unsigned short Dist_2;
    unsigned char  RSSI_2;
} __autoalign__ MeasuringResult;

typedef struct
{
    unsigned short  DataFlag;
    unsigned short  Azimuth;
    MeasuringResult Result[16];
} __autoalign__ Data_block;

// Named type (original was anonymous global — fixed here for multi-include safety)
typedef struct
{
    Data_block    BlockID[12];
    unsigned int  Timestamp;
    unsigned short Factory;
} __autoalign__ MSOP_Packet;

#pragma pack(pop)

typedef struct bm_response_scan
{
    uint16_t     angle;
    uint16_t     dist;
    uint8_t      rssi;
    unsigned int timestamp;
} bm_response_scan_t;

#endif  // __DATA_TYPE_H__
