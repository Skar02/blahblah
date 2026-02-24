#ifndef SHARED_PROTOCOL_H
#define SHARED_PROTOCOL_H

#include <stdint.h>

typedef struct
{
    int32_t velocity;
    uint32_t timestamp;
} Sample_t;

#endif