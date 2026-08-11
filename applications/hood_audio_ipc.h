#ifndef HOOD_AUDIO_IPC_H
#define HOOD_AUDIO_IPC_H

#include <rtthread.h>
#include "drv_ipc.h"

#define HOOD_AUDIO_PROTO        (0x4841U)
#define HOOD_AUDIO_VERSION      (1U)

typedef enum
{
    HOOD_AUDIO_CMD_NONE = 0,
    HOOD_AUDIO_CMD_LOW_EXHAUST = 1,
    HOOD_AUDIO_CMD_HIGH_EXHAUST = 2,
    HOOD_AUDIO_CMD_GAS_WARN = 3,
    HOOD_AUDIO_CMD_GAS_DANGER = 4,
    HOOD_AUDIO_CMD_GAS_CLEAR = 5,
    HOOD_AUDIO_CMD_FAN_STOP = 6,
    HOOD_AUDIO_CMD_MANUAL_BLOCKED = 7,
    HOOD_AUDIO_CMD_MUTE = 8,
    HOOD_AUDIO_CMD_UNMUTE = 9,
} hood_audio_cmd_t;

static inline void hood_audio_frame_prepare(edge_rc_frame_t *frame,
                                            hood_audio_cmd_t command,
                                            rt_uint32_t seq,
                                            rt_uint16_t value)
{
    rt_memset(frame, 0, sizeof(*frame));
    frame->role = RC_ROLE_M55_ECHO;
    frame->magic = RC_MAGIC_WORD;
    frame->seq = seq;
    frame->channel[0] = HOOD_AUDIO_PROTO;
    frame->channel[1] = HOOD_AUDIO_VERSION;
    frame->channel[2] = (rt_uint16_t)command;
    frame->channel[3] = value;
    frame->checksum = edge_rc_checksum(frame);
}

static inline rt_bool_t hood_audio_frame_is_valid(const edge_rc_frame_t *frame)
{
    return frame->magic == RC_MAGIC_WORD &&
           frame->channel[0] == HOOD_AUDIO_PROTO &&
           frame->channel[1] == HOOD_AUDIO_VERSION &&
           edge_rc_checksum(frame) == frame->checksum;
}

#endif
