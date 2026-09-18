#include <stdbool.h>

#include "fake_stm.h"
#include "can.h"

//int safety_tx_hook(CANPacket_t *to_send) { return 1; }

#include "faults.h"
#include "safety.h"
#include "drivers/can_common.h"

// libsafety stuff
#include "safety_helpers.h"

// fix: provide definition for extern bool referenced by hyundai_canfd buffered fwd
bool safety_tx_buffered_for_fwd = false;
