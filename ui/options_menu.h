/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "menu_manager.h"

// Core options, read from the core's MiSTer-style config string (iosys 0x02) and sent
// back as core_config bits (iosys 0x03). Values are saved per core on the SD card.
Menu *create_options_menu(int core_id);      // core_id <= 0: no game core loaded

// Send the options saved for this core (0 if none). Call whenever a core has just
// started: a freshly configured bitstream always comes up with core_config = 0.
void apply_saved_core_options(int core_id);
