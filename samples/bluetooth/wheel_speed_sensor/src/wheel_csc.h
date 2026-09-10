/*
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEEL_CSC_H_
#define WHEEL_CSC_H_

#include <stdint.h>

int wheel_csc_start(void);
void wheel_csc_publish(uint32_t cumulative_revolutions, uint16_t last_event_time_1024);

#endif /* WHEEL_CSC_H_ */
