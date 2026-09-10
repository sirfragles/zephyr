/*
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEEL_DETECTOR_H_
#define WHEEL_DETECTOR_H_

#include <stdbool.h>
#include <stdint.h>

struct wheel_revolution_event {
	uint32_t cumulative_revolutions;
	uint64_t timestamp_ns;
	uint16_t last_event_time_1024;
};

struct wheel_detector {
	float center_x;
	float center_y;
	float previous_phase;
	float accumulated_phase;
	uint64_t previous_timestamp_ns;
	uint64_t last_revolution_ns;
	uint32_t cumulative_revolutions;
	uint32_t warmup_samples;
	bool phase_valid;
};

void wheel_detector_init(struct wheel_detector *detector);
bool wheel_detector_process(struct wheel_detector *detector, float x_ms2, float y_ms2,
			    uint64_t timestamp_ns, struct wheel_revolution_event *event);

#endif /* WHEEL_DETECTOR_H_ */
