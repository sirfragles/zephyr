/*
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wheel_detector.h"

#include <math.h>
#include <string.h>

#define PI_F                         3.14159265358979323846f
#define FULL_ROTATION_RAD            (2.0f * PI_F)
#define NANOSECONDS_PER_SECOND       1000000000ULL
#define CENTER_FILTER_ALPHA          0.005f
#define MIN_ROTATING_VECTOR_MS2      2.0f
#define MAX_PHASE_STEP_RAD           1.6f
#define MIN_REVOLUTION_TIME_NS       70000000ULL
#define MAX_REVOLUTION_TIME_NS       5000000000ULL
#define DETECTOR_WARMUP_SAMPLE_COUNT 100U

void wheel_detector_init(struct wheel_detector *detector)
{
	memset(detector, 0, sizeof(*detector));
}

static float unwrap_phase_step(float step)
{
	if (step > PI_F) {
		step -= FULL_ROTATION_RAD;
	} else if (step < -PI_F) {
		step += FULL_ROTATION_RAD;
	}

	return step;
}

bool wheel_detector_process(struct wheel_detector *detector, float x_ms2, float y_ms2,
			    uint64_t timestamp_ns, struct wheel_revolution_event *event)
{
	float phase;
	float phase_step;
	float rotating_x;
	float rotating_y;
	float radius_squared;
	uint64_t revolution_time_ns;

	if (detector->warmup_samples == 0U) {
		detector->center_x = x_ms2;
		detector->center_y = y_ms2;
	}

	detector->center_x += CENTER_FILTER_ALPHA * (x_ms2 - detector->center_x);
	detector->center_y += CENTER_FILTER_ALPHA * (y_ms2 - detector->center_y);
	detector->warmup_samples++;

	rotating_x = x_ms2 - detector->center_x;
	rotating_y = y_ms2 - detector->center_y;
	radius_squared = rotating_x * rotating_x + rotating_y * rotating_y;

	if (detector->warmup_samples < DETECTOR_WARMUP_SAMPLE_COUNT ||
	    radius_squared < MIN_ROTATING_VECTOR_MS2 * MIN_ROTATING_VECTOR_MS2) {
		detector->phase_valid = false;
		detector->accumulated_phase = 0.0f;
		return false;
	}

	phase = atan2f(rotating_y, rotating_x);
	if (!detector->phase_valid) {
		detector->previous_phase = phase;
		detector->previous_timestamp_ns = timestamp_ns;
		detector->phase_valid = true;
		return false;
	}

	phase_step = unwrap_phase_step(phase - detector->previous_phase);
	detector->previous_phase = phase;
	detector->previous_timestamp_ns = timestamp_ns;

	/* A single large jump is an impact, not plausible wheel rotation. */
	if (fabsf(phase_step) > MAX_PHASE_STEP_RAD) {
		detector->phase_valid = false;
		detector->accumulated_phase = 0.0f;
		return false;
	}

	/* Do not let vibration in the opposite direction erase a nearly complete turn. */
	if (detector->accumulated_phase != 0.0f &&
	    ((detector->accumulated_phase > 0.0f) != (phase_step > 0.0f)) &&
	    fabsf(phase_step) > 0.35f) {
		detector->accumulated_phase = 0.0f;
	}
	detector->accumulated_phase += phase_step;

	if (fabsf(detector->accumulated_phase) < FULL_ROTATION_RAD) {
		return false;
	}

	revolution_time_ns = detector->last_revolution_ns == 0U
				     ? 0U
				     : timestamp_ns - detector->last_revolution_ns;
	detector->accumulated_phase = fmodf(detector->accumulated_phase, FULL_ROTATION_RAD);

	if (detector->last_revolution_ns != 0U && (revolution_time_ns < MIN_REVOLUTION_TIME_NS ||
						   revolution_time_ns > MAX_REVOLUTION_TIME_NS)) {
		detector->last_revolution_ns = timestamp_ns;
		return false;
	}

	detector->last_revolution_ns = timestamp_ns;
	detector->cumulative_revolutions++;
	event->cumulative_revolutions = detector->cumulative_revolutions;
	event->timestamp_ns = timestamp_ns;
	event->last_event_time_1024 =
		(uint16_t)((timestamp_ns / NANOSECONDS_PER_SECOND % 64U) * 1024U +
			   (timestamp_ns % NANOSECONDS_PER_SECOND) * 1024U /
				   NANOSECONDS_PER_SECOND);

	return true;
}
