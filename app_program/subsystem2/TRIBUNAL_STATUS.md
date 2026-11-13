# S2 tribunal changes and remaining work

> **Historical record** of the changes made for PreTri 1 (Week 8). For the
> current state see `STATUS.md` and `DEVIATIONS.md`. Note: this file says tilt
> is informational, but the integrated Vehicle Controller stops on tilt;
> which is intended is decision #67.

Only subsystem2 was edited. S5 avoidance is unchanged. Calibration, PWM and
PID constants are unchanged; old calibration is not validated for IRQ counts.

## Compact interface v1

motion_protocol.h uses explicit little-endian bytes, not compiler padding:
- Command: 5 bytes, type in 3 bits, 5 reserved zero bits, binary32 value.
- Status: 30 bytes; state (3), command (3), fault (2), complete (1) occupy
  2 bytes. Remaining bytes retain encoder int32 and numeric binary32 precision.

Numeric widths remain provisional until maximum range/resolution is measured.
Internal structs/floats remain for control calculations and diagnostics.
The queue now carries S2_PACKED_COMMAND. Use subsystem2_send_command rather
than writing raw structs to mbf_motion_cmd. Existing full status API remains;
subsystem2_get_packed_status supplies the compact operational alternative.
Other subsystem consumers have not been migrated.

STOP bypasses queue capacity through a flag and discards pending requests.
Other queued commands are FIFO, one consumed per service iteration. BUSY is
an explicit rejection, not an automatic retry. Consumption result is logged
and exposed through subsystem2_get_command_result with a wrapping sequence.
This is a latest-result snapshot, not guaranteed per-request acknowledgement
or completion history. Autonomous profiles retain external-command rejection.
Malformed queue sizes/codes/reserved bits/numeric values stop and drain the
queue. Invalid public submissions in queue-enabled mode set the STOP flag.

## Ownership, safety and timing

Integrated mode runs VC in motion_task at priority 14 on core 1. all_test
is an alternate motor owner. ISR counts edges without per-edge task wakeups.
Tilt and normal hump are informational in all_test and integrated modes.
S4 continues detecting/logging tilt. Collision, faults and stale IMU data
retain their existing stop handling.
ALL_TEST_CONTINUE_ON_SENSOR remains an explicit diagnostic safety bypass.

motion_task reports cumulative late-control counts and maximum control/service
work durations once per second, relative to the development 10 ms period. Millisecond diagnostics and logging
are not WCET proof or end-to-end stop latency. Use trace/GPIO on hardware.

## Deferred pending measurements/owner agreement

- Re-measure distance and both pivots before relying on integrated motion.
  S2_CALIBRATION_READY retains its previous value; it is not new evidence.
- Tune PID (currently zero), motor floors, tolerances and response deadlines.
- Derive numeric/encoder widths; signed counter overflow and coherent cross-
  core reset/read snapshots still need an explicit policy and verification.
- Agree request IDs and completion retention before changing that interface.
- S4 packed-output migration and shared TEAM.md corrections are outside scope.
- Record MQTT-under-motion timing, collision stop latency and tilt event logging, simultaneous
  events, all navigation actions and repeated physical distance/turn tests.

## Verification

motion_protocol_host_test.c is enabled only with S2_PROTOCOL_HOST_TEST=1.
It tests byte layout, reserved codes, finite values, bounds and precision.
subsystem2_test.c checks the packed queue size. Physical tests remain pending.
