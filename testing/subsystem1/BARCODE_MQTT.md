Barcode result events
=====================

Subscribe to `<MQTT_CLIENT_ID>/barcode` using the configured broker. The
topic can be overridden with `MQTT_BARCODE_TOPIC`. Payloads are JSON:

```json
{"frame":1,"status":"valid","letter":"Z","runs":29}
```

```json
{"frame":2,"status":"invalid","letter":null,"runs":23}
```

The sensor task enqueues an event when capture emits BARCODE or INVALID,
including timeout/rejected partial captures. Merely seeing black does not
publish a result. No result is emitted for a capture discarded by pause,
reset, timing fault, or junction classification before decode completes.

The radio task publishes with QoS 1 and retain=false. It keeps the pending
event until a successful acknowledgement; failed publish attempts retry.
QoS 1 can duplicate messages; frame IDs are per boot, not globally unique.

The dedicated queue holds four waiting events plus the radio's pending event.
Events wait for connectivity; a full queue rejects new events and logs a
negative `[BARCODE MQTT] enqueue` result. Queued events are lost on reboot.
`enqueue=0` means queued, not delivered. The broker acknowledgement has its
own log entry; it does not confirm that a subscriber processed the message.

Host queue test (build flags live in `testing/run_host_tests.sh`):

```sh
sh testing/run_host_tests.sh s1/barcode_queue
```

Broker delivery still requires a connected hardware test.
