# Gimbal Protocol

Select the wire format with `protocol` in `config/serial.yaml`:

```yaml
protocol: v2       # default
# protocol: legacy_a
```

Only one packet is sent for each RGB result.

## V2 packet

```text
V2,seq,age_us,yaw_cdeg,pitch_cdeg,dist_mm,depth_age_ms,flags,crc16\n
```

- `seq`: RGB frame sequence as unsigned 32-bit decimal; wraparound is allowed.
- `age_us`: RGB capture-to-packet age in microseconds, clamped to
  `0..500000`.
- `yaw_cdeg`, `pitch_cdeg`: relative angle errors in `0.01 deg`.
- `dist_mm`: laser-origin-to-target distance estimate.
- `depth_age_ms`: depth age relative to RGB, rounded and clamped to `0..1000`.
- `flags`:
  - bit 0: existing vision `valid`;
  - bit 1: RGB timestamp is reliable;
  - bit 2: this RGB frame received a new, non-reused depth frame;
  - bit 3: the existing depth estimate is valid;
  - bit 4: the depth frame was reused;
  - bits 5..31: zero.
- Invalid vision results retain the existing wire semantics: yaw, pitch and
  distance are zero, and flags bit 0 is clear.

`crc16` is CRC-16/CCITT-FALSE:

- polynomial `0x1021`;
- initial value `0xFFFF`;
- `refin=false`, `refout=false`;
- `xorout=0x0000`;
- input is the ASCII byte range beginning with `V2` and ending with the final
  decimal digit of `flags`;
- the comma after `flags`, CRC field and newline are excluded;
- output is four uppercase hexadecimal digits.

Example CRC input:

```text
V2,42,12345,125,-50,1000,5,15
```

Complete packet:

```text
V2,42,12345,125,-50,1000,5,15,4B91\n
```

## Legacy A packet

With `protocol: legacy_a`, the original packet remains byte-for-byte
compatible:

```text
A,yaw_cdeg,pitch_cdeg,dist_mm,valid\n
```

The STM32 side should compute:

```text
yaw_target = current_yaw + yaw_delta
pitch_target = current_pitch + pitch_delta
```

This replaces the old pixel PID outer loop.
