# ESPHome — Roger Technology Brushless Gate Motor

A custom ESPHome component that replaces the **Roger Technology B74/BConnect** WiFi
module, communicating directly with the gate's digital controller over the same TTL
serial interface. Presents a native `cover` entity (with position) to Home Assistant.

The protocol was reverse-engineered from serial traffic captures between a B74/BConnect
and a Roger Technology Brushless digital controller.

---

## Compatibility

**WiFi module replaced:** Roger Technology B74/BConnect

**Compatible digital controllers:**
- B70/1DC (from firmware P2.30+)
- B70/1DCHP
- B70/2ML
- EDGE1
- CTRL / CTRL/P

---

## File Structure

```
gate_motor/
├── gate_motor.yaml                    ← ESPHome config (edit this)
├── secrets.yaml                       ← Your credentials (create this)
└── components/
    └── gate_motor/
        ├── __init__.py                ← Component namespace declaration
        ├── cover.py                   ← ESPHome platform schema & codegen
        ├── gate_motor.h               ← C++ header
        └── gate_motor.cpp             ← C++ implementation
```

---

## Hardware

### Parts needed
- Wemos D1 Mini (ESP8266) **or** any ESP32 dev board
- Three dupont wires

### Wiring (Wemos D1 Mini)

The B74/BConnect connects to the digital controller via a 4-pin serial header on the
controller board. Wire the D1 Mini directly to that header:

```
Gate pin 1 (5V)  ──→ D1 Mini 5V      (powers the module)
Gate pin 2 (GND) ──→ D1 Mini GND
Gate pin 3 (TX)  ──→ D1 Mini RX (GPIO3)
Gate pin 4 (RX)  ──→ D1 Mini TX (GPIO1)
```

> **Voltage level:** The gate serial interface runs at 3.3V TTL — confirmed safe
> to connect directly to the D1 Mini without a level shifter.

> **UART note:** The D1 Mini has only one hardware UART. Setting `baud_rate: 0` in
> the logger config frees it for gate comms. Logs remain available over WiFi via
> the ESPHome dashboard or `esphome logs`.

### Wiring (ESP32 — alternative)

If using an ESP32, change the board to `esp32dev`, restore logger `baud_rate: 115200`,
and use `tx_pin: GPIO17` / `rx_pin: GPIO16` (UART2, leaves UART0 free for USB logging).

---

## Protocol Reference

All communication uses **Modbus RTU** at **115200 baud, 8N1** to device address
**`0x0A`** (10). The physical layer is 3.3V TTL serial — not RS485.

### Function codes used

| FC | Name | Direction |
|----|------|-----------|
| `0x03` | Read Holding Registers | ESP → controller |
| `0x06` | Write Single Register | ESP → controller |

---

### Register map

| Address | Name | Access | Description |
|---------|------|--------|-------------|
| `0x157C` | Status block base | FC03 read ×6 | Primary status — see detail below |
| `0x1711` | Limit state | FC03 read ×2 | Which limit is active — see detail below |
| `0x1965` | Command | FC06 write | Send gate commands — see detail below |

---

### Register `0x157C` — Status block (FC03, read 6 registers)

Reading 6 registers from `0x157C` returns 12 payload bytes covering registers
`0x157C`–`0x1581`. The two useful registers within this block are:

#### Register `0x157F` — Transition flag (bytes 6–7 of payload)

| Value | Meaning |
|-------|---------|
| `0x0001` | Command just received; clears within one poll cycle as motion begins |
| `0x0000` | Stable state |

#### Register `0x1580` — Gate status (bytes 8–9 of payload)

This register contains two independent bytes that together fully describe the
gate's state.

**High byte — Position counter**

This byte is a live position counter that tracks where the gate is in its travel.
It is not a state enum. The counter decrements as the gate opens and increments
as it closes.

| Value | Meaning |
|-------|---------|
| `0x00` | Gate at fully open limit |
| `0x08` | Gate at pedestrian open position (halfway) |
| `0x0F` | Gate at pedestrian / close limit |
| `0x01`–`0x0E` | Gate mid-travel; position = `1.0 - (counter / 0x0F)` |

The counter steps are not perfectly uniform (values like `0x04` and `0x06`
are skipped during fast travel) but are sufficiently regular for linear
interpolation to give a good position estimate.

**Low byte — Direction and limit flags**

| Value | Meaning |
|-------|---------|
| `0x01` | Travelling toward open (counter decreasing) |
| `0x02` | Stopped mid-travel (after STOP command) |
| `0x03` | Travelling toward close / pedestrian limit (counter increasing) |
| `0x05` | Stationary at open hard limit |
| `0x06` | Stationary at pedestrian / close limit |

The distinction between `0x05` (open limit) and `0x06` (pedestrian limit) is
reliable and unambiguous — no additional register is needed to determine which
end the gate is at.

**Complete state truth table**

| Position counter | Direction byte | HA cover state | Position % |
|-----------------|----------------|----------------|------------|
| `0x00` | `0x05` | Open | 100% |
| `0x01`–`0x0E` | `0x01` | Opening | `(1 - counter/15) × 100%` |
| `0x01`–`0x0E` | `0x03` | Closing | `(1 - counter/15) × 100%` |
| `0x01`–`0x0E` | `0x02` | Stopped | `(1 - counter/15) × 100%` |
| `0x08` | `0x05` | Open (pedestrian) | ~53% |
| `0x0F` | `0x06` | Closed | 0% |

---

### Register `0x1711` — Limit state flags (FC03, read 2 registers)

The first register (`r0`) encodes which limit sensor is currently active.
Bits 2 and 3 are always set as a fixed base value (`0x000C`).

| Value | Meaning |
|-------|---------|
| `0x004C` | At fully open limit (bit 6 set) |
| `0x008C` | At pedestrian / close limit (bit 7 set) |
| `0x000C` | In motion — neither limit active |

This register provides the same information as the direction byte in `0x1580`
and is used as a cross-check. The component primarily relies on `0x1580`.

---

### Register `0x1965` — Command (FC06 write single register)

Writing to this register sends a command to the gate. The high byte `0x68` is
a fixed magic prefix; the low byte is the command bitmask.

| Value | Command | Description |
|-------|---------|-------------|
| `0x6801` | STOP | Halt gate immediately |
| `0x6802` | OPEN | Open fully to open limit |
| `0x6804` | CLOSE | Close to pedestrian / close limit |
| `0x6810` | PEDESTRIAN | Open to pedestrian position (`0x08`, ~50%) |

The controller echoes the FC06 frame unchanged as acknowledgement. The
component waits for this echo before resuming normal polling.

---

### Observed polling sequence

The B74/BConnect polls in a round-robin pattern at approximately 200 ms per cycle.
Our component simplifies this to a single status register poll per cycle:

```
ESP  →  0A 03 15 7C 00 06 [CRC]    ← read 6 regs from 0x157C
CTRL →  0A 03 0C [12 bytes] [CRC]  ← 12-byte payload response
```

When a command is pending, one poll cycle is replaced by the command write:

```
ESP  →  0A 06 19 65 68 02 [CRC]    ← FC06 write OPEN to 0x1965
CTRL →  0A 06 19 65 68 02 [CRC]    ← echo (same frame)
```

---

## Setup

### 1. Create `secrets.yaml`

```yaml
wifi_ssid: "YourSSID"
wifi_password: "YourWiFiPassword"
api_encryption_key: "base64_key_here"   # generate with: esphome generate-key
ota_password: "somepassword"
```

### 2. Flash

No calibration is required — the position counter has fixed limits (`0x00`–`0x0F`)
that are the same on all compatible controllers.

```bash
esphome run gate_motor.yaml
```

---

## Home Assistant

The device appears as a **gate** entity with:
- Open / Close / Stop controls
- Position slider (0–100%)
- States: opening, closing, open, closed, stopped

```yaml
action: cover.open_cover
target:
  entity_id: cover.driveway_gate
```

The pedestrian open position (~50%) is accessible via the position slider or:

```yaml
action: cover.set_cover_position
target:
  entity_id: cover.driveway_gate
data:
  position: 50
```

---

## Troubleshooting

| Symptom | Check |
|---------|-------|
| No response from controller | TX/RX swapped — try reversing GPIO1/GPIO3 |
| CRC errors in logs | Baud rate mismatch or noisy signal |
| State never updates | Wrong UART GPIO pins in YAML |
| Gate moves but HA state wrong | Check ESPHome logs for direction byte values |
| Gate moves but HA state lags | Reduce `poll_interval` to `250ms` |
