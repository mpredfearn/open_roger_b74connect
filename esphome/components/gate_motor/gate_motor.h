// ESPHome custom component for Roger Technology Brushless gate motor controllers.
// Replaces the Roger Technology B74/BConnect WiFi module, using the same
// Modbus RTU protocol over the TTL serial interface on the digital controller.
//
// Compatible controllers: B70/1DC, B70/1DCHP, B70/2ML, EDGE1, CTRL, CTRL/P
// Protocol reverse-engineered from B74/BConnect serial traffic captures.

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"

namespace esphome {
namespace gate_motor {

// ─── Modbus register map ──────────────────────────────────────────────────────
static const uint8_t  MODBUS_ADDR         = 0x0A;
static const uint16_t REG_STATUS_BASE     = 0x157C;  // FC03 read 6 regs → primary status block
static const uint16_t REG_STATUS_COUNT    = 6;
static const uint16_t REG_LIMIT_STATE     = 0x1711;  // FC03 read 2 regs → limit/state flags
static const uint16_t REG_COMMAND         = 0x1965;  // FC06 write → gate commands

// Command values written to REG_COMMAND (high byte 0x68 = magic prefix)
static const uint16_t CMD_STOP       = 0x6801;
static const uint16_t CMD_OPEN       = 0x6802;
static const uint16_t CMD_CLOSE      = 0x6804;
static const uint16_t CMD_PEDESTRIAN = 0x6810;

// ─── Status register 0x1580 decode ───────────────────────────────────────────
// The 6-register block at 0x157C is read as 12 payload bytes.
// Useful data is in register 0x1580 (bytes 8-9 of the payload):
//
//   HIGH BYTE (byte 8) = POSITION COUNTER
//     Counts the gate's position as encoder steps.
//     0x00 = fully open (at open limit)
//     0x0F = at pedestrian limit (this is also the "closed" end for this gate)
//     Values in between = proportional travel between limits
//     Counter DECREASES as gate opens, INCREASES as gate closes.
//
//   LOW BYTE (byte 9) = DIRECTION / LIMIT FLAGS
//     0x01 = travelling toward open (counter decreasing)
//     0x02 = stopped mid-travel (after STOP command)
//     0x03 = travelling toward close/pedestrian limit (counter increasing)
//     0x05 = stationary at open limit
//     0x06 = stationary at pedestrian limit
//
// Register 0x157F (bytes 6-7 of payload):
//   0x0001 = transition flag: command just received, lasts ~1 poll cycle
//   0x0000 = stable state
//
// Register 0x1711 high byte (r0 of 2-reg read):
//   0x004C = gate at fully open limit   (bit6 set)
//   0x008C = gate at pedestrian limit   (bit7 set)
//   0x000C = gate in motion / mid-travel (neither bit set)

static const uint8_t PAYLOAD_TRANSITION_HI = 6;  // byte index in 12-byte payload
static const uint8_t PAYLOAD_TRANSITION_LO = 7;
static const uint8_t PAYLOAD_POSITION      = 8;  // position counter (high byte of 0x1580)
static const uint8_t PAYLOAD_DIRECTION     = 9;  // direction/limit flags (low byte of 0x1580)

// Direction/limit flag values (PAYLOAD_DIRECTION)
static const uint8_t DIR_OPENING   = 0x01;  // counter decreasing, moving toward open
static const uint8_t DIR_STOPPED   = 0x02;  // stopped mid-travel
static const uint8_t DIR_CLOSING   = 0x03;  // counter increasing, moving toward close
static const uint8_t DIR_AT_LIMIT  = 0x05;  // at open hard limit (stationary)
static const uint8_t DIR_AT_PED    = 0x06;  // at pedestrian limit (stationary)

// Position counter limits
static const uint8_t POS_OPEN_LIMIT = 0x00;  // fully open
static const uint8_t POS_PED_LIMIT  = 0x0F;  // pedestrian / close limit

// Reg 0x1711 r0 limit flag bits
static const uint16_t LIMIT_FLAG_OPEN = 0x004C;  // bit6: at open limit
static const uint16_t LIMIT_FLAG_PED  = 0x008C;  // bit7: at pedestrian limit
static const uint16_t LIMIT_FLAG_MOVE = 0x000C;  // no limit bits: in motion

// ─── Timing ──────────────────────────────────────────────────────────────────
static const uint32_t POLL_INTERVAL_MS     = 500;
static const uint32_t COMMAND_TIMEOUT_MS   = 10000;
static const uint32_t MODBUS_INTER_CHAR_MS = 2;  // ~87µs/byte at 115200; 2ms > 3.5-char gap


class GateMotorCover : public cover::Cover, public uart::UARTDevice, public Component {
 public:
  // ── ESPHome lifecycle ─────────────────────────────────────────────────────
  void setup() override;
  void loop() override;
  void dump_config() override;

  // ── Cover interface ───────────────────────────────────────────────────────
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;

  // ── Configuration setters ─────────────────────────────────────────────────
  void set_poll_interval(uint32_t ms) { poll_interval_ms_ = ms; }

 protected:
  // ── Modbus I/O ────────────────────────────────────────────────────────────
  void send_read_holding(uint16_t reg_addr, uint16_t count);
  void send_write_single(uint16_t reg_addr, uint16_t value);
  bool receive_frame(uint8_t *buf, uint8_t *len, uint32_t timeout_ms = 100);

  // ── CRC ───────────────────────────────────────────────────────────────────
  static uint16_t crc16(const uint8_t *data, uint8_t len);
  static bool validate_crc(const uint8_t *frame, uint8_t len);

  // ── State machine ─────────────────────────────────────────────────────────
  void poll_status();
  void apply_pending_command();
  void process_status_response(const uint8_t *payload);
  void update_cover_state(uint8_t position_counter, uint8_t direction);
  float counter_to_position(uint8_t counter);

  // ── Members ───────────────────────────────────────────────────────────────
  uint32_t poll_interval_ms_{POLL_INTERVAL_MS};

  uint32_t last_poll_ms_{0};
  bool     pending_command_{false};
  uint16_t pending_cmd_value_{0};
  uint32_t command_sent_ms_{0};

  uint8_t  last_position_counter_{0xFF};
  uint8_t  last_direction_{0xFF};

  uint8_t  rx_buf_[64];
};

}  // namespace gate_motor
}  // namespace esphome
