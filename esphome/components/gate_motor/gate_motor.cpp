// Roger Technology Brushless gate motor — ESPHome cover component
// Modbus RTU over TTL serial, protocol reverse-engineered from B74/BConnect captures.
#include "gate_motor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace gate_motor {

static const char *const TAG = "gate_motor";

// ─── ESPHome lifecycle ────────────────────────────────────────────────────────

void GateMotorCover::setup() {
  ESP_LOGI(TAG, "Gate motor cover initialised (Modbus addr=0x%02X)", MODBUS_ADDR);
}

void GateMotorCover::loop() {
  uint32_t now = millis();

  // Timeout guard: if command echo never arrived, give up
  if (pending_command_ && (now - command_sent_ms_ > COMMAND_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "Command 0x%04X timed out", pending_cmd_value_);
    pending_command_ = false;
  }

  if (now - last_poll_ms_ < poll_interval_ms_)
    return;
  last_poll_ms_ = now;

  // Send a queued command instead of polling this cycle
  if (pending_command_) {
    apply_pending_command();
    return;
  }

  poll_status();
}

void GateMotorCover::dump_config() {
  LOG_COVER("", "Gate Motor Cover", this);
  ESP_LOGCONFIG(TAG, "  Modbus address : 0x%02X", MODBUS_ADDR);
  ESP_LOGCONFIG(TAG, "  Poll interval  : %u ms", poll_interval_ms_);
}

// ─── Cover interface ──────────────────────────────────────────────────────────

cover::CoverTraits GateMotorCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void GateMotorCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    ESP_LOGI(TAG, "HA → STOP");
    pending_cmd_value_ = CMD_STOP;
    pending_command_   = true;
    command_sent_ms_   = millis();
    return;
  }

  if (call.get_position().has_value()) {
    float target = *call.get_position();
    if (target >= 1.0f) {
      ESP_LOGI(TAG, "HA → OPEN");
      pending_cmd_value_ = CMD_OPEN;
    } else if (target <= 0.0f) {
      ESP_LOGI(TAG, "HA → CLOSE (to pedestrian limit)");
      pending_cmd_value_ = CMD_CLOSE;
    } else {
      // Any partial position maps to pedestrian (the only supported intermediate stop)
      ESP_LOGI(TAG, "HA → PEDESTRIAN (partial open, target=%.0f%%)", target * 100);
      pending_cmd_value_ = CMD_PEDESTRIAN;
    }
    pending_command_ = true;
    command_sent_ms_ = millis();
  }
}

// ─── Polling ─────────────────────────────────────────────────────────────────

void GateMotorCover::poll_status() {
  send_read_holding(REG_STATUS_BASE, REG_STATUS_COUNT);

  uint8_t len = 0;
  if (!receive_frame(rx_buf_, &len)) {
    ESP_LOGW(TAG, "No response to status poll");
    return;
  }

  // Expected: 0A 03 0C [12 bytes] [CRC CRC] = 17 bytes
  if (len < 17 || rx_buf_[0] != MODBUS_ADDR || rx_buf_[1] != 0x03 || rx_buf_[2] != 0x0C) {
    ESP_LOGW(TAG, "Unexpected status response (len=%u fc=0x%02X bc=0x%02X)",
             len, rx_buf_[1], rx_buf_[2]);
    return;
  }
  if (!validate_crc(rx_buf_, len)) {
    ESP_LOGW(TAG, "CRC error in status response");
    return;
  }

  process_status_response(&rx_buf_[3]);
}

void GateMotorCover::apply_pending_command() {
  ESP_LOGI(TAG, "Sending command 0x%04X", pending_cmd_value_);
  send_write_single(REG_COMMAND, pending_cmd_value_);

  uint8_t len = 0;
  if (!receive_frame(rx_buf_, &len)) {
    ESP_LOGW(TAG, "No echo for command 0x%04X", pending_cmd_value_);
    pending_command_ = false;
    return;
  }

  // FC06 echo: 0A 06 [reg hi lo] [val hi lo] [CRC CRC] = 8 bytes
  if (len < 8 || rx_buf_[0] != MODBUS_ADDR || rx_buf_[1] != 0x06) {
    ESP_LOGW(TAG, "Unexpected command echo");
    pending_command_ = false;
    return;
  }
  if (!validate_crc(rx_buf_, len)) {
    ESP_LOGW(TAG, "CRC error in command echo");
    pending_command_ = false;
    return;
  }

  uint16_t echoed = ((uint16_t)rx_buf_[4] << 8) | rx_buf_[5];
  if (echoed == pending_cmd_value_) {
    ESP_LOGD(TAG, "Command 0x%04X confirmed by motor", pending_cmd_value_);
  } else {
    ESP_LOGW(TAG, "Command echo mismatch: sent 0x%04X got 0x%04X",
             pending_cmd_value_, echoed);
  }
  pending_command_ = false;
}

// ─── Response processing ──────────────────────────────────────────────────────

void GateMotorCover::process_status_response(const uint8_t *payload) {
  // payload[0..11] = 12 bytes for registers 0x157C–0x1581
  //
  // payload[6-7] = reg 0x157F: transition flag (0x0001 briefly after a command)
  // payload[8]   = reg 0x1580 high byte: position counter (0x00=open, 0x0F=ped/closed)
  // payload[9]   = reg 0x1580 low byte:  direction/limit flags

  uint8_t pos_counter = payload[PAYLOAD_POSITION];
  uint8_t direction   = payload[PAYLOAD_DIRECTION];

  if (pos_counter == last_position_counter_ && direction == last_direction_)
    return;  // no change

  ESP_LOGD(TAG, "Status: counter=0x%02X dir=0x%02X (was counter=0x%02X dir=0x%02X)",
           pos_counter, direction, last_position_counter_, last_direction_);

  last_position_counter_ = pos_counter;
  last_direction_        = direction;

  update_cover_state(pos_counter, direction);
}

void GateMotorCover::update_cover_state(uint8_t pos_counter, uint8_t direction) {
  cover::CoverOperation op = cover::COVER_OPERATION_IDLE;
  float new_pos = this->position;

  new_pos = counter_to_position(pos_counter);

  switch (direction) {
    case DIR_OPENING:
      // Gate travelling toward open limit (counter decreasing)
      op      = cover::COVER_OPERATION_OPENING;
      break;

    case DIR_CLOSING:
      // Gate travelling toward pedestrian/close limit (counter increasing)
      op      = cover::COVER_OPERATION_CLOSING;
      break;

    case DIR_AT_LIMIT:
    case DIR_AT_PED:
    case DIR_STOPPED:
      // Stopped mid-travel after STOP command
      op      = cover::COVER_OPERATION_IDLE;
      break;

    default:
      ESP_LOGW(TAG, "Unknown direction byte 0x%02X", direction);
      return;
  }

  this->position          = new_pos;
  this->current_operation = op;
  this->publish_state();
}

// ─── Modbus frame I/O ─────────────────────────────────────────────────────────

void GateMotorCover::send_read_holding(uint16_t reg_addr, uint16_t count) {
  uint8_t frame[8];
  frame[0] = MODBUS_ADDR;
  frame[1] = 0x03;
  frame[2] = (reg_addr >> 8) & 0xFF;
  frame[3] = reg_addr & 0xFF;
  frame[4] = (count >> 8) & 0xFF;
  frame[5] = count & 0xFF;
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;
  this->write_array(frame, 8);
  this->flush();
}

void GateMotorCover::send_write_single(uint16_t reg_addr, uint16_t value) {
  uint8_t frame[8];
  frame[0] = MODBUS_ADDR;
  frame[1] = 0x06;
  frame[2] = (reg_addr >> 8) & 0xFF;
  frame[3] = reg_addr & 0xFF;
  frame[4] = (value >> 8) & 0xFF;
  frame[5] = value & 0xFF;
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;
  this->write_array(frame, 8);
  this->flush();
}

bool GateMotorCover::receive_frame(uint8_t *buf, uint8_t *len, uint32_t timeout_ms) {
  uint32_t deadline    = millis() + timeout_ms;
  uint8_t  idx         = 0;
  uint32_t last_byte_ms = millis();

  while (millis() < deadline) {
    if (this->available()) {
      buf[idx++] = this->read();
      last_byte_ms = millis();
      if (idx >= sizeof(rx_buf_) - 1)
        break;
    } else {
      if (idx > 0 && (millis() - last_byte_ms) > MODBUS_INTER_CHAR_MS)
        break;
      yield();
    }
  }

  *len = idx;
  return idx > 0;
}

// ─── CRC16 (Modbus) ───────────────────────────────────────────────────────────

uint16_t GateMotorCover::crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

bool GateMotorCover::validate_crc(const uint8_t *frame, uint8_t len) {
  if (len < 4)
    return false;
  uint16_t calc     = crc16(frame, len - 2);
  uint16_t received = ((uint16_t)frame[len - 1] << 8) | frame[len - 2];
  return calc == received;
}

// ─── Position conversion ──────────────────────────────────────────────────────

float GateMotorCover::counter_to_position(uint8_t counter) {
  // counter 0x00 = fully open  = 1.0
  // counter 0x0F = ped/closed  = 0.0
  // Linear interpolation between limits
  float pos = 1.0f - ((float)counter / (float)POS_PED_LIMIT);
  return clamp(pos, 0.0f, 1.0f);
}

}  // namespace gate_motor
}  // namespace esphome
