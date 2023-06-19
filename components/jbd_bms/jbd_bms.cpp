#include "jbd_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace jbd_bms {

static const char *const TAG = "jbd_bms";

static const uint8_t MAX_NO_RESPONSE_COUNT = 5;

static const uint8_t JBD_PKT_START = 0xDD;
static const uint8_t JBD_PKT_END = 0x77;
static const uint8_t JBD_CMD_READ = 0xA5;
static const uint8_t JBD_CMD_WRITE = 0x5A;

static const uint8_t JBD_CMD_HWINFO = 0x03;
static const uint8_t JBD_CMD_CELLINFO = 0x04;

static const uint8_t JBD_CMD_CAP_REM = 0xE0;   // Set remaining capacity
static const uint8_t JBD_CMD_MOS = 0xE1;       // Set charging/discharging bitmask
static const uint8_t JBD_CMD_BALANCER = 0xE2;  // Enable/disable balancer

static const uint8_t JBD_MOS_CHARGE = 0x01;
static const uint8_t JBD_MOS_DISCHARGE = 0x02;

static const uint8_t ERRORS_SIZE = 16;
static const char *const ERRORS[ERRORS_SIZE] = {
    "Cell overvoltage",               // 0x00
    "Cell undervoltage",              // 0x01
    "Pack overvoltage",               // 0x02
    "Pack undervoltage",              // 0x03
    "Charging over temperature",      // 0x04
    "Charging under temperature",     // 0x05
    "Discharging over temperature",   // 0x06
    "Discharging under temperature",  // 0x07
    "Charging overcurrent",           // 0x08
    "Discharging overcurrent",        // 0x09
    "Short circuit",                  // 0x0A
    "IC front-end error",             // 0x0B
    "Mosfet Software Lock",           // 0x0C
    "High ambient temperature",       // 0x0D
    "Low ambient temperature",        // 0x0E
    "FET high temperature",           // 0x0F
};

static const uint8_t ALARMS_SIZE = 16;
static const char *const ALARMS[ALARMS_SIZE] = {
    "Single cell low voltage alarm",       // 0x00
    "Single cell high voltage alarm",      // 0x01
    "Whole pack low voltage alarm",        // 0x02
    "Whole pack high voltage alarm",       // 0x03
    "Charging over current alarm",         // 0x04
    "Discharge over current alarm",        // 0x05
    "Charging high temperature alarm",     // 0x06
    "Charging low temperature alarm",      // 0x07
    "Discharge high temperature alarm",    // 0x08
    "Low temperature discharge alarm",     // 0x09
    "Ambient high temperature alarm",      // 0x0A
    "Ambient low temperature alarm",       // 0x0B
    "PCB high temperature alarm",          // 0x0C
    "Large differential pressure alarm",   // 0x0D
    "Low capacity alarms",                 // 0x0E
    "Empty",                               // 0x0F
};

void JbdBms::setup() { 
  this->send_command_(JBD_CMD_READ, JBD_CMD_HWINFO); 
}

void JbdBms::proces_response_(void)
{
  
  for(;;) {
    const uint32_t now = millis();
    if (now - this->last_byte_ > this->rx_timeout_) {
      ESP_LOGVV(TAG, "Buffer cleared due to timeout: %s",
                format_hex_pretty(&this->rx_buffer_.front(), this->rx_buffer_.size()).c_str());
      this->rx_buffer_.clear();
      this->last_byte_ = now;
      return;
    }

    while (this->available()) {
      uint8_t byte;
      this->read_byte(&byte);
      if (this->parse_jbd_bms_byte_(byte)) {
        this->last_byte_ = now;
      } else {
        ESP_LOGVV(TAG, "Buffer cleared due to reset: %s",
                  format_hex_pretty(&this->rx_buffer_.front(), this->rx_buffer_.size()).c_str());
        this->rx_buffer_.clear();
      }
    }
  }
}

void JbdBms::loop() {

}

void JbdBms::update() {
  this->track_online_status_();
  this->send_command_(JBD_CMD_READ, JBD_CMD_HWINFO);
  this->proces_response_();
  this->send_command_(JBD_CMD_READ, JBD_CMD_CELLINFO);
  this->proces_response_();

  if (this->enable_fake_traffic_) {
    // Start: 0xDD modbus_addr 0x03 0x00 0x1B
    this->on_jbd_bms_data_(JBD_CMD_HWINFO, {0x17,0x00,0x00,0x00,0x02,0xD0,0x03,0xE8,0x00,0x00,0x20,0x78
                                           ,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x48,0x03,0x0F,0x00,0x02,0x0B,
                                            0x76,0x0B,0x82,0x04, 0x0B, 0x8D, 0x0B, 0x8D, 0x0B, 0x8D, 0x0B, 0x8D});
    // End: CRC_HI , CRC_LO,  0x77

    // Start: 0xDD modbus_addr 0x04 0x00 0x1E
    this->on_jbd_bms_data_(JBD_CMD_CELLINFO, {0x0F , 0x66 , 0x0F , 0x63 , 0x0F , 0x63 , 0x0F , 0x64 , 0x0F , 0x3E , 0x0F , 0x63 , 0x0F , 0x37 
                                            , 0x0F , 0x5B , 0x0F , 0x65 , 0x0F , 0x3B , 0x0F , 0x63 , 0x0F , 0x63 , 0x0F , 0x3C , 0x0F , 0x66 , 
                                              0x0F , 0x3D});
    // End: CRC_HI , CRC_LO,  0x77

  }
}

bool JbdBms::parse_jbd_bms_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  if (at == 0) {
    if (raw[0] != 0xDD) {
      ESP_LOGW(TAG, "Invalid header: 0x%02X", raw[0]);

      // return false to reset buffer
      return false;
    }

    return true;
  }

  // Byte 1 (Modbus Addr)
  // Byte 2 (Function)
  // Byte 3 (Status)
  // Byte 4 (Length)
  if (at < 4)
    return true;

  if(raw[1]!= this->modbus_id_) 
    return true;

  uint16_t data_len = raw[4];
  uint16_t frame_len = 5 + data_len + 3;

  // Byte 0...4+data_len+3
  if (at < frame_len - 1)
    return true;

  uint8_t function = raw[2];
  uint16_t computed_crc = chksum_(raw + 1, data_len + 4);
  uint16_t remote_crc = uint16_t(raw[frame_len - 3]) << 8 | (uint16_t(raw[frame_len - 2]) << 0);
  if (computed_crc != remote_crc) {
    ESP_LOGW(TAG, "CRC check failed! 0x%04X != 0x%04X", computed_crc, remote_crc);
    ESP_LOGW(TAG, "RX frame %s", format_hex_pretty(raw, frame_len).c_str());
    return false;
  }

  ESP_LOGVV(TAG, "RX <- %s", format_hex_pretty(raw, at + 1).c_str());

  std::vector<uint8_t> data(this->rx_buffer_.begin() + 5, this->rx_buffer_.begin() +5+data_len );

  this->on_jbd_bms_data_(function, data);

  // return false to reset buffer
  return false;
}

void JbdBms::on_jbd_bms_data_(const uint8_t &function, const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();

  switch (function) {
    case JBD_CMD_HWINFO:
      this->on_hardware_info_data_(data);
      break;
    case JBD_CMD_CELLINFO:
      this->on_cell_info_data_(data);
      break;
    default:
      ESP_LOGW(TAG, "Unhandled response received: %s", format_hex_pretty(&data.front(), data.size()).c_str());
  }
}

void JbdBms::on_cell_info_data_(const std::vector<uint8_t> &data) {
  auto jbd_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0);
  };

  ESP_LOGI(TAG, "Cell info frame (%d bytes) received Modbus Addres %d", data.size() , this->modbus_id_);
  ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), data.size()).c_str());

  uint8_t data_len = data.size();
  if (data_len < 2 || data_len > 64 || (data_len % 2) != 0) {
    ESP_LOGW(TAG, "Skipping cell info frame because of invalid length: %d", data_len);
    return;
  }

  // Byte Len  Payload                Content              Coeff.      Unit        Example value
  // 0     2   0x0F 0x3B              Cell voltage 1       0.001       V           3899 = 3.899
  // 2     2   0x0F 0x30              Cell voltage 2       0.001       V
  // 4     2   0x0F 0x2D              Cell voltage 3       0.001       V
  // 6     2   0x0F 0x31              Cell voltage 4       0.001       V
  uint8_t cells = std::min(data_len / 2, 32);
  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  float average_cell_voltage = 0.0f;
  uint8_t min_voltage_cell = 0;
  uint8_t max_voltage_cell = 0;
  for (uint8_t i = 0; i < cells; i++) {
    float cell_voltage = (float) jbd_get_16bit(0 + (i * 2)) * 0.001f;
    average_cell_voltage = average_cell_voltage + cell_voltage;
    if (cell_voltage < min_cell_voltage) {
      min_cell_voltage = cell_voltage;
      min_voltage_cell = i + 1;
    }
    if (cell_voltage > max_cell_voltage) {
      max_cell_voltage = cell_voltage;
      max_voltage_cell = i + 1;
    }
    this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
  }
  average_cell_voltage = average_cell_voltage / cells;

  this->publish_state_(this->min_cell_voltage_sensor_, min_cell_voltage);
  this->publish_state_(this->max_cell_voltage_sensor_, max_cell_voltage);
  this->publish_state_(this->max_voltage_cell_sensor_, (float) max_voltage_cell);
  this->publish_state_(this->min_voltage_cell_sensor_, (float) min_voltage_cell);
  this->publish_state_(this->delta_cell_voltage_sensor_, max_cell_voltage - min_cell_voltage);
  this->publish_state_(this->average_cell_voltage_sensor_, average_cell_voltage);
}

void JbdBms::on_hardware_info_data_(const std::vector<uint8_t> &data) {
  auto jbd_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0);
  };
  auto jbd_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(jbd_get_16bit(i + 0)) << 16) | (uint32_t(jbd_get_16bit(i + 2)) << 0);
  };

  ESP_LOGI(TAG, "Hardware info frame (%d bytes) received Modbus Addres %d", data.size() , this->modbus_id_);
  if (data.size() == 0 ) {
    ESP_LOGW(TAG, "Skipping hardware info frame because of invalid length: %d", data.size());
    return;
  }

  ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), data.size()).c_str());
 

  ESP_LOGD(TAG, "  Device model: %s", this->device_model_.c_str());

  // Byte Len  Payload                Content              Coeff.      Unit        Example value
  //  0    2   0x06 0x17              Total voltage                                1559
  float total_voltage = jbd_get_16bit(0) * 0.01f;
  this->publish_state_(this->total_voltage_sensor_, total_voltage);

  //  2    2   0x00 0x00              Current
  float current = (float) ((int16_t) jbd_get_16bit(2)) * 0.01f;
  float power = total_voltage * current;
  this->publish_state_(this->current_sensor_, current);
  this->publish_state_(this->power_sensor_, power);
  this->publish_state_(this->charging_power_sensor_, std::max(0.0f, power));               // 500W vs 0W -> 500W
  this->publish_state_(this->discharging_power_sensor_, std::abs(std::min(0.0f, power)));  // -500W vs 0W -> 500W

  //  4    2   0x01 0xF3              Residual Capacity                            499
  this->publish_state_(this->capacity_remaining_sensor_, (float) jbd_get_16bit(4) * 0.01f);

  //  6    2   0x01 0xF4              Nominal Capacity                             500
  this->publish_state_(this->nominal_capacity_sensor_, (float) jbd_get_16bit(6) * 0.01f);

  //  8    2   0x00 0x00              Cycle Life
  this->publish_state_(this->charging_cycles_sensor_, (float) jbd_get_16bit(8));

  // 10    2   0x2C 0x7C              Production date
  uint16_t production_date = jbd_get_16bit(10);
  ESP_LOGD(TAG, "  Date of manufacture: %d.%d.%d", 2000 + (production_date >> 9), (production_date >> 5) & 0x0f,
           production_date & 0x1f);

  // 12    4   0x00 0x00 0x00 0x00    Balancer Status
  uint32_t balance_status_bitmask = jbd_get_32bit(12);
  this->publish_state_(this->balancer_status_bitmask_sensor_, (float) balance_status_bitmask);
  this->publish_state_(this->balancing_binary_sensor_, balance_status_bitmask > 0);

  // 16    2   0x00 0x00              Protection Status
  uint16_t errors_bitmask = jbd_get_16bit(16);
  this->publish_state_(this->errors_bitmask_sensor_, (float) errors_bitmask);
  this->publish_state_(this->errors_text_sensor_, this->error_bits_to_string_(errors_bitmask));

  // 18    1   0x80                   Version                                      0x10 = 1.0, 0x80 = 8.0
  this->publish_state_(this->software_version_sensor_, (data[18] >> 4) + ((data[18] & 0x0f) * 0.1f));

  // 19    1   0x64                   Stage of charge
  this->publish_state_(this->state_of_charge_sensor_, data[19]);

  // 20    1   0x03                   Mosfet bitmask
  uint8_t operation_status = data[20];
  this->publish_state_(this->operation_status_bitmask_sensor_, operation_status);
  this->publish_state_(this->charging_binary_sensor_, operation_status & JBD_MOS_CHARGE);
  this->publish_state_(this->discharging_binary_sensor_, operation_status & JBD_MOS_DISCHARGE);

  // 21    1   0x04                   Cell count
  this->publish_state_(this->battery_strings_sensor_, data[21]);

  // 22    2   0x00 0x00              Alarm Status
  uint16_t alarm_bitmask = jbd_get_16bit(22);
  this->publish_state_(this->alarm_bitmask_sensor_, (float) alarm_bitmask);
  this->publish_state_(this->alarms_text_sensor_, this->alarm_bits_to_string_(alarm_bitmask));

  // 24    2   0x00 0x00              Ambient temp sensor
  this->publish_state_(this->temperature_ambient_sensor_,(float) (jbd_get_16bit(24) - 2731) * 0.1f);

  // 26    2   0x00 0x00              FET temp sensor
  this->publish_state_(this->temperature_fet_sensor_,(float) (jbd_get_16bit(26) - 2731) * 0.1f);

  // 28    1   0x04                   Temperature sensors cnt
  // 29    2   0x0B 0x8D              Temperature 1
  // 31    2   0x0B 0x8C              Temperature 2
  // 33    2   0x0B 0x88              Temperature 3
  // 35    2   0x0B 0x88              Temperature 4
  uint8_t temperature_sensors = std::min(data[28], (uint8_t) 4);
  this->publish_state_(this->temperature_sensors_sensor_, temperature_sensors);
  for (uint8_t i = 0; i < temperature_sensors; i++) {
    this->publish_state_(this->temperatures_[i].temperature_sensor_,
                         (float) (jbd_get_16bit(29 + (i * 2)) - 2731) * 0.1f);
  }

}

void JbdBms::track_online_status_() {
  if (this->no_response_count_ < MAX_NO_RESPONSE_COUNT) {
    this->no_response_count_++;
  }
  if (this->no_response_count_ == MAX_NO_RESPONSE_COUNT) {
    this->publish_device_unavailable_();
    this->no_response_count_++;
  }
}

void JbdBms::reset_online_status_tracker_() {
  this->no_response_count_ = 0;
  this->publish_state_(this->online_status_binary_sensor_, true);
}

void JbdBms::publish_device_unavailable_() {
  this->publish_state_(this->online_status_binary_sensor_, false);
  this->publish_state_(this->errors_text_sensor_, "Offline");
  this->publish_state_(this->alarms_text_sensor_, "Offline");

  this->publish_state_(state_of_charge_sensor_, NAN);
  this->publish_state_(total_voltage_sensor_, NAN);
  this->publish_state_(current_sensor_, NAN);
  this->publish_state_(power_sensor_, NAN);
  this->publish_state_(charging_power_sensor_, NAN);
  this->publish_state_(discharging_power_sensor_, NAN);
  this->publish_state_(nominal_capacity_sensor_, NAN);
  this->publish_state_(charging_cycles_sensor_, NAN);
  this->publish_state_(capacity_remaining_sensor_, NAN);
  this->publish_state_(min_cell_voltage_sensor_, NAN);
  this->publish_state_(max_cell_voltage_sensor_, NAN);
  this->publish_state_(min_voltage_cell_sensor_, NAN);
  this->publish_state_(max_voltage_cell_sensor_, NAN);
  this->publish_state_(delta_cell_voltage_sensor_, NAN);
  this->publish_state_(average_cell_voltage_sensor_, NAN);
  this->publish_state_(operation_status_bitmask_sensor_, NAN);
  this->publish_state_(errors_bitmask_sensor_, NAN);
  this->publish_state_(alarm_bitmask_sensor_, NAN);
  this->publish_state_(balancer_status_bitmask_sensor_, NAN);
  this->publish_state_(battery_strings_sensor_, NAN);
  this->publish_state_(temperature_sensors_sensor_, NAN);
  this->publish_state_(temperature_ambient_sensor_, NAN);
  this->publish_state_(temperature_fet_sensor_, NAN);
  this->publish_state_(software_version_sensor_, NAN);

  for (auto &temperature : this->temperatures_) {
    this->publish_state_(temperature.temperature_sensor_, NAN);
  }

  for (auto &cell : this->cells_) {
    this->publish_state_(cell.cell_voltage_sensor_, NAN);
  }
}

void JbdBms::dump_config() {  // NOLINT(google-readability-function-size,readability-function-size)
  ESP_LOGCONFIG(TAG, "JbdBms:");
  ESP_LOGCONFIG(TAG, "  RX timeout: %d ms", this->rx_timeout_);
  ESP_LOGCONFIG(TAG, "  Fake traffic enabled: %s", YESNO(this->enable_fake_traffic_));

  LOG_BINARY_SENSOR("", "Balancing", this->balancing_binary_sensor_);
  LOG_BINARY_SENSOR("", "Charging", this->charging_binary_sensor_);
  LOG_BINARY_SENSOR("", "Discharging", this->discharging_binary_sensor_);

  LOG_SENSOR("", "Total voltage", this->total_voltage_sensor_);
  LOG_SENSOR("", "Battery strings", this->battery_strings_sensor_);
  LOG_SENSOR("", "Software version", this->software_version_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Charging Power", this->charging_power_sensor_);
  LOG_SENSOR("", "Discharging Power", this->discharging_power_sensor_);
  LOG_SENSOR("", "State of charge", this->state_of_charge_sensor_);
  LOG_SENSOR("", "Operation status bitmask", operation_status_bitmask_sensor_);
  LOG_SENSOR("", "Errors bitmask", errors_bitmask_sensor_);
  LOG_SENSOR("", "Alarm bitmask", alarm_bitmask_sensor_);
  LOG_SENSOR("", "Nominal capacity", this->nominal_capacity_sensor_);
  LOG_SENSOR("", "Charging cycles", this->charging_cycles_sensor_);
  LOG_SENSOR("", "Balancer status bitmask", balancer_status_bitmask_sensor_);
  LOG_SENSOR("", "Capacity remaining", this->capacity_remaining_sensor_);
  LOG_SENSOR("", "Average cell voltage sensor", this->average_cell_voltage_sensor_);
  LOG_SENSOR("", "Delta cell voltage sensor", delta_cell_voltage_sensor_);
  LOG_SENSOR("", "Maximum cell voltage", this->max_cell_voltage_sensor_);
  LOG_SENSOR("", "Min voltage cell", this->min_voltage_cell_sensor_);
  LOG_SENSOR("", "Max voltage cell", this->max_voltage_cell_sensor_);
  LOG_SENSOR("", "Minimum cell voltage", this->min_cell_voltage_sensor_);
  LOG_SENSOR("", "Temperature sensors", temperature_sensors_sensor_);
  LOG_SENSOR("", "Temperature ambient sensor", temperature_ambient_sensor_);
  LOG_SENSOR("", "Temperature fet sensor", temperature_fet_sensor_);
  LOG_SENSOR("", "Temperature 1", this->temperatures_[0].temperature_sensor_);
  LOG_SENSOR("", "Temperature 2", this->temperatures_[1].temperature_sensor_);
  LOG_SENSOR("", "Temperature 3", this->temperatures_[2].temperature_sensor_);
  LOG_SENSOR("", "Temperature 4", this->temperatures_[3].temperature_sensor_);
  LOG_SENSOR("", "Cell Voltage 1", this->cells_[0].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 2", this->cells_[1].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 3", this->cells_[2].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 4", this->cells_[3].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 5", this->cells_[4].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 6", this->cells_[5].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 7", this->cells_[6].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 8", this->cells_[7].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 9", this->cells_[8].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 10", this->cells_[9].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 11", this->cells_[10].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 12", this->cells_[11].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 13", this->cells_[12].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 14", this->cells_[13].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 15", this->cells_[14].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 16", this->cells_[15].cell_voltage_sensor_);
  LOG_TEXT_SENSOR("", "Operation status", this->operation_status_text_sensor_);
  LOG_TEXT_SENSOR("", "Errors", this->errors_text_sensor_);
  LOG_TEXT_SENSOR("", "Alarms", this->alarms_text_sensor_);
  LOG_TEXT_SENSOR("", "Device model", this->device_model_text_sensor_);

  this->check_uart_settings(9600);
}

float JbdBms::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void JbdBms::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;

  binary_sensor->publish_state(state);
}

void JbdBms::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

void JbdBms::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;

  text_sensor->publish_state(state);
}

void JbdBms::write_register(uint8_t address, uint16_t value) {
  this->send_command_(JBD_CMD_WRITE, JBD_CMD_MOS);  // @TODO: Pass value
}

void JbdBms::send_command_(uint8_t action, uint8_t function) {
  uint8_t frame[8];
  uint8_t data_len = 0;

  frame[0] = JBD_PKT_START;
  frame[1] = this->modbus_id_;
  frame[2] = action;
  frame[3] = function;
  frame[4] = data_len;
  auto crc = chksum_(frame + 1, data_len + 4);
  frame[5] = crc >> 8;
  frame[6] = crc >> 0;
  frame[7] = JBD_PKT_END;

  this->write_array(frame, 8);
  this->flush();
  
}

std::string JbdBms::error_bits_to_string_(const uint16_t mask) {
  std::string values = "";
  if (mask) {
    for (int i = 0; i < ERRORS_SIZE; i++) {
      if (mask & (1 << i)) {
        values.append(ERRORS[i]);
        values.append(";");
      }
    }
    if (!values.empty()) {
      values.pop_back();
    }
  }
  return values;
}

std::string JbdBms::alarm_bits_to_string_(const uint16_t mask) {
  std::string values = "";
  if (mask) {
    for (int i = 0; i < ALARMS_SIZE; i++) {
      if (mask & (1 << i)) {
        values.append(ALARMS[i]);
        values.append(";");
      }
    }
    if (!values.empty()) {
      values.pop_back();
    }
  }
  return values;
}

}  // namespace jbd_bms
}  // namespace esphome
