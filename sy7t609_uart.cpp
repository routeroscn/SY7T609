#include "sy7t609_uart.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace sy7t609 {

static const char *const TAG = "sy7t609";

// ============================================================================
// 测量值解析函数
// ============================================================================
float SY7T609_UART::parse_milli_(uint32_t raw) {
  return raw / 1000.0f;
}

// 24bit 二进制补码：>= 0x800000 视为负数。
// 注意：此处沿用原实现的语义——返回幅值（绝对值），保持测量输出兼容。
float SY7T609_UART::parse_abs_milli_(uint32_t raw) {
  uint32_t v = (raw >= 0x800000) ? (0x01000000u - raw) : raw;
  return static_cast<float>(v) / 1000.0f;
}

// 24bit 二进制补码：保留正负号。
// 正值=正向（用电/吸收），负值=反向（回馈电网/光伏倒送、容性无功）。
float SY7T609_UART::parse_signed_milli_(uint32_t raw) {
  int32_t v = (raw >= 0x800000) ? (static_cast<int32_t>(raw) - 0x01000000) : static_cast<int32_t>(raw);
  return static_cast<float>(v) / 1000.0f;
}

float SY7T609_UART::parse_raw_(uint32_t raw) {
  return static_cast<float>(raw);
}

// ============================================================================
// 测量值注册表（表驱动核心）
// 想新增一种读取数据：在此表加一行 + 在 .h 增加成员/ setter + 在 sensor.py 注册即可。
// ============================================================================
const std::array<SY7T609_UART::Measurement, 7> &SY7T609_UART::get_measurements_() {
  static const std::array<Measurement, 7> table = {{
      {ADDR_PF,     &SY7T609_UART::power_factor_sensor_,   &SY7T609_UART::parse_signed_milli_},
      {ADDR_VRMS,   &SY7T609_UART::voltage_sensor_,        &SY7T609_UART::parse_milli_},
      {ADDR_IRMS,   &SY7T609_UART::current_sensor_,        &SY7T609_UART::parse_milli_},
      {ADDR_POWER,  &SY7T609_UART::power_sensor_,          &SY7T609_UART::parse_signed_milli_},
      {ADDR_VAR,    &SY7T609_UART::power_reactive_sensor_, &SY7T609_UART::parse_signed_milli_},
      {ADDR_EPPCNT, &SY7T609_UART::energy_sensor_,         &SY7T609_UART::parse_raw_},
      {ADDR_CTEMP,  &SY7T609_UART::temperature_sensor_,    &SY7T609_UART::parse_milli_},
  }};
  return table;
}

// ============================================================================
// 生命周期
// ============================================================================
void SY7T609_UART::setup() {
  this->flush();
  // 上电软复位，芯片会从自身 flash 载入已保存的校准值。
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_SOFT_RESET);
}

void SY7T609_UART::loop() {
  handleActionCallback();

  const uint32_t now = millis();

  // 超时恢复：超过 500ms 没有完整帧，则清空缓冲并复位读取状态。
  // （软复位后偶尔收不到回复，会卡住状态机，这里作为兜底补丁。）
  if (now - this->last_read_ > 500 && this->available() != SSI_UART_READ_RECV_PKG_SIZE) {
    while (this->available()) {
      this->read();
      yield();
    }
    this->last_read_ = now;
    if (m_state != ProcessState::IDLE) {
      m_state = ProcessState::IDLE;
      read_index_ = 0;
    }
  }

  // 收到完整回复帧且正处于读取等待中：解析并发布，然后推进到下一项。
  if (now - this->last_read_ > 20 && this->available() == SSI_UART_READ_RECV_PKG_SIZE &&
      m_state == ProcessState::READING) {
    auto resp = *this->read_array<SSI_UART_READ_RECV_PKG_SIZE>();
    bool ok = true;

    if (resp[0] != REPLY_ACK_WITH_DATA) {
      ESP_LOGE(TAG, "SY7T609 read_index[%u], invalid REPLY_ACK_WITH_DATA! 0x%02X",
               static_cast<unsigned>(read_index_), resp[0]);
      ok = false;
    }
    uint8_t sum = checksum<SSI_UART_READ_RECV_PKG_SIZE>(resp);
    if (sum != resp[resp.max_size() - 1]) {
      ESP_LOGW(TAG, "SY7T609 read_index[%u], invalid checksum! 0x%02X != 0x%02X",
               static_cast<unsigned>(read_index_), sum, resp[resp.max_size() - 1]);
      ok = false;
    }

    const auto &table = get_measurements_();
    const Measurement &m = table[read_index_];
    sensor::Sensor *target = this->*(m.sensor_ptr);
    if (ok && target != nullptr) {
      target->publish_state(m.parse(extractData(resp)));
    }

    advance_read_();
    this->last_read_ = now;
  }
}

void SY7T609_UART::update() {
  if (m_state == ProcessState::IDLE && !isNeedHandleActionCallback()) {
    if (seek_next_active_(0)) {
      start_current_read_();
    }
  } else {
    ESP_LOGD(TAG, "SY7T609 skip update, state[%d], callbackQueue[%u].",
             static_cast<int>(m_state), static_cast<unsigned>(m_vecActionCallback.size()));
  }
}

// ============================================================================
// 读取流程（表驱动）
// ============================================================================
bool SY7T609_UART::seek_next_active_(std::size_t from) {
  const auto &table = get_measurements_();
  for (std::size_t i = from; i < table.size(); i++) {
    if (this->*(table[i].sensor_ptr) != nullptr) {
      read_index_ = i;
      return true;
    }
  }
  return false;
}

void SY7T609_UART::start_current_read_() {
  const auto &table = get_measurements_();
  uartSendReadCmd(table[read_index_].addr);
  m_state = ProcessState::READING;
}

void SY7T609_UART::advance_read_() {
  if (seek_next_active_(read_index_ + 1)) {
    start_current_read_();
  } else {
    m_state = ProcessState::IDLE;
    read_index_ = 0;
  }
}

// ============================================================================
// SSI 读写
// ============================================================================
void SY7T609_UART::uartSendReadCmd(uint16_t addr) {
  std::array<uint8_t, SSI_UART_READ_SEND_PKG_SIZE> data{};
  data[0] = SSI_HEADER;
  data[1] = SSI_UART_READ_SEND_PKG_SIZE;
  data[2] = CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = CMD_READ_REGITSTER_3BYTES;
  data[6] = checksum<SSI_UART_READ_SEND_PKG_SIZE>(data);

  this->write_array(data);
  this->flush();
  delay(0);
}

void SY7T609_UART::uartSendWriteCmd(uint16_t addr, uint32_t value) {
  std::array<uint8_t, SSI_UART_WRITE_SEND_PKG_SIZE> data{};
  data[0] = SSI_HEADER;
  data[1] = SSI_UART_WRITE_SEND_PKG_SIZE;
  data[2] = CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = CMD_WRITE_RETISTER_3BYTES;
  data[6] = value & 0xFF;
  data[7] = (value >> 8) & 0xFF;
  data[8] = (value >> 16) & 0xFF;
  data[9] = checksum<SSI_UART_WRITE_SEND_PKG_SIZE>(data);

  this->write_array(data);
  this->flush();
  delay(0);
}

// ============================================================================
// 动作回调队列
// ============================================================================
int SY7T609_UART::addActionCallBack(ActionCallbackFuncPtr ptrFunc) {
  m_vecActionCallback.push_back(ptrFunc);
  return m_vecActionCallback.size();
}

void SY7T609_UART::handleActionCallback() {
  if (m_vecActionCallback.empty()) {
    return;
  }
  for (std::size_t i = 0; i < m_vecActionCallback.size(); i++) {
    ActionCallbackFuncPtr ptrFunc = m_vecActionCallback[i];
    if (ptrFunc) {
      ESP_LOGI(TAG, "SY7T609 handleActionCallback[%u]...", static_cast<unsigned>(i));
      (this->*ptrFunc)();
    }
  }

  // 清空可能残留的回复数据，并复位读取状态。
  while (this->available()) {
    this->read();
  }
  m_vecActionCallback.clear();
  m_state = ProcessState::IDLE;
  read_index_ = 0;
}

// ============================================================================
// 动作实现
// ============================================================================
void SY7T609_UART::reset_energy_() {
  ESP_LOGI(TAG, "SY7T609 reset energy...");
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_CLEAR_ENGERGY_COUNTERS);
  delay(20);
}

void SY7T609_UART::reset_calibration_() {
  ESP_LOGI(TAG, "SY7T609 apply fixed calibration...");
  uartSendWriteCmd(ADDR_IGAIN, DEFAULT_IGAIN);              delay(20);
  uartSendWriteCmd(ADDR_VGAIN, DEFAULT_VGAIN);              delay(20);
  uartSendWriteCmd(ADDR_ISCALE, DEFAULT_ISCALE);           delay(20);
  uartSendWriteCmd(ADDR_VSCALE, DEFAULT_VSCALE);           delay(20);
  uartSendWriteCmd(ADDR_PSCALE, DEFAULT_PSCALE);           delay(20);
  uartSendWriteCmd(ADDR_ACCUM, DEFAULT_ACCUM);             delay(20);
  uartSendWriteCmd(ADDR_IRMS_TARGET, DEFAULT_IRMS_TARGET); delay(20);
  uartSendWriteCmd(ADDR_VRMS_TARGET, DEFAULT_VRMS_TARGET); delay(20);
  uartSendWriteCmd(ADDR_POWER_TARGET, DEFAULT_POWER_TARGET); delay(20);
  uartSendWriteCmd(ADDR_BUCKETL, DEFAULT_BUCKETL);         delay(20);
  uartSendWriteCmd(ADDR_BUCKETH, DEFAULT_BUCKETH);         delay(20);
  uartSendWriteCmd(ADDR_CONTROL, DEFAULT_CONTROL);         delay(20);
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_SAVE_TO_FLASH);
  delay(20);
  ESP_LOGI(TAG, "SY7T609 calibration saved to flash.");
}

// 寄存器目标值 = 真实工程量 × 1000，并裁剪到 24bit。
static uint32_t to_target_milli_(float real_value) {
  long v = lroundf(real_value * 1000.0f);
  if (v < 0) v = 0;
  return static_cast<uint32_t>(v) & 0xFFFFFFu;
}

void SY7T609_UART::calibrate_voltage_() {
  uint32_t target = to_target_milli_(pending_cal_voltage_);
  ESP_LOGI(TAG, "SY7T609 calibrate VOLTAGE: real=%.3f V -> VRMS_TARGET=0x%06X",
           pending_cal_voltage_, static_cast<unsigned>(target));
  uartSendWriteCmd(ADDR_VRMS_TARGET, target);                  delay(20);
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_CALIBRATION_VOLTAGE); delay(40);
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_SAVE_TO_FLASH);       delay(40);
  ESP_LOGI(TAG, "SY7T609 voltage calibration done & saved.");
}

void SY7T609_UART::calibrate_current_() {
  uint32_t target = to_target_milli_(pending_cal_current_);
  ESP_LOGI(TAG, "SY7T609 calibrate CURRENT: real=%.3f A -> IRMS_TARGET=0x%06X",
           pending_cal_current_, static_cast<unsigned>(target));
  uartSendWriteCmd(ADDR_IRMS_TARGET, target);                  delay(20);
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_CALIBRATION_CURRENT); delay(40);
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_SAVE_TO_FLASH);       delay(40);
  ESP_LOGI(TAG, "SY7T609 current calibration done & saved.");
}

void SY7T609_UART::calibrate_generic_() {
  ESP_LOGI(TAG, "SY7T609 calibrate GENERIC: reg=0x%04X val=0x%06X cmd=0x%06X",
           static_cast<unsigned>(pending_cal_addr_),
           static_cast<unsigned>(pending_cal_value_),
           static_cast<unsigned>(pending_cal_command_));
  if (pending_cal_addr_ != ADDR_ERROR) {
    uartSendWriteCmd(pending_cal_addr_, pending_cal_value_);   delay(20);
  }
  if (pending_cal_command_ != CMD_NONE) {
    uartSendWriteCmd(ADDR_COMMAND, pending_cal_command_);      delay(40);
  }
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_SAVE_TO_FLASH);       delay(40);
  ESP_LOGI(TAG, "SY7T609 generic calibration done & saved.");
}

void SY7T609_UART::save_calibration_() {
  ESP_LOGI(TAG, "SY7T609 save calibration to flash...");
  uartSendWriteCmd(ADDR_COMMAND, CMD_REG_SAVE_TO_FLASH);
  delay(20);
}

}  // namespace sy7t609
}  // namespace esphome
