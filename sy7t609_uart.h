#pragma once

#include <array>
#include <vector>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/datatypes.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "sy7t609_def.h"

namespace esphome {
namespace sy7t609 {

template<typename... Ts> class ResetEnergyAction;
template<typename... Ts> class ResetCalibrationAction;
template<typename... Ts> class CalibrateVoltageAction;
template<typename... Ts> class CalibrateCurrentAction;
template<typename... Ts> class CalibrateGenericAction;
template<typename... Ts> class SaveCalibrationAction;

class SY7T609_UART;
typedef void (SY7T609_UART::*ActionCallbackFuncPtr)(void);

// 单个测量值的解析函数原型：入参为从回复帧中提取出的 24bit 原始值，返回工程量。
typedef float (*MeasurementParser)(uint32_t raw);

class SY7T609_UART : public PollingComponent, public uart::UARTDevice {
 public:
  // ------------------------------------------------------------------
  // 测量值表项：地址 + 目标传感器（指向成员指针）+ 解析函数
  // 新增一种读取数据，只需在 get_measurements_() 中增加一行即可。
  // ------------------------------------------------------------------
  struct Measurement {
    uint16_t addr;
    sensor::Sensor *SY7T609_UART::*sensor_ptr;
    MeasurementParser parse;
  };

  // ---------------- 传感器 setter（保持原有 YAML 调用兼容）----------------
  void set_power_factor_sensor(sensor::Sensor *s) { power_factor_sensor_ = s; }
  void set_voltage_sensor(sensor::Sensor *s) { voltage_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { current_sensor_ = s; }
  void set_power_sensor(sensor::Sensor *s) { power_sensor_ = s; }
  void set_power_reactive_sensor(sensor::Sensor *s) { power_reactive_sensor_ = s; }
  void set_energy_sensor(sensor::Sensor *s) { energy_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { temperature_sensor_ = s; }

  void setup() override;
  void loop() override;
  void update() override;

 protected:
  // ---------------- 读取流程（表驱动）----------------
  const std::array<Measurement, 7> &get_measurements_();
  bool seek_next_active_(std::size_t from);  // 找到下一个已配置传感器的表项
  void start_current_read_();                // 发起当前表项的读取
  void advance_read_();                      // 推进到下一项或结束

  // ---------------- 寄存器读写 ----------------
  void uartSendReadCmd(uint16_t addr);
  void uartSendWriteCmd(uint16_t addr, uint32_t value);

  // ---------------- 动作实现 ----------------
  void reset_energy_();       // 清除电能计数
  void reset_calibration_();  // 把固定校准值（def.h）写回芯片并存 flash

  // 自动校准：写"真实值"到对应 TARGET 寄存器 -> 发校准命令 -> 芯片反算增益 -> 存 flash
  void calibrate_voltage_();  // 依据 pending_cal_voltage_（实际电压 V）校准 VGAIN
  void calibrate_current_();  // 依据 pending_cal_current_（实际电流 A）校准 IGAIN
  // 通用校准：命令码 / 目标寄存器 / 寄存器值 全部由 YAML 提供，
  // 用于按数据手册做功率、相位补偿等高级校准（不硬编码未验证的命令码）。
  void calibrate_generic_();
  void save_calibration_();   // 仅发送 SAVE_TO_FLASH

  // ---------------- 测量值解析函数（静态，无状态）----------------
  static float parse_milli_(uint32_t raw);        // 原始值 / 1000
  static float parse_abs_milli_(uint32_t raw);    // 24bit 有符号 -> 取幅值 / 1000
  static float parse_signed_milli_(uint32_t raw); // 24bit 有符号 -> 保留正负 / 1000
  static float parse_raw_(uint32_t raw);          // 原始值直接返回

  // ---------------- 动作回调队列 ----------------
  int addActionCallBack(ActionCallbackFuncPtr ptrFunc);
  void handleActionCallback();
  bool isNeedHandleActionCallback() { return !m_vecActionCallback.empty(); }

  template<std::size_t SIZE>
  uint8_t checksum(const std::array<uint8_t, SIZE> &array) {
    uint8_t sum = 0;
    for (std::size_t i = 0; i < array.max_size() - 1; i++) {
      sum += array[i];
    }
    return ~sum + 1;
  }

  // 从读取回复帧中提取 24bit 数据（低字节在前）
  static uint32_t extractData(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array) {
    return (static_cast<uint32_t>(array[4]) << 16) |
           (static_cast<uint32_t>(array[3]) << 8) |
           static_cast<uint32_t>(array[2]);
  }

 protected:
  // 友元动作
  template<typename... Ts> friend class ResetEnergyAction;
  template<typename... Ts> friend class ResetCalibrationAction;
  template<typename... Ts> friend class CalibrateVoltageAction;
  template<typename... Ts> friend class CalibrateCurrentAction;
  template<typename... Ts> friend class CalibrateGenericAction;
  template<typename... Ts> friend class SaveCalibrationAction;

  // 运行状态
  ProcessState m_state{ProcessState::IDLE};
  std::size_t read_index_{0};
  uint32_t last_read_{0};

  // 校准动作待处理参数（在 Action::play 中写入，延迟到回调队列执行）
  float pending_cal_voltage_{0.0f};   // 实际电压 (V)
  float pending_cal_current_{0.0f};   // 实际电流 (A)
  uint16_t pending_cal_addr_{ADDR_ERROR};  // 通用校准目标寄存器
  uint32_t pending_cal_value_{0};          // 通用校准写入值（原始寄存器值）
  uint32_t pending_cal_command_{CMD_NONE}; // 通用校准命令码

  // 传感器
  sensor::Sensor *power_factor_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *power_reactive_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

 private:
  std::vector<ActionCallbackFuncPtr> m_vecActionCallback{};
};

// ============================================================================
// Actions
// ============================================================================
template<typename... Ts> class ResetEnergyAction : public Action<Ts...> {
 public:
  ResetEnergyAction(SY7T609_UART *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->addActionCallBack(&SY7T609_UART::reset_energy_); }
 protected:
  SY7T609_UART *parent_;
};

template<typename... Ts> class ResetCalibrationAction : public Action<Ts...> {
 public:
  ResetCalibrationAction(SY7T609_UART *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->addActionCallBack(&SY7T609_UART::reset_calibration_); }
 protected:
  SY7T609_UART *parent_;
};

// 电压校准：传入实际电压（单位 V），芯片据此校准 VGAIN
template<typename... Ts> class CalibrateVoltageAction : public Action<Ts...> {
 public:
  CalibrateVoltageAction(SY7T609_UART *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(float, value)
  void play(Ts... x) override {
    this->parent_->pending_cal_voltage_ = this->value_.value(x...);
    this->parent_->addActionCallBack(&SY7T609_UART::calibrate_voltage_);
  }
 protected:
  SY7T609_UART *parent_;
};

// 电流校准：传入实际电流（单位 A），芯片据此校准 IGAIN
template<typename... Ts> class CalibrateCurrentAction : public Action<Ts...> {
 public:
  CalibrateCurrentAction(SY7T609_UART *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(float, value)
  void play(Ts... x) override {
    this->parent_->pending_cal_current_ = this->value_.value(x...);
    this->parent_->addActionCallBack(&SY7T609_UART::calibrate_current_);
  }
 protected:
  SY7T609_UART *parent_;
};

// 通用校准：命令码 / 目标寄存器 / 写入值均由 YAML 指定（按数据手册）
template<typename... Ts> class CalibrateGenericAction : public Action<Ts...> {
 public:
  CalibrateGenericAction(SY7T609_UART *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint32_t, command)
  TEMPLATABLE_VALUE(uint32_t, target_register)
  TEMPLATABLE_VALUE(uint32_t, value)
  void play(Ts... x) override {
    this->parent_->pending_cal_command_ = this->command_.value(x...);
    this->parent_->pending_cal_addr_ = static_cast<uint16_t>(this->target_register_.value(x...));
    this->parent_->pending_cal_value_ = this->value_.value(x...) & 0xFFFFFF;
    this->parent_->addActionCallBack(&SY7T609_UART::calibrate_generic_);
  }
 protected:
  SY7T609_UART *parent_;
};

// 手动保存校准到 flash
template<typename... Ts> class SaveCalibrationAction : public Action<Ts...> {
 public:
  SaveCalibrationAction(SY7T609_UART *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->addActionCallBack(&SY7T609_UART::save_calibration_); }
 protected:
  SY7T609_UART *parent_;
};

}  // namespace sy7t609
}  // namespace esphome
