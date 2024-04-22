#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "sy7t609-uart-protocol.h"

#define SSI_HEADER (0xAA)
#define SSI_DEFAULT_FRAME_SIZE (3)
#define SSI_MAX_PAYLOAD_SIZE (7)
#define SSI_READ_PAYLOAD_SIZE (4)
#define SSI_REPLY_PAYLOAD_SIZE (3)
#define SSI_WRITE_PAYLOAD_SIZE (7)

#define CMD_CLEAR_ADDRESS (0xA0)
#define CMD_SELECT_REGISTER_ADDRESS (0xA3)
#define CMD_READ_REGISTER_3BYTES (0xE3)
#define CMD_WRITE_REGISTER_3BYTES (0xD3)
#define AUTO_REPORT_PACKET_SIZE 31
int t609_error_count = 0;
// #define CONTROL (0x048817)
// #define VSCALE (667000)
// #define ISCALE (31500)
// #define PSCALE (164145)
// #define BUCKETH (0x1FD)
// #define BUCKETL (0x28E9F8)
// #define ACCUM (670 * 5)
//0x3A43C 238.5
typedef enum process_state {
  PROCESS_STATE_READ_1 = 0,
  PROCESS_STATE_READ_2,
  PROCESS_STATE_READ_3,
  PROCESS_STATE_READ_4
  // PROCESS_STATE_READ_5
  // PROCESS_STATE_READ_POWER,
  // PROCESS_STATE_READ_VAG_POWER,
  // PROCESS_STATE_READ_EPPCNT,
  // PROCESS_STATE_READ_EPMCNT,
  // PROCESS_STATE_READ_TEMPERATURE,
  // PROCESS_STATE_READ_FREQUENCY,
  // PROCESS_STATE_READ_IPEAK,
  // PROCESS_STATE_READ_VPEAK,
  // PROCESS_STATE_UPDATE_INFO
} process_state_t;

enum sy7t609_reply_code {
  REPLY_ACK_WITH_DATA = 0xAA,
  REPLY_ACK_WITHOUT_DATA = 0xAD
  // REPLY_AUTO_REPORTING_HEADER = 0xAE,  //自动报告
  // REPLY_NEGATIVE_ACK = 0xB0,
  // REPLY_COMMAND_NOT_IMPLEMENTED = 0xBC,
  // REPLY_CHECKSUM_FAILED = 0xBD,
  // REPLY_BUFFER_OVERFLOW = 0xBF
};
//2162720 校正值 0x210520 

// static sy7t609_info_t s_info;
static uint8_t getChecksumForCommand(const uint8_t* data, size_t size);
static process_state_t s_process_state = PROCESS_STATE_READ_1;

// static void updateInfo(sy7t609_info_t info);
static bool resetSy7t609(void);
// static bool setControlRegister(void);
// static bool setScale(void);
// static bool setBucket(void);
// static bool setAccum(void);
// static bool clearEnergyCounters(void);
static uint32_t readPF(void);
static uint32_t readVRMS(void);
static uint32_t readIRMS(void);
static uint32_t readPOWER(void);
static uint32_t readvarPower(void);
static uint32_t readEPPCNT(void);
// static uint32_t readEPMCNT(void);
static uint32_t readTemperature(void);
static uint32_t readFREQUENCY(void);
// static uint32_t readIPEAK(void);
// static uint32_t readVPEAK(void);

// sy7t609_info_t getSy7t609Info(void) {
  // sy7t609MeasurementProcess();
  // return s_info;
// }

void initSy7t609(void) {
  resetSy7t609();
  // delay(10);
  // setBucket();
  //clearEnergyCounters();
  // delay(10);
  // setAutoReporting(true);
}

void sy7t609MeasurementProcess(sy7t609_info_t *s_info) {
  switch (s_process_state) {
    case PROCESS_STATE_READ_1:
	  s_info->vrms = readVRMS();
	  s_info->irms = readIRMS();
      s_process_state = PROCESS_STATE_READ_2;
      break;
    case PROCESS_STATE_READ_2:
	  s_info->power = readPOWER();
	  s_info->var_power = readvarPower();
      s_process_state = PROCESS_STATE_READ_3;
      break;
    case PROCESS_STATE_READ_3:
      s_info->pf = readPF();
      s_info->frequency = readFREQUENCY();
      s_process_state = PROCESS_STATE_READ_4;
      break;
    case PROCESS_STATE_READ_4:
	  s_info->epp_cnt = readEPPCNT();
      s_info->ctemp = readTemperature();
	  // s_info->eem_cnt = readEPMCNT();
      // s_info->ipeak = readIPEAK();
      s_process_state = PROCESS_STATE_READ_1;
      break;
    // case PROCESS_STATE_READ_5:
      // s_info->ctemp = readTemperature();
      // s_process_state = PROCESS_STATE_READ_1;
      // break;
    default:
      s_process_state = PROCESS_STATE_READ_1;
      break;
  }
}

static uint8_t getChecksumForCommand(const uint8_t* data, size_t size) {
  uint8_t checksum = 0;
  // Calculate checksum for payload
  for (size_t i = 0; i < size - 1; ++i) {
    checksum += data[i];
  }
  // Calculate two's complement checksum
  checksum = ~checksum + 1;
  return checksum;
}

bool readRegister(uint16_t addr, uint32_t* out_value) {
  uint8_t checksum = 0;
  uint8_t reply_buffer[6];
  uint8_t reply_index = 0;
  uint8_t data[7];
if (t609_error_count < 80) {
  data[0] = SSI_HEADER;
  data[1] = SSI_DEFAULT_FRAME_SIZE + SSI_READ_PAYLOAD_SIZE;
  data[2] = CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = CMD_READ_REGISTER_3BYTES;
  data[6] = getChecksumForCommand(data,sizeof(data));
  
  // Flush RX buffer
  Serial.flush();
  Serial.write(data, sizeof(data));
  // Wait for reply
  unsigned long start_time = millis();
  while (millis() - start_time < 100) {
    if (Serial.available() > 0) {
      // Read reply
      reply_buffer[reply_index++] = Serial.read();
      if (reply_index >= sizeof(reply_buffer)) {
        // Reply received, break the loop
        break;
      }
    }
  }

// Check if enough data received
  if (reply_index < sizeof(reply_buffer)) {
  // Serial.print("SHIBAI");
  t609_error_count++;
    return false;
  }

  // Read reply into buffer
  if (reply_buffer[0] != REPLY_ACK_WITH_DATA) {
    return false;
  }
t609_error_count = 0;
  // 检查校验和
  // Serial.println("Before memcpy:");
  // Serial.print("reply_buffer: ");
  // for (int i = 0; i < sizeof(reply_buffer); i++) {
  // Serial.print(reply_buffer[i], HEX);
  // Serial.print(" ");
  // }
  // Serial.println();
  checksum = getChecksumForCommand(reply_buffer,sizeof(reply_buffer));
  // Serial.println(checksum, HEX);
  if (reply_buffer[5] != checksum) {
    // 校验和错误
    //Serial.println("Checksum Error");
    return false;
  }
}
  *out_value = ((uint32_t)reply_buffer[4] << 16) | ((uint32_t)reply_buffer[3] << 8) | reply_buffer[2];
  return true;
}

bool writeRegister(uint16_t addr, uint32_t value) {
  uint8_t reply = 0;
  uint8_t data[10];
if (t609_error_count < 80) {
  data[0] = SSI_HEADER;
  data[1] = SSI_DEFAULT_FRAME_SIZE + SSI_WRITE_PAYLOAD_SIZE;
  data[2] = CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = CMD_WRITE_REGISTER_3BYTES;
  data[6] = value & 0xFF;
  data[7] = (value >> 8) & 0xFF;
  data[8] = (value >> 16) & 0xFF;
  data[9] = getChecksumForCommand(data,sizeof(data));
  // Flush RX buffer
  Serial.flush();

  Serial.write(data, sizeof(data));
  // Serial.flush();

  // Wait for reply
  unsigned long start_time = millis();
  while (millis() - start_time < 100) {
    if (Serial.available() > 0) {
      // Read reply
      reply = Serial.read();
      // Check reply code
      if (reply == REPLY_ACK_WITHOUT_DATA) {
        // Success
        return true;
      } else {
        return false;
      }
    }
  }
  // Timeout
}
  return false;
}

static uint32_t readPF() {
  uint32_t data;
  if (readRegister(ADDR_PF, &data) == false) {
    data = 0;
  } else {
    if (data >= 0x800000) {
      data = 0x01000000 - data;
    }
  }
  return data;
}

static uint32_t readVRMS() {
  uint32_t data;
  if (readRegister(ADDR_VRMS, &data) == false) {
    data = 0;
  }
  return data;
}

static uint32_t readIRMS() {
  uint32_t data;
  if (readRegister(ADDR_IRMS, &data) == false) {
    data = 0;
  // } else {
    // data = (uint32_t)(data / 128);
  }
  return data;
}

static uint32_t readPOWER(void) {
  uint32_t data;
  if (readRegister(ADDR_POWER, &data) == false) {
    data = 0;
  } else {
    if (data >= 0x800000) {
      data = 0x01000000 - data;
    }
  }
  return data;
}

static uint32_t readvarPower(void) {
  uint32_t data;
    if (readRegister(ADDR_VAR, &data) == false) {
    data = 0;
  } else {
    if (data >= 0x800000) {
      data = 0x01000000 - data;
    }
  }
  return data;
}

static uint32_t readEPPCNT(void) {
  uint32_t data;
  if (readRegister(ADDR_EPPCNT, &data) == false) {
    data = 0;
  }
  return data;
}
//反向功率
// static uint32_t readEPMCNT(void)
// {
    // uint32_t data;
    // if (readRegister(ADDR_EPMCNT, &data) == false) {
        // data = 0;
    // }
  // return data;
// }

// static uint32_t readIPEAK(void) {
  // uint32_t data;
  // if (readRegister(ADDR_IPEAK, &data) == false) {
    // data = 0;
  // }
 // return static_cast<float>(data) / 1000.0f;
// }

// static uint32_t readVPEAK(void) {
  // uint32_t data;
  // if (readRegister(ADDR_VPEAK, &data) == false) {
    // data = 0;
  // }
 // return static_cast<float>(data) / 1000.0f;
// }

static uint32_t readTemperature(void) {
  uint32_t data;
  if (readRegister(ADDR_CTEMP, &data) == false) {
    data = 0;
  }
  return data;
}

static uint32_t readFREQUENCY(void) {
  uint32_t data;

  if (readRegister(ADDR_FREQUENCY, &data) == false) {
    data = 0;
  }
  return data;
}

// static bool setControlRegister(void) {
  // if (writeRegister(ADDR_CONTROL, CONTROL) == false) {
    // return false;
  // }
  // return true;
// }

// static bool setScale(void) {
  // if (writeRegister(ADDR_VSCALE, VSCALE) == false) {
    // return false;
  // }

  // if (writeRegister(ADDR_ISCALE, ISCALE) == false) {
    // return false;
  // }

  // if (writeRegister(ADDR_PSCALE, PSCALE) == false) {
    // return false;
  // }

  // return true;
// }

// static bool setBucket(void) {
  // if (writeRegister(ADDR_BUCKETL, BUCKETL) == false) {
    // return false;
  // }

  // if (writeRegister(ADDR_BUCKETH, BUCKETH) == false) {
    // return false;
  // }
  // return true;
// }
//单次计数器
// static bool setAccum(void) {
  // if (writeRegister(ADDR_ACCUM, ACCUM) == false) {
    // return false;
  // }
  // return true;
// }
//重置统计数据
static bool clearEnergyCounters(void) {
  if (writeRegister(ADDR_COMMAND, CMD_REG_CLEAR_ENGERGY_COUNTERS) == false) {
    return false;
  }
  return true;
}
//执行软复位
static bool resetSy7t609(void) {
  t609_error_count = 0;
  if (writeRegister(ADDR_COMMAND, CMD_REG_SOFT_RESET) == false) {
    return false;
  }
  return true;
}

bool setAutoReporting(bool enable) {
  // 构建命令字节
  // uint8_t header = REPLY_AUTO_REPORTING_HEADER;
  uint32_t command = enable ? 0xAE0001 : 0xAE0000;
  // 调用写寄存器函数发送命令到设备
  if (writeRegister(ADDR_COMMAND, command) == false) {
    return false;
  }
  return true;
}
