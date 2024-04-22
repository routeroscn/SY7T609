#ifndef __SY7T609_UART_PROTOCOL_H__
#define __SY7T609_UART_PROTOCOL_H__

/*
* The SY7T609+S1 firmware implements a "simple serial interfac" (SSI)
* on the UART interface.
*/

/* Command-Response Mode
* In this protocol, the host is the master and must initiate communications.
* The master should first set the device's register address pointer berfore
* performing read or write operations.
*
* After sending the synchronization header code (0xAA), the master sends
* (in the following order) the byte counts (bytes in payload),
* the payload and then the checksum that provides data integrity check.
*
* # a generic command packet #
* | Header | Byte | Payload | Checksum |
* | (0xAA) | Count|         |          |
* 
*
* Register Address Pointer Selection
* |           PAYLOAD          |
* |  0xA3   | Register Address |
* | Command | (2 Bytes)        |
*
* Read Command
* # Case1 #                # Case2 #
* |    PAYLOAD    |        |              PAYLOAD                  |
* |     0xE3      |        |   0xE0    |          0x1F             |
* |    Command    |        |  Command  | (Number of Bytes to Read) |
* Case1) To read 0 to 15 bytes, the command byte is completed with
*        the number of bytes to read. (ex. to read 3 bytes)
* Case2) In order to read a larger number of bytes(up to 255), the command
*        0xE0 must be used. (ex. to read 31 bytes)
*
* Write Command
* # Case1 #
* |              PAYLOAD              |
* |   0xD3    |        Data           |
* |  Command  | (Number of Bytes = 3) |
*
* # Case2 #
* |                     PAYLOAD                    |
* |   0xD0    |                 Data               |
* |  Command  | (Number of Bytes = Byte Count - 4) |
* Case1) To read 0 to 15 bytes, the command byte is completed with
*        the number of bytes to write. (ex. to write 3 bytes)
* Case2) In order to read a larger number of bytes(up to 255), the command
*        0xD0 must be used. (ex. to read 31 bytes) 
*/

enum sy7t609_register_map {
    ADDR_COMMAND     = 0x0000,  // 命令寄存器
    // ADDR_FW_VER      = 0x0003,  // 固件版本寄存器
    ADDR_CONTROL     = 0x0006,  // 控制寄存器
    // 电能表寄存器
    ADDR_VAVG        = 0x002D,  // 电压平均值
    ADDR_IAVG        = 0x0030,  // 电流平均值
    ADDR_VRMS        = 0x0033,  // 电压有效值*
    ADDR_IRMS        = 0x0036,  // 电流有效值*
    ADDR_POWER       = 0x0039,  // 有功功率*
    ADDR_VAR         = 0x003C,  // 无功功率*
    ADDR_FREQUENCY   = 0x0042,  // 频率*
    // ADDR_AVG_POWER   = 0x0045,  // 平均有功功率
    ADDR_PF          = 0x0048,  // 功率因数*
    ADDR_EPPCNT      = 0x0069,  // 正向有功能量计数*
    // ADDR_EPMCNT      = 0x006C,  // 反向有功能量计数
    // ADDR_VA          = 0x003F,  // 有功功率
    ADDR_CTEMP       = 0x0027,  // 芯片温度*
    ADDR_IPEAK       = 0x008A,  // 电流峰值*
    ADDR_VPEAK       = 0x0093,  // 电压峰值*
    // 输入/输出控制寄存器
    // ADDR_DIO_DIR     = 0x0099,  // 数字输入/输出方向
    // ADDR_DIO_SET     = 0x009F,  // 数字输入/输出设置
    // ADDR_DIO_RST     = 0x00A2,  // 数字输入/输出复位
    // 校准寄存器
     ADDR_BUCKETL     = 0x00C0,  // 电表低16位寄存器
     ADDR_BUCKETH     = 0x00C3,  // 电表高8位寄存器
    // ADDR_IGAIN       = 0x00D5,  // 电流增益
    ADDR_VGAIN       = 0x00D8,  // 电压增益
    // ADDR_ISCALE      = 0x00ED,  // 电流刻度
    // ADDR_VSCALE      = 0x00F0,  // 电压刻度
    // ADDR_PSCALE      = 0x00F3,  // 功率刻度
    ADDR_ACCUM       = 0x0105,  // 累计间隔寄存器
    ADDR_IRMS_TARGET = 0x0117,  // 电流有效值目标 校准电压用的当前基准值
    ADDR_VRMS_TARGET = 0x011A   // 电压有效值目标
};


enum command_register_code {
    CMD_REG_CLEAR_ENGERGY_COUNTERS = 0xEC0000, // 清除所有能量计数器
    CMD_REG_SOFT_RESET = 0xBD0000, // 执行软复位 仅重置程序计数器而不重置硬件
    CMD_REG_SAVE_TO_FLASH = 0xACC200, // 保存到闪存 保存校准到闪存
    CMD_REG_AUTO_REPORTING = 0xAE0001, // 开启自动报告
    CMD_REG_CLEAR_FLASH_STORAGE_0 = 0xACC000, // 清除闪存存储 0
    CMD_REG_CLEAR_FLASH_STORAGE_1 = 0XACC100, // 清除闪存存储 1
    CMD_REG_CALIBRATION_VOLTAGE = 0xCA0020, // 电压校准
    CMD_REG_CALIBRATION_CURRENT = 0xCA0010, // 电流校准
    CMD_REG_CALIBRATION_ALL = 0xCA0030 // 全部校准
}; 


typedef struct sy7t609_info {
    uint32_t vrms;           // 有效电压
    uint32_t irms;           // 有效电流
    uint32_t pf;             // 功率因素
    uint32_t power;          // 有功功率
    uint32_t var_power;      // 无功功率
    uint32_t epp_cnt;        // 正向有功能量计数
    // uint32_t eem_cnt;        // 反向有功能量计数 这个肯定没用了
    uint32_t frequency;      // 频率
    uint32_t ctemp;          // 温度
    // uint32_t ipeak;          // 电流峰值
    // uint32_t vpeak;          // 电压峰值 没什么卵用
} sy7t609_info_t;


// #define COMMAND_REGISTER_CALIBRATION_MASK (0xFF0000)
// #define CONTROL_REGISTER_MASK             (0x001815)
bool readRegitser(uint16_t addr, uint32_t* value);
bool writeRegister(uint16_t addr, uint32_t value);
void initSy7t609(void);
// bool setAutoReporting(bool enableAutoReporting);
void sy7t609MeasurementProcess(sy7t609_info_t *s_info);
#endif //__SY7T609_UART_PROTOCOL_H__