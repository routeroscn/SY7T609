import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import maybe_simple_id
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_POWER_FACTOR,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_POWER,
    CONF_REACTIVE_POWER,
    CONF_ENERGY,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_LIGHTBULB,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_VOLT_AMPS_REACTIVE,
    UNIT_WATT_HOURS,
    UNIT_CELSIUS,
)

# 常量定义
CONF_CHIP_TEMPERATURE = "chip_temperature"

DEPENDENCIES = ["uart"]

# 定义命名空间和核心类
sy7t609_ns = cg.esphome_ns.namespace("sy7t609")
SY7T609 = sy7t609_ns.class_("SY7T609_UART", cg.PollingComponent, uart.UARTDevice)

# 定义动作
ResetEnergyAction = sy7t609_ns.class_("ResetEnergyAction", automation.Action)
ResetCalibrationAction = sy7t609_ns.class_("ResetCalibrationAction", automation.Action)
CalibrateVoltageAction = sy7t609_ns.class_("CalibrateVoltageAction", automation.Action)
CalibrateCurrentAction = sy7t609_ns.class_("CalibrateCurrentAction", automation.Action)
CalibrateGenericAction = sy7t609_ns.class_("CalibrateGenericAction", automation.Action)
SaveCalibrationAction = sy7t609_ns.class_("SaveCalibrationAction", automation.Action)

# 校准动作用到的配置键
CONF_VALUE = "value"
CONF_COMMAND = "command"
CONF_TARGET_REGISTER = "target_register"

# 传感器键 -> setter 名称（新增读取数据时在此追加一行即可）
SENSOR_SETTERS = {
    CONF_POWER_FACTOR: "set_power_factor_sensor",
    CONF_VOLTAGE: "set_voltage_sensor",
    CONF_CURRENT: "set_current_sensor",
    CONF_POWER: "set_power_sensor",
    CONF_REACTIVE_POWER: "set_power_reactive_sensor",
    CONF_ENERGY: "set_energy_sensor",
    CONF_CHIP_TEMPERATURE: "set_temperature_sensor",
}

# 定义 Schema
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SY7T609),
            cv.Optional(CONF_POWER_FACTOR): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_POWER_FACTOR,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_REACTIVE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE,
                icon=ICON_LIGHTBULB,
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_CHIP_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


# ----------------------------------------------------------------------------
# 无参动作（复用模式）：reset_energy / reset_calibration
# ----------------------------------------------------------------------------
def register_simple_action(action_name, action_type):
    @automation.register_action(
        action_name,
        action_type,
        maybe_simple_id(
            {
                cv.Required(CONF_ID): cv.use_id(SY7T609),
            }
        ),
        synchronous=False,  # 实际执行被推迟到 loop() 的回调队列中
    )
    async def _to_code(config, action_id, template_arg, args):
        parent = await cg.get_variable(config[CONF_ID])
        return cg.new_Pvariable(action_id, template_arg, parent)


register_simple_action("sy7t609.reset_energy", ResetEnergyAction)
register_simple_action("sy7t609.reset_calibration", ResetCalibrationAction)
register_simple_action("sy7t609.save_calibration", SaveCalibrationAction)


# ----------------------------------------------------------------------------
# 单值校准动作：calibrate_voltage / calibrate_current
# 用法：value 传入"当前实际值"（电压 V / 电流 A，支持模板/lambda）。
# ----------------------------------------------------------------------------
def register_single_value_calibration(action_name, action_type):
    @automation.register_action(
        action_name,
        action_type,
        cv.Schema(
            {
                cv.Required(CONF_ID): cv.use_id(SY7T609),
                cv.Required(CONF_VALUE): cv.templatable(cv.positive_float),
            }
        ),
        synchronous=False,
    )
    async def _to_code(config, action_id, template_arg, args):
        parent = await cg.get_variable(config[CONF_ID])
        var = cg.new_Pvariable(action_id, template_arg, parent)
        templ = await cg.templatable(config[CONF_VALUE], args, float)
        cg.add(var.set_value(templ))
        return var


register_single_value_calibration("sy7t609.calibrate_voltage", CalibrateVoltageAction)
register_single_value_calibration("sy7t609.calibrate_current", CalibrateCurrentAction)


# ----------------------------------------------------------------------------
# 通用校准动作：sy7t609.calibrate
# 命令码 / 目标寄存器 / 写入值均由用户按数据手册指定，用于功率、相位补偿等高级校准。
# ----------------------------------------------------------------------------
@automation.register_action(
    "sy7t609.calibrate",
    CalibrateGenericAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(SY7T609),
            cv.Required(CONF_COMMAND): cv.templatable(cv.hex_uint32_t),
            cv.Optional(CONF_TARGET_REGISTER, default=0x0FFF): cv.templatable(
                cv.hex_uint32_t
            ),
            cv.Optional(CONF_VALUE, default=0): cv.templatable(cv.hex_uint32_t),
        }
    ),
    synchronous=False,
)
async def calibrate_generic_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cmd = await cg.templatable(config[CONF_COMMAND], args, cg.uint32)
    cg.add(var.set_command(cmd))
    reg = await cg.templatable(config[CONF_TARGET_REGISTER], args, cg.uint32)
    cg.add(var.set_target_register(reg))
    val = await cg.templatable(config[CONF_VALUE], args, cg.uint32)
    cg.add(var.set_value(val))
    return var


# ----------------------------------------------------------------------------
# to_code 主逻辑
# ----------------------------------------------------------------------------
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # 传感器映射
    for key, setter in SENSOR_SETTERS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))
