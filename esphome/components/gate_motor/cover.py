import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, cover
from esphome.const import CONF_ID

from . import gate_motor_ns

DEPENDENCIES = ["uart"]

CONF_POLL_INTERVAL = "poll_interval"

GateMotorCover = gate_motor_ns.class_(
    "GateMotorCover", cover.Cover, uart.UARTDevice, cg.Component
)

# cover.cover_schema() is the correct API in ESPHome 2022+ (replaces COVER_SCHEMA)
CONFIG_SCHEMA = cover.cover_schema(GateMotorCover).extend(
    {
        cv.Optional(CONF_POLL_INTERVAL, default="500ms"): cv.positive_time_period_milliseconds,
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
