import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_MR60BHA2_ID, MR60BHA2Component

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_MR60BHA2_ID): cv.use_id(MR60BHA2Component),
    cv.Optional("frames"): text_sensor.text_sensor_schema(),
    cv.Optional("health"): text_sensor.text_sensor_schema(),
})


async def to_code(config: dict) -> None:
    component = await cg.get_variable(config[CONF_MR60BHA2_ID])
    if "frames" in config:
        cg.add(component.set_frames(await text_sensor.new_text_sensor(config["frames"])))
    if "health" in config:
        cg.add(component.set_health(await text_sensor.new_text_sensor(config["health"])))
