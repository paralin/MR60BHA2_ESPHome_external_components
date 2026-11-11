import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISTANCE,
    DEVICE_CLASS_DISTANCE,
    ICON_HEART_PULSE,
    ICON_PULSE,
    ICON_SIGNAL,
    ICON_COUNTER,
    STATE_CLASS_MEASUREMENT,
    UNIT_BEATS_PER_MINUTE,
    UNIT_CENTIMETER,
)

from . import CONF_MR60BHA2_ID, MR60BHA2Component

DEPENDENCIES = ["seeed_mr60bha2"]

CONF_BREATH_RATE = "breath_rate"
CONF_HEART_RATE = "heart_rate"
CONF_NUM_TARGETS = "num_targets"
CONF_TOTAL_PHASE = "total_phase"
CONF_BREATH_PHASE = "breath_phase"
CONF_HEART_PHASE = "heart_phase"
CONF_TARGET_X = "target_x"
CONF_TARGET_Y = "target_y"
CONF_TARGET_DOPPLER = "target_doppler"
CONF_MOTION_ENERGY = "motion_energy"

UNIT_RADIANS = "rad"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MR60BHA2_ID): cv.use_id(MR60BHA2Component),
        cv.Optional(CONF_BREATH_RATE): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_PULSE,
        ),
        cv.Optional(CONF_HEART_RATE): sensor.sensor_schema(
            accuracy_decimals=0,
            icon=ICON_HEART_PULSE,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_BEATS_PER_MINUTE,
        ),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_CENTIMETER,
            accuracy_decimals=2,
            icon=ICON_SIGNAL,
        ),
        cv.Optional(CONF_NUM_TARGETS): sensor.sensor_schema(
            icon=ICON_COUNTER,
        ),
        # Extended: raw displacement phases
        cv.Optional(CONF_TOTAL_PHASE): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_RADIANS,
            accuracy_decimals=4,
            icon="mdi:waveform",
        ),
        cv.Optional(CONF_BREATH_PHASE): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_RADIANS,
            accuracy_decimals=4,
            icon="mdi:lungs",
        ),
        cv.Optional(CONF_HEART_PHASE): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_RADIANS,
            accuracy_decimals=4,
            icon="mdi:heart-pulse",
        ),
        # Extended: point-cloud first-target position and Doppler
        cv.Optional(CONF_TARGET_X): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=3,
            icon="mdi:axis-x-arrow",
        ),
        cv.Optional(CONF_TARGET_Y): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=3,
            icon="mdi:axis-y-arrow",
        ),
        cv.Optional(CONF_TARGET_DOPPLER): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=0,
            icon="mdi:speedometer",
        ),
        # Extended: derived motion energy
        cv.Optional(CONF_MOTION_ENERGY): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=5,
            icon="mdi:vibrate",
        ),
    }
)


async def to_code(config):
    mr60bha2_component = await cg.get_variable(config[CONF_MR60BHA2_ID])
    if breath_rate_config := config.get(CONF_BREATH_RATE):
        sens = await sensor.new_sensor(breath_rate_config)
        cg.add(mr60bha2_component.set_breath_rate_sensor(sens))
    if heart_rate_config := config.get(CONF_HEART_RATE):
        sens = await sensor.new_sensor(heart_rate_config)
        cg.add(mr60bha2_component.set_heart_rate_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE):
        sens = await sensor.new_sensor(distance_config)
        cg.add(mr60bha2_component.set_distance_sensor(sens))
    if num_targets_config := config.get(CONF_NUM_TARGETS):
        sens = await sensor.new_sensor(num_targets_config)
        cg.add(mr60bha2_component.set_num_targets_sensor(sens))
    if total_phase_config := config.get(CONF_TOTAL_PHASE):
        sens = await sensor.new_sensor(total_phase_config)
        cg.add(mr60bha2_component.set_total_phase_sensor(sens))
    if breath_phase_config := config.get(CONF_BREATH_PHASE):
        sens = await sensor.new_sensor(breath_phase_config)
        cg.add(mr60bha2_component.set_breath_phase_sensor(sens))
    if heart_phase_config := config.get(CONF_HEART_PHASE):
        sens = await sensor.new_sensor(heart_phase_config)
        cg.add(mr60bha2_component.set_heart_phase_sensor(sens))
    if target_x_config := config.get(CONF_TARGET_X):
        sens = await sensor.new_sensor(target_x_config)
        cg.add(mr60bha2_component.set_target_x_sensor(sens))
    if target_y_config := config.get(CONF_TARGET_Y):
        sens = await sensor.new_sensor(target_y_config)
        cg.add(mr60bha2_component.set_target_y_sensor(sens))
    if target_doppler_config := config.get(CONF_TARGET_DOPPLER):
        sens = await sensor.new_sensor(target_doppler_config)
        cg.add(mr60bha2_component.set_target_doppler_sensor(sens))
    if motion_energy_config := config.get(CONF_MOTION_ENERGY):
        sens = await sensor.new_sensor(motion_energy_config)
        cg.add(mr60bha2_component.set_motion_energy_sensor(sens))
