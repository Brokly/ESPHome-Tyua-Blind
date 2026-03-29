import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import cover, uart, text_sensor, select, switch, button 
from esphome import automation, pins
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_DEVICE_CLASS,
    CONF_ICON,
    CONF_NAME,
    ENTITY_CATEGORY_CONFIG,
)

CODEOWNERS = ["@Brokly"]
DEPENDENCIES = ["cover", "uart"]
AUTO_LOAD = ["cover","text_sensor", "select", "switch", "button"]

CONF_PRODUCT_ID = 'product_id'
ICON_PRODUCT_ID = 'mdi:cog'
CONF_SHOW_PROTCOL = 'protocol_debug_level'
CONF_REFRESH_POS = 'refresh_position'
CONF_REVERSED = 'motor_reversed'
CONF_SAVE_TIMEOUT = 'save_timeout'
CONF_REFRESH_TIMEOUT = 'reftesh_timeout'
CONF_BUTTON_LIM_UP = 'button_limit_open'
ICON_OPEN = 'mdi:arrow-collapse-up'
CONF_BUTTON_LIM_DOWN = 'button_limit_close'
ICON_CLOSE = 'mdi:arrow-collapse-down'
CONF_BUTTON_LIM_RESET = 'button_limits_reset'
ICON_RESET = 'mdi:arrow-expand-all'
CONF_SELECT_SPEED = 'motor_speed_selector'
ICON_SPEED = 'mdi:car-speed-limiter'
CONF_CHILDREN_LOCK = 'children_lock'
ICON_CHILDREN_LOCK = 'mdi:lock'
CONF_MOTOR_SPEED = 'motor_speed'
CONF_ATOM_MODE = 'atom_switch'
ICON_ATOM_MODE = 'mdi:format-line-spacing'


tuya_blind_ns = cg.esphome_ns.namespace("tuya_blind")
TuyaBlind = tuya_blind_ns.class_("TuyaBlind", cover.Cover, cg.Component)
TuyaBlindButton =tuya_blind_ns.class_("TuyaBlind_Button", button.Button, cg.Component)
TuyaBlindSelect = tuya_blind_ns.class_("TuyaBlind_Select", select.Select, cg.Component)
TuyaBlindSwitch = tuya_blind_ns.class_("TuyaBlind_Switch", switch.Switch, cg.Component)

LOG_LEVEL_SHOW_PROTO = {
    "NONE": 0,
    "ERROR": 1,
    "WARN": 2,
    "INFO": 3,
    "CONFIG": 4,
    "DEBUG": 5,
    "VERBOSE": 6,
    "VERY_VERBOSE": 7,
}

SPEED_VALUES = {
    "LOW": 0,
    "MIDDLE": 1,
    "HIGH": 2,
}

def validate_speed(config):
    if CONF_SELECT_SPEED in config and CONF_MOTOR_SPEED in config:
        raise cv.Invalid(f"Options 'motor_speed' and 'motor_speed_selector' cannot be used simultaneously.")
    return config

def validate_refresh(config):
    if CONF_REFRESH_POS in config:
       refr=config[CONF_REFRESH_POS]
       if CONF_REFRESH_TIMEOUT in refr:
          val=refr[CONF_REFRESH_TIMEOUT]
          if val < cv.time_period("50ms"):
             raise cv.Invalid(f"Invalid 'reftesh_timeout' value cannot be less than 50ms ")
    return config

def validate_store(config):
    if CONF_REFRESH_POS in config:
       refr=config[CONF_REFRESH_POS]
       if CONF_SAVE_TIMEOUT in refr:
          val=refr[CONF_SAVE_TIMEOUT]
          if val < cv.time_period("1000ms"):
             raise cv.Invalid(f"Invalid 'save_timeout' value cannot be less than 1s ")
    return config



CONFIG_SCHEMA = cv.All(
#    cover.COVER_SCHEMA.extend(
    cover.cover_schema(cover.Cover).extend(
        {
            cv.GenerateID(): cv.declare_id(TuyaBlind),
            cv.Required(CONF_BUTTON_LIM_UP):button.button_schema( 
                TuyaBlindButton,
                icon = ICON_OPEN,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Required(CONF_BUTTON_LIM_DOWN):button.button_schema( 
                TuyaBlindButton,
                icon=ICON_CLOSE,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Required(CONF_BUTTON_LIM_RESET):button.button_schema( 
                TuyaBlindButton,
                icon=ICON_RESET,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_SELECT_SPEED):select.select_schema( 
                TuyaBlindSelect,
                icon=ICON_SPEED,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_CHILDREN_LOCK):switch.switch_schema( 
                TuyaBlindSwitch,
                icon=ICON_CHILDREN_LOCK,
            ),
            cv.Optional(CONF_ATOM_MODE):switch.switch_schema( 
                TuyaBlindSwitch,
                icon=ICON_ATOM_MODE,
            ),
            cv.Optional(CONF_REVERSED, default=False): cv.boolean,
            cv.Optional(CONF_REFRESH_POS): cv.Schema(
              {
                cv.Optional(CONF_REFRESH_TIMEOUT, default=100): cv.positive_time_period_milliseconds,
                cv.Optional(CONF_SAVE_TIMEOUT): cv.positive_time_period_milliseconds,
              }
            ),
            cv.Optional(CONF_SHOW_PROTCOL): cv.enum(LOG_LEVEL_SHOW_PROTO, upper=True),
            cv.Optional(CONF_MOTOR_SPEED): cv.enum(SPEED_VALUES, upper=True),
            cv.Optional(CONF_PRODUCT_ID): text_sensor.text_sensor_schema(icon=ICON_PRODUCT_ID),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    validate_speed,
    validate_refresh,
    validate_store,
)

async def to_code(config):

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)
    cg.add(var.set_limits_buttons(
       await button.new_button(config[CONF_BUTTON_LIM_UP]),
       await button.new_button(config[CONF_BUTTON_LIM_DOWN]),
       await button.new_button(config[CONF_BUTTON_LIM_RESET])))

    if CONF_UART_ID in config:
       parent = await cg.get_variable(config[CONF_UART_ID])
       cg.add(var.initBlind(parent))
    else:
       raise cv.Invalid(
          f"Setting 'uart_id' is required !"
       )

    if CONF_SELECT_SPEED in config:
        cg.add(var.set_speed_select(await select.new_select(config[CONF_SELECT_SPEED], options=[])))
    elif CONF_MOTOR_SPEED in config:
        cg.add_define("TBLIND_MOTOR_SPEED",config[CONF_MOTOR_SPEED])

    if CONF_PRODUCT_ID in config:
        cg.add(var.set_product_id_text(await text_sensor.new_text_sensor(config[CONF_PRODUCT_ID])))

    if CONF_REFRESH_POS in config:
        refrpos=config[CONF_REFRESH_POS]
        if CONF_REFRESH_TIMEOUT in refrpos:
           cg.add_define("TBLIND_VIRTUAL_POS",refrpos[CONF_REFRESH_TIMEOUT])
        if CONF_SAVE_TIMEOUT in refrpos:
           cg.add_define("TBLIND_RESTORE",refrpos[CONF_SAVE_TIMEOUT])
    
    if CONF_CHILDREN_LOCK in config:
        cg.add_define("TBLIND_CHILDREN_LOCK")
        cg.add(var.set_children_lock_switch(await switch.new_switch(config[CONF_CHILDREN_LOCK])))

    if CONF_ATOM_MODE in config:
        cg.add_define("TBLIND_ATOM")
        cg.add(var.set_atom_switch(await switch.new_switch(config[CONF_ATOM_MODE])))

    if CONF_REVERSED in config:
        if config[CONF_REVERSED]:
           cg.add_define("TBLIND_MOTOR_REVERS",1)
        else:
           cg.add_define("TBLIND_MOTOR_REVERS",0)
  
    if CONF_SHOW_PROTCOL in config:
        cg.add_define("TBLIND_PRINT_RAW_PROTO", config[CONF_SHOW_PROTCOL])
        
        
