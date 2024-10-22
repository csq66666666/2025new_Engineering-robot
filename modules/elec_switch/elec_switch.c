#include "elec_switch.h"
#include "memory.h"
#include <stdlib.h>

static uint8_t idx;
static ElecSwitchInstance *elec_switch_instance[ELECSWITCH_MAX_NUM] = {NULL};

ElecSwitchInstance *ElecSwitchInit(ElecSwitch_Init_Config_s *_config)
{
    ElecSwitchInstance *elec_switch_ins = (ElecSwitchInstance *)malloc(sizeof(ElecSwitchInstance));
    memset(elec_switch_ins, 0, sizeof(ElecSwitchInstance));

    // 先将GPIO初始化
    GPIO_Init_s gpio_init = {
        .GPIOx = _config->GPIOx,
        .pin_init_state = _config->trigger_level ? GPIO_PIN_RESET : GPIO_PIN_SET,
        .GPIO_InitStruct = {
            .Pin = _config->GPIO_Pin,
            .Mode = _config->GPIO_Mode,
            .Pull = _config->trigger_level ? GPIO_PULLDOWN : GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_LOW,
        },
    };
    GPIOInit(&gpio_init);
    // 注册GPIO
    GPIO_Init_Config_s gpio_config = {
        .GPIOx = _config->GPIOx,
        .GPIO_Pin = _config->GPIO_Pin,
        .id = elec_switch_ins,
    };
    elec_switch_ins->gpio_ins = GPIORegister(&gpio_config);
    elec_switch_ins->trigger_level = _config->trigger_level;

    elec_switch_instance[idx++] = elec_switch_ins;
    return elec_switch_ins;
}

void ElecSwitchSet(ElecSwitchInstance *_ins)
{
    _ins->trigger_level ? GPIOSet(_ins->gpio_ins) : GPIOReset(_ins->gpio_ins);
}




void ElecSwitchReset(ElecSwitchInstance *_ins)
{
    _ins->trigger_level ? GPIOReset(_ins->gpio_ins) : GPIOSet(_ins->gpio_ins);
}

GPIO_PinState ElecSwitchRead(ElecSwitchInstance *_ins)
{
    return GPIORead(_ins->gpio_ins);
}
