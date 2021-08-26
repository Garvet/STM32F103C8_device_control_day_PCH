#include <Device_relay.h>

void activate_pin(struct relay_t* relay) {
    if (relay->state != relay->invert)
        HAL_GPIO_WritePin(relay->port, relay->pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(relay->port, relay->pin, GPIO_PIN_RESET);
}

struct relay_t relay_init(GPIO_TypeDef*  port, uint16_t pin, bool invert) {
    struct relay_t relay;
    relay.port = port;
    relay.pin = pin;
    relay.invert = invert;
    relay.state = false;
    activate_pin(&relay);
    return relay;
}

void relay_set_state(struct relay_t* relay, bool state) {
    relay->state = state;
    activate_pin(relay);
}

bool relay_toggle(struct relay_t* relay) {
    relay_set_state(relay, !(relay->state));
    return relay->state;
}

void relay_off(struct relay_t* relay) {
    relay_set_state(relay, false);
}

void relay_on(struct relay_t* relay) {
    relay_set_state(relay, true);
}

GPIO_PinState relay_pin_signal(struct relay_t* relay) {
    if(relay->state != relay->invert)
        return GPIO_PIN_SET;
    return GPIO_PIN_RESET;
}
