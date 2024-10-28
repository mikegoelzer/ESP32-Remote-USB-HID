#ifndef GPIOS_H
#define GPIOS_H

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
///  GPIOs
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//
// Output pins
//

// Signals to Motherboard
// Defualts:  pin 26 output active low => power on/off
//            pin 27 output active low => reset
#define GPIO_OUTPUT_MOBO_POWER       CONFIG_GPIO_OUTPUT_MOBO_POWER
#define GPIO_OUTPUT_MOBO_RESET       CONFIG_GPIO_OUTPUT_MOBO_RESET
#define GPIO_OUTPUT_PIN_SEL_MOBO     ((1ULL<<GPIO_OUTPUT_MOBO_POWER) | (1ULL<<GPIO_OUTPUT_MOBO_RESET))

// Signals to status LEDs:
//   AMBER = mobo reset (pin 0)
//   GREEN = system ready (pin 2)
#define GPIO_OUTPUT_AMBER             CONFIG_GPIO_OUTPUT_RGB_AMBER
#define GPIO_OUTPUT_GREEN             CONFIG_GPIO_OUTPUT_RGB_GREEN
#define GPIO_OUTPUT_PIN_SEL_LEDS      ((1ULL<<GPIO_OUTPUT_AMBER) | (1ULL<<GPIO_OUTPUT_GREEN))

//
// Input pins
//

// Input read off of motherboard
// Default:  pin 32 input active high => HDD activity on/off
//           (HDD activity is used to detect if the system is running)
#define GPIO_INPUT_HDD_ACTIVE                   CONFIG_GPIO_INPUT_HDD_ACTIVE
#define GPIO_INPUT_SENSING_HDD_ACTIVE_PIN_SEL   (1ULL<<GPIO_INPUT_HDD_ACTIVE)

// Buttons for manual triggering of Teensy
// Default:  pin 14 - OK button
//           pin 11 - power button
//           pin 10 - reset button
#define GPIO_INPUT_BTN_OK            CONFIG_GPIO_INPUT_BTN_OK
#define GPIO_INPUT_BTN_POWER         CONFIG_GPIO_INPUT_BTN_POWER
#define GPIO_INPUT_BTN_RESET         CONFIG_GPIO_INPUT_BTN_RESET
#define GPIO_INPUT_PIN_SEL           ((1ULL<<GPIO_INPUT_BTN_OK) | (1ULL<<GPIO_INPUT_BTN_POWER) | (1ULL<<GPIO_INPUT_BTN_RESET))

#endif // GPIOS_H