#include <Arduino.h>
#include <EEPROM.h>

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"
#include "soc/timer_group_reg.h"
// #include "packages/esp32/hardware/esp32/$VERSION/tools/sdk/esp32/include/soc/esp32/include/soc/timer_group_struct.h"

/*
 *  Use biped namespace.
 */


// Timer constants
#define C1_FREQ 200
#define C2_FREQ 25
#define DIVIDER_1 40000
#define DIVIDER_2 40000
#define COUNTER_1 10
#define COUNTER_2 80

using namespace biped;

unsigned int id;

// Pin values
bool sda_pin = false;
bool scl_pin = false;

// Timer handles
hw_timer_t* timer1;
hw_timer_t* timer2;

// Interrupt Counters for testing
int counter1 = 0;
int counter2 = 0;
int main_counter = 0;

// TODO Timer ISRs
void timer1_isr() {
  // TODO: handle interrupt
  gpio_set_level((gpio_num_t)ESP32Pin::i2c_scl, scl_pin = !scl_pin);
  counter1 ++;
  delayMicroseconds(random(1000, 2000));
  *((volatile uint32_t*)TIMG_INT_CLR_TIMERS_REG(1)) |= TIMG_T0_INT_CLR;
  *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) |= TIMG_T0_ALARM_EN;
}


void timer2_isr() {
  // TODO: handle interrupt
  gpio_set_level((gpio_num_t)ESP32Pin::i2c_sda, sda_pin = !sda_pin);
  counter2 ++;
  delayMicroseconds(random(1000, 2000));
  *((volatile uint32_t*)TIMG_INT_CLR_TIMERS_REG(1)) |= TIMG_T1_INT_CLR;
  *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) |= TIMG_T1_ALARM_EN;
}


void setup() {
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));
    // Initialize Serial
    Serial::initialize();

    // Initialize GPIOs
    gpio_set_direction((gpio_num_t)ESP32Pin::i2c_scl, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)ESP32Pin::i2c_sda, GPIO_MODE_OUTPUT);
    
    // Initialize timers
    // Disable interrupts
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) &= ~TIMG_T0_EN;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) &= ~TIMG_T1_EN;
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) &= ~TIMG_T0_LEVEL_INT_EN;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) &= ~TIMG_T1_LEVEL_INT_EN;
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) &= ~TIMG_T0_ALARM_EN;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) &= ~TIMG_T1_ALARM_EN;

    // Set dividers
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) &= ~TIMG_T0_DIVIDER_M;
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) |= DIVIDER_1 << TIMG_T0_DIVIDER_S;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) &= ~TIMG_T1_DIVIDER_M;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) |= DIVIDER_2 << TIMG_T1_DIVIDER_S;

    // Set timer to count up
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) |= TIMG_T0_INCREASE;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) |= TIMG_T1_INCREASE;

    // Set auto-reload on alarm
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) |= TIMG_T0_AUTORELOAD;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) |= TIMG_T1_AUTORELOAD; 

    // Set alarm register value
    *((volatile uint32_t*)TIMG_T0ALARMLO_REG(1)) = COUNTER_1;
    *((volatile uint32_t*)TIMG_T1ALARMLO_REG(1)) = COUNTER_2;
    *((volatile uint32_t*)TIMG_T0ALARMHI_REG(1)) = 0x00;
    *((volatile uint32_t*)TIMG_T1ALARMHI_REG(1)) = 0x00;

    // Attach interrupts
    esp_intr_alloc(ETS_TG1_T0_LEVEL_INTR_SOURCE, 0, (intr_handler_t)timer1_isr, NULL, NULL);
    esp_intr_alloc(ETS_TG1_T1_LEVEL_INTR_SOURCE, 0, (intr_handler_t)timer2_isr, NULL, NULL);

    *((volatile uint32_t*)TIMG_T0LOADLO_REG(1)) = 0x00;
    *((volatile uint32_t*)TIMG_T1LOADLO_REG(1)) = 0x00;
    *((volatile uint32_t*)TIMG_T0LOADHI_REG(1)) = 0x00;
    *((volatile uint32_t*)TIMG_T1LOADHI_REG(1)) = 0x00;

    // Enable interrupts
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) |= TIMG_T0_ALARM_EN;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) |= TIMG_T1_ALARM_EN;
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) |= TIMG_T0_EN;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) |= TIMG_T1_EN;
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) |= TIMG_T0_LEVEL_INT_EN;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) |= TIMG_T1_LEVEL_INT_EN;
    *((volatile uint32_t*)TIMG_T0CONFIG_REG(1)) &= ~TIMG_T0_EDGE_INT_EN;
    *((volatile uint32_t*)TIMG_T1CONFIG_REG(1)) &= ~TIMG_T1_EDGE_INT_EN;
}


void loop() {
    delay(1000);
    main_counter ++;
    biped::Serial(LogLevel::info) << "Loop number " << main_counter << "\n";
    biped::Serial(LogLevel::info) << "Counter 1 value: " << counter1 << "\n";
    biped::Serial(LogLevel::info) << "Counter 2 value: " << counter2 << "\n\n";

    // Reset counters
    counter1 = 0;
    counter2 = 0;
}
