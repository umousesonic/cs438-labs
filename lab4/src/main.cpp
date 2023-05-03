#include <Arduino.h>
#include <string>
#include <EEPROM.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "driver/gpio.h"

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"

#include "MCP23018.h"


/*
 *  Use biped namespace.
 */
using namespace biped;

unsigned int id;
unsigned int count_a, count_b, count_c = 0;

// Declare semaphore handle
static SemaphoreHandle_t display_lock;
static SemaphoreHandle_t motor_lock;
static SemaphoreHandle_t btn_lock;

bool btn_a = false;
bool btn_b = false;
bool btn_c = false;

const uint8_t pinmode_a = 0b11000000;
const uint8_t pinmode_b = 0b00000001;

// create IO expander instance
MCP23018* ioe = new MCP23018(0x20);

// TODO declare execution flag(s)
volatile bool had_int = false;

static uint8_t dir = 0x00;

void IOhandler() {
    xSemaphoreGiveFromISR(btn_lock, NULL);
}


void io_task(void* args) {
    // Do nothing
    while (true) {
        // get the semaphore
        xSemaphoreTake(display_lock, portMAX_DELAY);

        // Update display
        Display(0) << "btn_a: " << (btn_a ? "ACTIVATED" : "*ded*");
        Display(1) << "count_a " << count_a;
        Display(2) << "btn_b: " << (btn_b ? "ACTIVATED" : "*ded*");
        Display(3) << "count_b " << count_b;
        Display(4) << "btn_c: " << (btn_c ? "ACTIVATED" : "*ded*");
        Display(5) << "count_c " << count_c;

        btn_a = btn_b = btn_c = false;
        Display::display();
    }
}


void motor_task(void* pvParameters){
    while(true) {
        // get the semaphore
        xSemaphoreTake(motor_lock, portMAX_DELAY);
        // Update motor dir
        dir ++;
        ioe->SetAPin(IOExpanderAPortAPin::motor_left_direction, (dir & 0x01));
        ioe->SetAPin(IOExpanderAPortAPin::motor_right_direction, (dir & 0x02));
    }
}


void btn_task(void* pvParameters) {
    while (true) {
        // get the semaphore
        xSemaphoreTake(btn_lock, portMAX_DELAY);

        uint8_t int_a, int_b;

        int_a = ioe->readFromRegister(INTFA);
        int_b = ioe->readFromRegister(INTFB);
        uint8_t gpio_a = ioe->GetPortA();
        uint8_t gpio_b = ioe->GetPortB();
        
        btn_a = int_a & (1 << IOExpanderAPortAPin::pushbutton_a);
        btn_b = int_a & (1 << IOExpanderAPortAPin::pushbutton_b);
        btn_c = int_b & (1 << IOExpanderAPortBPin::pushbutton_c);
        
        btn_a = btn_a && ~gpio_a & (1 << IOExpanderAPortAPin::pushbutton_a);
        btn_b = btn_b && ~gpio_a & (1 << IOExpanderAPortAPin::pushbutton_b);
        btn_c = btn_c && ~gpio_b & (1 << IOExpanderAPortBPin::pushbutton_c);

        // Updatet counters
        count_a = btn_a ? count_a+1 : count_a;
        count_b = btn_b ? count_b+1 : count_b;
        count_c = btn_c ? count_c+1 : count_c;

        // Clear flag
        had_int = false;
        ioe->readFromRegister(INTCAPA);
        ioe->readFromRegister(INTCAPB);

        // Update motor
        if (btn_a) xSemaphoreGive(motor_lock);

        // Update display
        xSemaphoreGive(display_lock);
    }
}


void setup() {
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));
    Display::initialize();

    // setup IO expander
    ioe->begin();

    // setup IO expnader interrupts
    ioe->setBitInRegister(IOCON, 7, 0);
    ioe->setBitInRegister(IOCON, 6, 1);
    ioe->setBitInRegister(IOCON, 5, 0);
    ioe->setBitInRegister(IOCON, 2, 0);
    ioe->setBitInRegister(IOCON, 1, 0);  // Active-low
    ioe->setBitInRegister(IOCON, 0, 1);

    ioe->setMaskInRegister(INTCONA, pinmode_a, false);
    ioe->setMaskInRegister(INTCONB, pinmode_b, false);
    
    // setup the IO direction
    ioe->SetDirections(pinmode_a, pinmode_b);
    ioe->SetPullups(0xFF, 0xFF);

    // Create mutex 
    display_lock = xSemaphoreCreateBinary();
    motor_lock = xSemaphoreCreateBinary();
    btn_lock = xSemaphoreCreateBinary();

    // Create task
    xTaskCreate(io_task, "io_task", 2048, NULL, 0, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 0, NULL);
    xTaskCreate(btn_task, "btn_task", 2048, NULL, 0, NULL);

    // Turn on interrupts
    attachInterrupt(ESP32Pin::io_expander_a_interrupt, IOhandler, FALLING);
    ioe->setMaskInRegister(INTENA, pinmode_a, true);
    ioe->setMaskInRegister(INTENB, pinmode_b, true);
    
    // enable motor and set motor speed
    ioe->SetAPin(IOExpanderAPortAPin::motor_left_direction, true);
    ioe->SetAPin(IOExpanderAPortAPin::motor_right_direction, true);
    
    pinMode(ESP32Pin::motor_left_pwm, OUTPUT);
    pinMode(ESP32Pin::motor_right_pwm, OUTPUT);

    analogWrite(ESP32Pin::motor_left_pwm, 100);
    analogWrite(ESP32Pin::motor_right_pwm, 100);

    ioe->SetBPin(IOExpanderAPortBPin::motor_enable, true);
    Display(0) << "TEST";
}   

void loop() {
    // Do nothing
}