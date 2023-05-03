/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"

/*
 *  Use biped namespace.
 */
using namespace biped;

unsigned int id;
// TODO Declare variables for the neopixel LEDs
std::shared_ptr<NeoPixel::Frame> myframe = std::make_shared<NeoPixel::Frame>();
int color = 0;
NeoPixel* leds;
unsigned int counter;


void setup() {
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));

    // TODO Initialize the display, serial, and neopixel
    Display::initialize();
    Serial::initialize();
    leds = new biped::NeoPixel();

    // Update display
    Display(0) << id;
    Display(1) << "ziningg2";
    Display(2) << "tw27";
    Display(3) << "net_id3";

    Display::display();

    // Send in serial
    biped::Serial(LogLevel::info) << id;
    biped::Serial(LogLevel::info) << "ziningg2";
    biped::Serial(LogLevel::info) << "tw27";
    biped::Serial(LogLevel::info) << "net_id3";

    

    // TODO Display and print to serial the robot ID and net ids of your group
    leds->clear();
    leds->setBrightness(30);
    
    myframe->push_back(Eigen::Vector3i(0, 0, 0));
    myframe->push_back(Eigen::Vector3i(0, 0, 0));
    myframe->push_back(Eigen::Vector3i(0, 0, 0));
    myframe->push_back(Eigen::Vector3i(0, 0, 0));

}

void loop() {
    // Generate a simple pattern for the LEDs
    color ++;
    if (color > 255) color = 0;
    for(int i = 0; i < 4; i++) {
        (*myframe)[i] = Eigen::Vector3i(color, 255-color, 255-(color/2));
    }
    leds->setFrame(myframe);
    leds->show();

    // Display and print to serial the cycle counts
    counter++;
    biped::Serial(LogLevel::info) << counter;
    Display(4) << counter;
    Display::display();
    
    delay(50);
}
