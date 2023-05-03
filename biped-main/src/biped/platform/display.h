/**
 *  @file   display.h
 *  @author Simon Yu
 *  @date   01/02/2023
 *  @brief  Display class header.
 *
 *  This file defines the display class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_DISPLAY_H_
#define PLATFORM_DISPLAY_H_

/*
 *  External headers.
 */
#include <Adafruit_SH110X.h>
#include <map>
#include <mutex>
#include <sstream>

/*
 *  Project headers.
 */
#include "utility/low_pass_filter.hpp"
#include "common/type.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Display class.
 *
 *  This class provides functions for printing to the OLED display.
 */
class Display
{
public:

    /**
     *  @param  line Line to print.
     *  @param  raw Whether to print in raw mode.
     *  @brief  Display class constructor.
     *
     *  This constructor acquires the display driver mutex and sets up
     *  the display for printing depending on the printing mode.
     */
    Display(const unsigned& line, const bool& raw = false);

    /**
     *  @brief  Display class destructor.
     *
     *  This destructor flushes the class member string stream buffer
     *  to the display driver buffer, saves the current X cursor position,
     *  and releases the display driver mutex.
     */
    ~Display();

    /**
     *  @brief  Initialize display.
     *
     *  This function initializes the display driver and configures the
     *  display as well as Z acceleration low-pass filter.
     */
    static void
    initialize();

    /**
     *  @brief  Flush the display driver buffer to the display.
     *
     *  This function flushes the display driver buffer to the display
     *  hardware, therefore displaying the contents of the display driver
     *  buffer onto the display. The function also sets the orientation of
     *  the display depending on the current low-pass filtered Z acceleration
     *  value.
     */
    static void
    display();

    /**
     *  @param  item Stream manipulator to perform.
     *  @return Display object reference.
     *  @brief  Perform the given stream manipulator.
     *
     *  This operator function interprets the given stream manipulator,
     *  performs their respective functionalities, and returns a reference
     *  to this object.
     */
    Display&
    operator<<(const StreamManipulator& item);

    /**
     *  @tparam Type Type of data to stream.
     *  @param  item data to stream.
     *  @return Display object reference.
     *  @brief  Stream the given data.
     *
     *  This operator function streams the given data to the string stream
     *  buffer and returns a reference to this object.
     */
    template<typename Type>
    inline Display&
    operator<<(const Type& item)
    {
        /*
         *  Stream the given data to the string stream buffer.
         */
        ss_ << item;

        /*
         *  Return a reference to this object.
         */
        return *this;
    }

private:

    /**
     *  @brief  Perform the carriage return.
     *
     *  This function performs the carriage return by manipulating the
     *  display cursor.
     */
    void
    carriageReturn();

    /**
     *  @brief  Clear the current line.
     *
     *  This function clears the current line by filling the line with
     *  a black rectangle.
     */
    void
    clearLine();

    /**
     *  @brief  Flush string stream buffer to display driver buffer.
     *
     *  This function flushes the class member string stream buffer to
     *  the display driver buffer and subsequently clears the string
     *  stream buffer.
     */
    void
    flush();

    /**
     *  @brief  Restore the display cursor position.
     *
     *  This function restores the display cursor position to the last
     *  saved position.
     */
    void
    restoreCursor();

    static std::map<unsigned, int16_t> cursor_x_locations_; //!< X cursor location map.
    static volatile bool displayed_;    //!< Display flag.
    static uint16_t font_height_;   //!< Font height.
    static uint16_t font_width_;    //!< Font width.
    unsigned line_; //!< Line number.
    static std::unique_lock<std::mutex> lock_sh1107_;   //!< Display driver object mutex lock.
    static LowPassFilter<double> low_pass_filter_acceleration_z_; //!< Z acceleration low-pass filter.
    static std::mutex mutex_sh1107_;    //!< Display driver object mutex.
    static Adafruit_SH1107 sh1107_; //!< SH1107 OLED display driver object.
    std::stringstream ss_;  //!< String stream buffer.
};
}   // namespace biped

#endif  // PLATFORM_DISPLAY_H_
