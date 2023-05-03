/**
 *  @file   time_of_flight.cpp
 *  @author Simon Yu
 *  @date   01/10/2023
 *  @brief  Time-of-flight class source.
 *
 *  This file implements the time-of-flight class.
 */

/*
 *  External headers.
 */
#include <limits>

/*
 *  Project headers.
 */
#include "utility/math.h"
#include "common/parameter.h"
#include "platform/serial.h"
#include "platform/time_of_flight.h"

/*
 *  Biped namespace.
 */
namespace biped
{
TimeOfFlight::TimeOfFlight(const uint8_t& address, const uint8_t& shutdown_pin,
        const std::shared_ptr<IOExpander> io_expander) :
        median_filter_(MedianFilterParameter::window_size),
        vl53l4cx_(&Wire, shutdown_pin, io_expander->getRaw())
{
    /*
     *  Initialize time-of-flight sensor driver object.
     */
    vl53l4cx_.begin();

    /*
     *  Initialize time-of-flight sensor.
     */
    if (const auto error = vl53l4cx_.InitSensor(address << 1))
    {
        Serial(LogLevel::error) << "Failed to initialize time-of-flight sensor."
                << static_cast<int>(error) << " " << static_cast<int>(address);
        return;
    }

    /*
     *  Start time-of-flight sensor measurement.
     */
    if (vl53l4cx_.VL53L4CX_StartMeasurement())
    {
        Serial(LogLevel::error) << "Failed to start time-of-flight sensor measurement.";
        return;
    }
}

double
TimeOfFlight::read()
{
    /*
     *  Declare variables
     */
    uint8_t measurement_data_ready = 0;
    VL53L4CX_MultiRangingData_t multi_ranging_data;
    double range_min = std::numeric_limits<double>::max();
    double range_min_global = std::numeric_limits<double>::max();
    VL53L4CX_Error status = VL53L4CX_ERROR_NONE;

    /*
     *  Get time-of-flight sensor measurements.
     */
    status = vl53l4cx_.VL53L4CX_GetMeasurementDataReady(&measurement_data_ready);

    if (status == VL53L4CX_ERROR_NONE && measurement_data_ready)
    {
        /*
         *  Get time-of-flight sensor multi-ranging data.
         */
        status = vl53l4cx_.VL53L4CX_GetMultiRangingData(&multi_ranging_data);

        if (multi_ranging_data.NumberOfObjectsFound > 0)
        {
            /*
             *  Convert range data to meters and store the smallest ranging data.
             */
            for (int i = 0; i < multi_ranging_data.NumberOfObjectsFound; i ++)
            {
                const auto &range = millimetersToMeters(
                        multi_ranging_data.RangeData[i].RangeMilliMeter);

                if (multi_ranging_data.RangeData[i].RangeStatus == 0)
                {
                    if (range < range_min)
                    {
                        range_min = range;
                    }
                }

                if (range < range_min_global)
                {
                    range_min_global = range;
                }
            }
        }

        /*
         *  Clear interrupt and restart time-of-flight sensor measurement.
         */
        if (status == VL53L4CX_ERROR_NONE)
        {
            status = vl53l4cx_.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }

    /*
     *  Use global minimum range data if no valid range data exists.
     */
    if (range_min == std::numeric_limits<double>::max())
    {
        range_min = range_min_global;
    }

    /*
     *  Filter and return the minimum range data using the median filter.
     */
    return median_filter_.filter(range_min);
}
}   // namespace biped
