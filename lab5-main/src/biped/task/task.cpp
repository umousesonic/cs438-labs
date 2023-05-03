/*
 * task.cpp
 *
 *  Created on: Dec 3, 2022
 *      Author: simonyu
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#include "platform/display.h"
#include "common/parameter.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "platform/neopixel.h"
#include "platform/serial.h"
#include "sensor/sensor.h"
#include "task/task.h"

namespace biped
{
void
ioExpanderAInterruptTask(void* pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (io_expander_a_)
        {
            io_expander_a_->onInterrupt();
        }
    }

    vTaskDelete(nullptr);
}

void
ioExpanderBInterruptTask(void* pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (io_expander_b_)
        {
            io_expander_b_->onInterrupt();
        }
    }

    vTaskDelete(nullptr);
}

void realTimeTask(void* pvParameters) {
    unsigned long time_point_start = 0;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        interval_real_time_task_ = micros() - time_point_start;
        time_point_start = micros();

        /*
         *  Perform fast domain sensing.
         *  See the Sensor class for details.
         */
        
        // TODO LAB 5 YOUR CODE HERE.
        // lock_wire_.lock();
        // while(!lock_wire_.try_lock()) {}
        sensor_->sense(true);
        // lock_wire_.unlock();
        /*
         *  Perform fast domain control.
         *  See the Controller class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.

        /*
         *  Slow domain tasks.
         */
        if (timer_domain_ >= PeriodParameter::slow)
        {
            /*
             *  Perform slow domain sensing.
             *  See the Sensor class for details.
             */
            // TODO LAB 5 YOUR CODE HERE.
            // while(!lock_wire_.try_lock()) {}
            // lock_wire_.lock();
            sensor_->sense(false);
            // lock_wire_.unlock();
            /*
             *  Perform slow domain control.
             *  See the Controller class for details.
             */
            // TODO LAB 7 YOUR CODE HERE.

            /*
             *  Reset timing domain timer.
             */
            timer_domain_ = 0;
        }

        /*
         *  Perform actuation.
         *  See the Actuator class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.

        /*
         *  Update timing domain timer.
         */
        timer_domain_ += PeriodParameter::fast;

        execution_time_real_time_task_ = micros() - time_point_start;
    }

    vTaskDelete(nullptr);
}

void
bestEffortTask()
{
    Display(0) << "Biped: #" << serial_number_;

    Display(1) << "Real-time: " << execution_time_real_time_task_ << " "
            << interval_real_time_task_;

    // 
    IMUData data = sensor_->getIMUDataMPU6050();
    TimeOfFlightData td = sensor_->getTimeOfFlightData();

    Display(2) << "Acc:" << floor(data.acceleration_x * 100) / 100 << " " << floor(data.acceleration_y * 100) / 100 << " " << floor(data.acceleration_z * 100) / 100;
    Display(3) << "Att:" << floor(data.attitude_x * 100) / 100 << " " << floor(data.attitude_y * 100) / 100 << " " << floor(data.attitude_z * 100) / 100;

    Display(4) << "AnVel:" << floor(data.angular_velocity_x * 100) / 100 << " " << floor(data.angular_velocity_y * 100) / 100 << " " << floor(data.angular_velocity_z * 100) / 100;

    std::string tdbuf = "";

    

    if (!td.ranges_left.empty()) tdbuf += "ToF: L:" + std::to_string(floor(td.ranges_left[0] * 100) / 100);
    else tdbuf += "ToF: L: na";

    if (!td.ranges_right.empty()) tdbuf += "\nR: " + std::to_string(floor(td.ranges_right[0] * 100) / 100);
    else tdbuf += "\nR: na";

    if (!td.ranges_middle.empty()) tdbuf += " M: " + std::to_string(floor(td.ranges_middle[0] * 100) / 100);
    else tdbuf += " M: na";

    Display(5) << tdbuf;
    
    /*
     *  Execute plan.
     *  See the Planner class for details.
     */
    // TODO LAB 8 YOUR CODE HERE.

    /*
     *  Show the NeoPixel frame.
     *  See the NeoPixel class for details.
     */
    neopixel_->show();
    Display::display();
}
}
