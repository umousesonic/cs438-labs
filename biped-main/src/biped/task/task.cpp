/**
 *  @file   task.cpp
 *  @author Simon Yu
 *  @date   12/03/2022
 *  @brief  Task function source.
 *
 *  This file implements the task functions.
//  */
// #define TEST_MPU6050
// #define TEST_COMPASS
// #define TEST_BMX160
// #define CONTROLLER
// #define TEST_ENCODER
// #define TEST_MOTOR
// #define TEST_CALIBRATION
#define PLANNER

/*
 *  External headers.
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>

/*
 *  Project headers.
 */
#include "actuator/actuator.h"
#include "platform/camera.h"
#include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "planner/maneuver_planner.h"
#include "platform/neopixel.h"
#include "sensor/sensor.h"
#include "task/task.h"
#include "planner/waypoint_planner.h"

/*
 *  Biped namespace.
 */
namespace biped
{
void
ioExpanderAInterruptTask(void* pvParameters)
{
    for (;;)
    {
        /*
         *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
         *  function. Set clear count on exit to true and maximum
         *  task wait time to be maximum delay.
         */
        // TODO LAB 6 YOUR CODE HERE.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         *  Validate I/O expander A object pointer and call the I/O
         *  expander interrupt callback function.
         *  See the I/O expander class for details.
         */
        // TODO LAB 6 YOUR CODE HERE.
        if (io_expander_a_ != nullptr) {
            io_expander_a_->onInterrupt();
        }
    }

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
ioExpanderBInterruptTask(void* pvParameters)
{
    for (;;)
    {
        /*
         *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
         *  function. Set clear count on exit to true and maximum
         *  task wait time to be maximum delay.
         */
        // TODO LAB 6 YOUR CODE HERE.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         *  Validate I/O expander B object pointer and call the I/O
         *  expander interrupt callback function.
         *  See the I/O expander class for details.
         */
        // TODO LAB 6 YOUR CODE HERE.
        if (io_expander_b_ != nullptr) {
            io_expander_b_->onInterrupt();
        }
    }

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
cameraTask(void* pvParameters)
{
    /*
     *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
     *  function. Set clear count on exit to true and maximum
     *  task wait time to be maximum delay.
     */
    // TODO LAB 9 YOUR CODE HERE.

    /*
     *  Validate camera object pointer and perform streaming.
     *  See the camera class for details.
     */
    // TODO LAB 9 YOUR CODE HERE.

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
wiFiTask(void* pvParameters)
{
    /*
     *  Initialize the Wi-Fi driver object and disable sleep.
     *  See parameter header for details
     */
    // TODO LAB 9 YOUR CODE HERE.

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
realTimeTask(void* pvParameters)
{
    /*
     *  Declare start time point and set to 0.
     */
    unsigned long time_point_start = 0;

    for (;;)
    {
        /*
         *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
         *  function. Set clear count on exit to true and maximum
         *  task wait time to be maximum delay.
         */
        // TODO LAB 6 YOUR CODE HERE.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         *  Calculate real-time task interval and update start
         *  time point.
         */
        interval_real_time_task_ = micros() - time_point_start;
        time_point_start = micros();

        /*
         *  Perform fast domain sensing.
         *  See the sensor class for details.
         */
        // TODO LAB 6 YOUR CODE HERE.
        sensor_->sense(true);

        /*
         *  Perform fast domain control.
         *  See the controller class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.
        controller_->control(true);

        /*
         *  Slow domain tasks.
         */
        if (timer_domain_ >= PeriodParameter::slow)
        {
            /*
             *  Perform slow domain sensing.
             *  See the sensor class for details.
             */
            // TODO LAB 6 YOUR CODE HERE.
            sensor_->sense(false);

            /*
             *  Perform slow domain control.
             *  See the controller class for details.
             */
            // TODO LAB 7 YOUR CODE HERE.
            controller_->control(false);

            /*
             *  Reset timing domain timer.
             */
            timer_domain_ = 0;
        }

        /*
         *  Perform actuation using the actuation
         *  command struct from the controller object.
         *  See the actuator class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.
        actuator_->actuate(controller_->getActuationCommand());

        /*
         *  Update timing domain timer.
         */
        timer_domain_ += PeriodParameter::fast;

        /*
         *  Calculate real-time task execution time.
         */
        execution_time_real_time_task_ = micros() - time_point_start;
    }

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
bestEffortTask()
{
    /*
     *  Declare camera task woken flag and set to false.
     */
    static bool camera_task_woken = false;

    /*
     *  Print serial number to display.
     */
    Display(0) << "Biped: #" << serial_number_;

    /*
     *  Print real-time task timings to display.
     */
    Display(1) << "Real-time: " << execution_time_real_time_task_ << " "
            << interval_real_time_task_;
    
    // TEST encoder
    #ifdef TEST_ENCODER
    Display(2) << "Enc Stp: " << sensor_->getEncoderData().steps;
    Display(3) << "Enc Pos: " << sensor_->getEncoderData().position_x;
    Display(4) << "Enc Vel: " << sensor_->getEncoderData().velocity_x;
    #endif

    //TEST MPU6050
    #ifdef TEST_MPU6050
    IMUData data = sensor_->getIMUDataMPU6050();
    IMUData data_ = sensor_->getIMUDataBMX160();
    
    Display(2) << "Acc:" << floor(data.acceleration_x * 100) / 100 << " " << floor(data.acceleration_y * 100) / 100 << " " << floor(data.acceleration_z * 100) / 100;
    Display(3) << "Att y:" << floor(data.attitude_y * 100) / 100 << " Att z:" << floor(data_.attitude_z * 100) / 100;
    Display(4) << "AnVel:" << floor(data.angular_velocity_x * 100) / 100 << " " << floor(data.angular_velocity_y * 100) / 100 << " " << floor(data.angular_velocity_z * 100) / 100;
    #endif
    // TEST BMX160
    #ifdef TEST_BMX160
    IMUData data = sensor_->getIMUDataBMX160();
    
    Display(2) << "Acc:" << floor(data.acceleration_x * 100) / 100 << " " << floor(data.acceleration_y * 100) / 100 << " " << floor(data.acceleration_z * 100) / 100;
    Display(3) << "Att:" << floor(data.attitude_z * 100) / 100;
    Display(4) << "AnVel:" << floor(data.angular_velocity_x * 100) / 100 << " " << floor(data.angular_velocity_y * 100) / 100 << " " << floor(data.angular_velocity_z * 100) / 100;
    // Display(4) << "compass:" << floor(data.compass_x * 100) / 100 << " " << floor(data.compass_y * 100) / 100 << " " << floor(data.compass_z * 100) / 100;
    #endif
    // TEST ToF
    #ifdef TEST_TOF
    // std::string tdbuf = "";
    TimeOfFlightData td = sensor_->getTimeOfFlightData();
    Display(4) << "ToF: L:" + std::to_string(floor(td.range_left * 100) / 100);
    Display(5) << "R: " + std::to_string(floor(td.range_right * 100) / 100);
    Display(6) << "M: " + std::to_string(floor(td.range_middle * 100) / 100);



    // Display(4) << tdbuf;
    #endif

    #ifdef TEST_MOTOR
    ActuationCommand command;
    command.motor_enable = true;
    command.motor_left_pwm = 50;
    command.motor_right_pwm = 50;
    actuator_->actuate(command);

    #endif
    
    /*
    //  *  Print controller active status to display.
    //  */
    #ifdef CONTROLLER
    if (controller_->getActiveStatus())
    {
        Display(2) << "Controller: active";
    }
    else
    {
        Display(2) << "Controller: inactive";
    }
    #endif

    #ifdef TEST_COMPASS
    IMUData data = sensor_->getIMUDataBMX160();
    Display(3) << "Compass x:" << floor(data.compass_x * 100) / 100;
    Display(4) << "Compass y:" << floor(data.compass_y * 100) / 100;
    Display(5) << "Compass z:" << floor(data.compass_z * 100) / 100;


    #endif

    #ifdef TEST_CALIBRATION
    Compass::Calibration data = sensor_->getCompassCalibrationBMX160();
    IMUData data_ = sensor_->getIMUDataBMX160();
    Display(2) << "os:" << floor(data.offset_x * 100) / 100 << " " << floor(data.offset_y * 100) / 100 << " " << floor(data.offset_z * 100) / 100;
    Display(3) << "sc:" << floor(data.scaler_x * 100) / 100 << " " << floor(data.scaler_y * 100) / 100 << " " << floor(data.scaler_z * 100) / 100;
    Display(4) << "sn:" << floor(data.sign_x * 100) / 100 << " " << floor(data.sign_y * 100) / 100 << " " << floor(data.sign_z * 100) / 100;
    Display(5) << "compass:" << floor(data_.compass_x * 100) / 100 << " " << floor(data_.compass_y * 100) / 100 << " " << floor(data_.compass_z * 100) / 100;


    #endif

    // /*
    //  *  Execute plan and store the planner stage.
    //  *  See the planner class for details.
    //  */
    #ifdef PLANNER
    // TODO LAB 8 YOUR CODE HERE.
    const int stage = planner_->plan();

    /*
     *  Print planner status to display.
     */
    if (stage < 0)
    {
        Display(3) << "Planner: inactive";
    }
    else
    {
        Display(3) << "Planner: stage " << stage;
    }
    #endif

    // /*
    //  *  Print Wi-Fi status to display.
    //  */
    #ifdef WIFI
    // if (WiFi.status() == WL_CONNECTED)
    // {
    //     Display(4) << "Wi-Fi: " << WiFi.localIP().toString().c_str();

    //     /*
    //      *  If the Wi-Fi is connected, validate the camera task
    //      *  handle and wake camera task using the FreeRTOS
    //      *  xTaskNotifyGive function if the camera task woken flag
    //      *  is false. Then, set camera task woken flag to true.
    //      */
    //     // TODO LAB 9 YOUR CODE HERE.
    // }
    // else
    // {
    //     Display(4) << "Wi-Fi: disconnected";
    // }
    #endif

    // /*
    //  *  Show the NeoPixel frame.
    //  *  See the NeoPixel class for details.
    //  */
    // // TODO LAB 6 YOUR CODE HERE.
    if (neopixel_ != nullptr) neopixel_->show();

    /*
     *  Flush the display driver buffer to the display.
     *  See the display class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    Display::display();
}
}   // namespace biped
