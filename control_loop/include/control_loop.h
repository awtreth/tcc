#ifndef CONTROLLER_TIME_HANDLER_H
#define CONTROLLER_TIME_HANDLER_H

#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>              // mutex, unique_lock
#include <condition_variable> // condition_variable
#include <icontroller.h>
#include <ihardware.h>
#include <functional>

namespace control_loop {

/**
 * @brief Represents a potencially real-time control loop.
 *
 * It has a single real-time inner thread that periodically
 * read values from the sensors, process the input commands and finally send the proper signals to the actuators. The interaciton with the
 * system is handled by control_loop::IHardware and the "control boxes" are handled by instances of control_loop::IController.
 *
 */
class ControlLoop {

public:

//    ControlLoop();
    ControlLoop(HardwarePtr hwInterface);

    virtual ~ControlLoop();


    //TODO: check if it works
    /**
     * @brief Load a controller so that, on the next iteration, it will be processed
     *
     * @param controllerName Given name to the controller
     * @param controller An instance of controller_loop::IController
     * @return bool not implemented yet
     */
    bool loadController(std::string controllerName, ControllerPtr controller);


    /**
     * @brief
     *
     * @param controllerName
     * @return ControllerPtr
     */
    ControllerPtr unloadController(std::string controllerName);


    /**
     * @brief
     *
     * @param controller_out_name
     * @param controller_in_name
     * @param controller_in
     * @return ControllerPtr
     */
    ControllerPtr switchController(std::string controller_out_name, std::string controller_in_name, ControllerPtr controller_in);


    /**
     * @brief Simply continue the loop iterations
     *
     * @param readWaitTime
     * @return bool returns true if the loop is successfully resumed and false when it was not possible (usually, it happens when the IHardware was not loaded yet)
     */
    bool resumeLoop(std::chrono::microseconds readWaitTime = std::chrono::microseconds(0));


    /**
     * @brief Imediatelly pauses the loop
     *
     */
    void pauseLoop();


    /**
     * @brief Stops the ControlLoop so that it cannot be used anymore
     *
     */
    void close();


    /**
     * @brief Sets the loop frequency in real-time. It can be called during execution
     *
     * @param freq the desired frequency in Hz
     * @return bool not implemented yet
     */
    bool setFrequency(double freq);


    /**
     * @brief safelly gets the set frequency
     *
     * @return double Loop's frequency in Hz
     */
    double getFrequency();

protected:

    virtual void prepareRead();//entre a escrita e a leitura

    /**
     * @brief By default, only calls hardwareInterface->read()
     *
     */
    virtual void read();


    /**
     * @brief By default, calls the method "update" of all controllers loaded (similar to ros_control's ControllerManager)
     *
     */
    virtual void update();


    /**
     * @brief By default, calls hardwareInterface->write()
     *
     */
    virtual void write();

    /**
     * @brief This method is called when the deadlines aren't met. The default implementation only prints the estimated delay and the desired period.
     * By default, the missed iteration is processed and the next deadline is updated to "realization + period"
     *
     * @param desired The missed deadline time_point
     * @param realization Current time
     */
    virtual void onMiss(std::chrono::steady_clock::time_point desired, std::chrono::steady_clock::time_point realization);


private:

    std::thread mainThread; /**< inner real-time tread*/

    //TODO: change it to multiple hardware interfaces
    HardwarePtr hardwareInterface;
    std::map<std::string,ControllerPtr> controllers;
    std::map<std::string,ControllerPtr> rtControllers;

    std::chrono::microseconds period = std::chrono::microseconds(long(20e3));//por padrão: 20ms
    std::chrono::microseconds rtPeriod;

    //Variable access control flags
    bool isPaused = true;
    bool isClosed = false;

    bool hasPeriodChange = true;
    bool hasControllerChange = true;

    std::mutex requestMtx;

    std::condition_variable pauseCv;

    std::chrono::steady_clock::time_point resumeTime;
    std::chrono::steady_clock::time_point nextLoopTime;

    //Initialization helper methods
    void defaultInit();
    bool initThread();

    void loop();
    void loopUpdateCheck();//return false if it was closed

};

}
#endif
