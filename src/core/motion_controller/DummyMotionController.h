#ifndef DUMMY_MOTION_CONTROLLER_H
#define DUMMY_MOTION_CONTROLLER_H

#include <motion_controller/MotionController.h>
#include <AsyncLogger.h>
#include <chrono>
#include <thread>

class DummyMotionController:public MotionController {

private:
    AsyncLogger logger;
    Pose pose;


    steady_clock::time_point lastWritePoint = std::chrono::steady_clock::now();

    steady_clock::time_point lastReadPoint = std::chrono::steady_clock::now();

    steady_clock::time_point now;

    std::chrono::microseconds delayTime;

    // MotionController interface
protected:

    void writeCmd(std::vector<WriteJointCommand> cmd){
        now = std::chrono::steady_clock::now();
        long period = std::chrono::duration_cast<std::chrono::microseconds>(now-lastWritePoint).count();
        long ts = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        long expected = std::chrono::duration_cast<std::chrono::microseconds>(getNextWriteTime().time_since_epoch()).count();

        //std::cout << "WRITE," + std::to_string(log.ts) + "," + std::to_string(log.period) + "," + std::to_string(log.expected) << std::endl;
        //std::cout << pose.toString() << std::endl;

        logger.write("WRITE " + std::to_string(ts) + " " + std::to_string(period) + " " + std::to_string(expected)+ " " + pose.toString());
        //logger.write(pose.toString());

        lastWritePoint = now;

        std::this_thread::sleep_for(delayTime);
    }

    std::vector<ReadJointCommand> onReadCmd(){
        return std::vector<ReadJointCommand>();
    }

    JointMap readCmd(std::vector<ReadJointCommand> cmd){
        now = std::chrono::steady_clock::now();
        long period = std::chrono::duration_cast<std::chrono::microseconds>(now-lastReadPoint).count();
        long ts = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        long expected = std::chrono::duration_cast<std::chrono::microseconds>(getNextReadTime().time_since_epoch()).count();

        //std::cout << "READ," + std::to_string(log.ts) + "," + std::to_string(log.period) + "," + std::to_string(log.expected) << std::endl;
        //std::cout << pose.toString() << std::endl;

        logger.write("READ " + std::to_string(ts) + " " + std::to_string(period) + " " + std::to_string(expected));
        //logger.write(pose.toString());

        lastReadPoint = now;

        std::this_thread::sleep_for(delayTime);

        return JointMap();
    }

    std::vector<WriteJointCommand> onWriteCmd(Pose currentPose){
        pose = currentPose;
        return MotionController::onWriteCmd(currentPose);
    }

public:

    DummyMotionController(long _delayTime = 10e3){
        delayTime = std::chrono::microseconds(_delayTime);
        initThread();
    }

    ~DummyMotionController(){
        logger.close();
    }
};


#endif
