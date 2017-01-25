//#include <gazebo/gazebo_config.h>
//#include <gazebo/transport/transport.hh>
//#include <gazebo/msgs/msgs.hh>
////#include <gz_interface_msgs.pb.h>
//#include <gazebo/gazebo_client.hh>

//#include "DxlMemMap.h"
//#include <iostream>
//#include <jsoncpp/json/json.h>
//#include "DxlModelParamConverter.h"
//#include <Joint.h>
//#include <JointController.h>
//#include <GzJointController.h>
//#include <stdlib.h>

//#include <memory>
//#include <vector>
//#include <string>
//#include <stdio.h>
//#include <stdlib.h>

////#include <GzMotionController.h>

//#include <DummyMotionController.h>

#include <iostream>
#include <string>
#include <MapVec.h>
#include <map>
#include <JointCommand.h>
#include <Pose.h>
#include <Page.h>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

    std::vector<Pose> poses = std::vector<Pose>(5);

    poses[0].setTimestamp(0);
    poses[1].setTimestamp(12);
    poses[2].setTimestamp(36);
    poses[3].setTimestamp(78);
    poses[4].setTimestamp(132);

//    poses[0].setTimeToNext(12);
//    poses[1].setTimeToNext(24);
//    poses[2].setTimeToNext(42);
//    poses[3].setTimeToNext(54);
//    poses[4].setTimeToNext(86);


    Page page(poses);

    //page.setTimesByPeriod(15);

    //page.setTimesByTimeToNext();
    page.setTimesByTimestamp(86);

    page.computePageDuration();

    for(auto i = 0; i < 15; i++){
        std::cout << page.advanceTime(20) << std::endl;
        std::cout << page.getCurrentPoseId() << " " << page.getLoopCount()<< " " << page.getPageTimeCount()<< " " << std::endl;
    }

//    for(auto pose: page.getPoses()){
//        std::cout << pose.getTimestamp() << " " << pose.getTimeToNext() << std::endl;
//    }

//    page.roundPoseTimes(10);

//    for(auto pose: page.getPoses()){
//        std::cout << pose.getTimestamp() << " " << pose.getTimeToNext() << std::endl;
//    }

//    std::cout << page.computePageDuration() << std::endl;


//    TorqueWriteJointCommand cmd(2);

//    //cmd.setTorque(34);

//    std::cout << readJointStateCommand.getCmdID() << std::endl;
//    //std::cout << ReadJointStateCommand.getTorque() << std::endl;

//    std::cout << TorqueWriteJointCommand::CMD_ID << std::endl;


//    DummyMotionController motionController;

//        motionController.setReadPeriod(2e6);
//        motionController.setWritePeriod(2e6);

//        motionController.resume(3e6);

//        sleep(5);

//        motionController.close();
//    std::vector<std::string> vecStr;

//    vecStr.push_back("Motor 1");
//    vecStr.push_back("Motor 2");

//    GzJointController controller(vecStr,"~/simple_arm/writeRequest", "~/simple_arm/readRequest", "~/simple_arm/readResponse");

//    GzMotionController motionController(controller);

//    motionController.setReadPeriod(2e6);
//    motionController.setWritePeriod(2e6);

//    motionController.resume(3e6);

//    //getchar();

//    sleep(5);

//    motionController.pause();

//    std::cout << "paused" << std::endl;

//    getchar();

//    motionController.close();

//    return 0;


//    std::cout << "criando controller" << std::endl;

//    GzJointController controller(vecStr,"~/simple_arm/writeRequest", "~/simple_arm/readRequest", "~/simple_arm/readResponse");

//    std::cout << "conexão estabelecida" << std::endl;

//    PidValues pid(0.1,0,0);

//    std::cout << "write PID " + std::to_string(pid.kp) << std::endl;

//    std::vector<PidValues> pidVec(2,pid);


//    controller.setPosVelPid(pidVec,pidVec);

//    std::cout << "PID enviado" << std::endl;

//    std::cout << "writePosVel" << std::endl;


//    std::vector<JointCommandPtr> cmdVec;

//    cmdVec.push_back(std::make_shared<JointPosVelCommand>(JointPosVelCommand(45*3.141592/180,0)));
//    cmdVec.push_back(std::make_shared<JointPosVelCommand>(JointPosVelCommand(-15*3.141592/180,0)));

//    controller.sendCommand(cmdVec);

////    std::vector<JointPosVelCommand> posVelCmdVec;

////    posVelCmdVec.push_back(JointPosVelCommand(45*3.141592/180,0));
////    posVelCmdVec.push_back(JointPosVelCommand(-15*3.141592/180,0));

////    controller.goPosVel(posVelCmdVec);

//    std::cout << "PosVel enviado" << std::endl;

//    //getchar();

//    std::ofstream ofs("/tmp/gzLog.txt");

//    while(1) {
//        std::cout << "read JointState" << std::endl;

//        controller.readJointStates();

//        JointState jState = controller.getLastJointState(0);

//        ofs << jState.position << " " << jState.velocity << std::endl;

//        usleep(100000);
//    }

//    ofs.close();


    //JointPosVelCommand jcmd(2,3);

    //jcmd.hasPosVel();

    //JointPosVelCommand jcmd(2,3);

}
