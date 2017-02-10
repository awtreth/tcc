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
#include <PageSet.h>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

    Pose p11, p12, p13, p21, p22;
    Page page1, page2;

    p11.addPosVel(PosVel(110,111,"motor0"));
    p11.addPosVel(PosVel(112,113,"motor1"));
    p11.setTimeToNext(2);

    p12.addPosVel(PosVel(120,121,"motor0"));
    p12.addPosVel(PosVel(122,123,"motor1"));
    p12.setTimeToNext(2);


    p13.addPosVel(PosVel(132,133,"motor0"));
    p13.addPosVel(PosVel(132,133,"motor1"));
    p13.setTimeToNext(2);


    p21.addPosVel(PosVel(210,211,"motor2"));
    p21.addPosVel(PosVel(212,213,"motor3"));
    p21.setTimeToNext(3);

    p22.addPosVel(PosVel(222,223,"motor2"));
    p22.addPosVel(PosVel(222,223,"motor3"));
    p22.setTimeToNext(5);

    page1.addPose(p11);
    page1.addPose(p12);
    page1.addPose(p13);

    page1.setModelName("Model1");
    page1.setMotionName("Motion1");
    page1.setTimesByTimeToNext();

    page2.addPose(p21);
    page2.addPose(p22);

    page2.setModelName("Model2");
    page2.setMotionName("Motion2");
    page2.setTimesByTimeToNext();

    PageSet pset;

    pset.setPage(page1);
    pset.setPage(page2);

    std::cout << pset.toString() << std::endl;


    int i = 0;

    std::cout << "currentTime = " << i << " hasPose ";

    if(pset.resetTime()){
        std::cout << true << std::endl;
        std::cout << pset.getCurrentPose().toString() << std::endl;
    }else
        std::cout << false << std::endl;

    for(i = 1; i < 18; i++){
        std::cout << "currentTime = " << i << " hasPose ";

        if(pset.advanceTime(1)){
            std::cout << true << std::endl;
            std::cout << pset.getCurrentPose().toString() << std::endl;
        }else
            std::cout << false << std::endl;

    }


    ///////////////////////////////////////////////////////////////

//    Pose pose1, pose2, pose3;

//    pose1.addPosVel("motor 0", PosVel(0,1,"motor 0"));
//    pose1.addPosVel("motor 1", PosVel(2,3,"motor 1"));
//    pose1.addPosVel("motor 2", PosVel(4,5,"motor 2"));

//    pose2.addPosVel("motor 3", PosVel(6,7,"motor 3"));
//    pose2.addPosVel("motor 4", PosVel(8,9,"motor 4"));
//    pose2.addPosVel("motor 2", PosVel(10,11,"motor 2"));
//    pose2.addPosVel("motor 0", PosVel(12,13,"motor 0"));

//    pose3.addPosVel("motor 3", PosVel(14,15,"motor 3"));
//    pose3.addPosVel("motor 4", PosVel(16,17,"motor 4"));
//    pose3.addPosVel("motor 5", PosVel(18,19,"motor 5"));

//    Page page1, page2, page3;

//    page1.addPose(pose1);
//    page1.addPose(pose1);


//    page1.getPose(0).setTimestamp(0);
//    page1.getPose(1).setTimestamp(2);

//    page1.setTimesByTimestamp(4);
//    page1.computePageDuration();

//    page2.addPose(pose2);
//    page3.addPose(pose3);

//    page1.setModelName("model1");
//    page2.setModelName("model2");
//    page3.setModelName("model3");


//    //std::cout << page2.matchPoses(page2) << std::endl;
//    //std::cout << page1.matchPoses(page3) << std::endl;

//    PageSet pset;

//    pset.setPage(page1);
//    pset.setPage(page3);


//    std::cout << pset.setPage(page1) << std::endl;


//    std::cout << pset.setPage(page1) << std::endl;


//    std::cout << pset.toString() << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////

//    std::vector<Pose> poses = std::vector<Pose>(5);

//    poses[0].setTimestamp(0);
//    poses[1].setTimestamp(12);
//    poses[2].setTimestamp(36);
//    poses[3].setTimestamp(78);
//    poses[4].setTimestamp(132);

////    poses[0].setTimeToNext(12);
////    poses[1].setTimeToNext(24);
////    poses[2].setTimeToNext(42);
////    poses[3].setTimeToNext(54);
////    poses[4].setTimeToNext(86);


//    Page page(poses);

//    //page.setTimesByPeriod(15);

//    //page.setTimesByTimeToNext();
//    page.setTimesByTimestamp(86);

//    page.computePageDuration();

//    for(auto i = 0; i < 15; i++){
//        std::cout << page.advanceTime(20) << std::endl;
//        std::cout << page.getCurrentPoseId() << " " << page.getLoopCount()<< " " << page.getPageTimeCount()<< " " << std::endl;
//    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////

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
