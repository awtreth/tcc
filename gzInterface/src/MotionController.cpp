#include <MotionController.h>
#include <JointController.h>

std::queue<PageSet> MotionController::getPageSetQueue() const
{
    return pageSetQueue;
}

bool MotionController::isMoving()
{
    return !this->isPaused();
}

void MotionController::onWrite()
{
    if(currentPageSet.hasCurrentPose())
        wCmd = onWriteCmd(currentPageSet.getCurrentPose());

    currentPageSet.advanceTime(getWritePeriod());
}

void MotionController::write()
{
    writeCmd(wCmd);
    currentPageSet.advanceTime(getWritePeriod());
}

void MotionController::onRead()
{
    rCmd = onReadCmd();
}

void MotionController::read()
{
    joints = readCmd(rCmd);
}

void MotionController::afterRead(){
    afterRead(joints);
}

std::vector<WriteJointCommandPtr> MotionController::onWriteCmd(Pose currentPose)
{
    auto posVelCmds = currentPose.toJointCommand();
    auto ret = std::vector<WriteJointCommandPtr>();

    for(auto posVelCmd : posVelCmds)
        ret.push_back(std::make_shared<PosVelWriteJointCommand>(posVelCmd));

    return ret;
}

void MotionController::writeCmd(std::vector<WriteJointCommandPtr> cmd)
{
    jointController->sendWriteCommand(cmd);
}

std::vector<ReadJointCommandPtr> MotionController::onReadCmd()
{
    auto ret = std::vector<ReadJointCommandPtr>();

    for(Joint joint : joints)
        ret.push_back(std::make_shared<ReadJointCommand>(ReadJointCommand(joint.getName())));

    return ret;

}

std::vector<Joint> MotionController::readCmd(std::vector<ReadJointCommandPtr> cmd)
{
    return jointController->sendReadCommand(cmd);
}

void MotionController::afterRead(std::vector<Joint> updatedJoints)
{

}

MotionController::MotionController()
{
    auto jointController = JointControllerPtr();
    auto currentPageSet = PageSet();
    auto pageSetQueue = std::queue<PageSet>();
}

MotionController::MotionController(JointControllerPtr _jointController)
{
    jointController = _jointController;
    joints = jointController->getJointVec();
    pageSetQueue = std::queue<PageSet>();
}

PageSet MotionController::getCurrentPageSet() const
{
    return currentPageSet;
}
