#include <Page.h>
#include <math.h>

std::string Page::getModelName() const
{
    return modelName;
}

void Page::setModelName(const std::string& value)
{
    modelName = value;
}

std::vector<Pose> Page::getPoses() const
{
    return poses;
}

void Page::setPoses(const std::vector<Pose>& value)
{
    //TODO: verificar se os tempos são válidos. Lançar exceção quando não são
    poses = value;
    //computePageDuration();
}

void Page::fromJsonFile(const char* filePath)
{
    //TODO
}

void Page::toJsonFile(const char* filePath)
{
    //TODO
}


bool Page::advanceTime(long tick)
{
    //TODO
    return false;
}

int Page::getLoopCount() const
{
    return loopCount;
}

long Page::getPageDuration() const
{
    return pageDuration;
}

long Page::computePageDuration()
{
   pageDuration = 0;

    for(auto pose : poses)
        pageDuration += pose.getTimeToNext();

    return pageDuration;
}

int Page::getGlobalTimeCount() const
{
    return globalTimeCount;
}

int Page::getPageTimeCount() const
{
    return pageTimeCount;
}

int Page::getPoseTimeCount() const
{
    return poseTimeCount;
}

int Page::getNumberOfLoops() const
{
    return numberOfLoops;
}

void Page::setNumberOfLoops(int value)
{
    numberOfLoops = value;
}

int Page::getCurrentPoseId() const
{
    return currentPoseId;
}

Page::Page(const std::vector<Pose>& value)
{
    setPoses(value);
}

void Page::setPose(int index, const Pose& pose)
{
    //TODO: verificar se o timestamp é válido com a pose anterior e posterior. Lançar exceção quando não for
    //pageDuration += pose.getTimeToNext() - poses[index].getTimeToNext();
    poses[index] = pose;
}

Pose& Page::getPose(int index)
{
    return poses[index];
}

long roundByResolution(long value, long resolution) {

    return ((long)((value/(double)resolution)+0.5))*resolution;
}

long Page::roundPoseTimes(long resolution)
{

    long offset = 0;
    long adjustedTime;

    for(Pose& pose : poses){
        pose.setTimestamp(roundByResolution(pose.getTimestamp(),resolution));
        adjustedTime = pose.getTimeToNext() + offset;
        pose.setTimeToNext(roundByResolution(adjustedTime,resolution));
        offset = adjustedTime - pose.getTimeToNext();
    }

    //setTimesByTimestamp();


    return offset;
}

int Page::addPose(const Pose& newPose)
{
    poses.push_back(newPose);
    //pageDuration += newPose.getTimeToNext();

    return poses.size();
}

void Page::setTimesByPeriod(long period)
{
    //TODO: period check

    long nextTimestamp = 0;

    for(Pose& pose : poses){
        pose.setTimestamp(nextTimestamp);
        //pose.setTimeToThis(period);
        pose.setTimeToNext(period);
        nextTimestamp += period;
    }

    pageDuration = period*poses.size();
}

void Page::setTimesByTimestamp(long lastTimeToNext) {


    for(unsigned int i = 1; i < poses.size(); i++)
        poses[i-1].setTimeToNext(poses[i].getTimestamp()-poses[i-1].getTimestamp());

    poses.back().setTimeToNext(lastTimeToNext);

}

void Page::setTimesByTimeToNext() {

    long nextTimestamp = 0;

    for(Pose& pose : poses){
        pose.setTimestamp(nextTimestamp);
        nextTimestamp += pose.getTimeToNext();
    }
}

bool Page::checkTimes() {

    if(poses.size()>0){
        if(!poses[0].hasValidTimes())
            return false;

        for(unsigned int i = 1; i < poses.size(); i++){
            auto timestampDiff = poses[i].getTimestamp() - poses[i-1].getTimestamp();

            if(timestampDiff <= 0 || timestampDiff != poses[i-1].getTimeToNext() || !poses[i].hasValidTimes())
                return false;
        }
    }

    return true;
}

std::string Page::getMotionName() const
{
    return motionName;
}

void Page::setMotionName(const std::string& value)
{
    motionName = value;
}
