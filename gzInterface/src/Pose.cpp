#include <Pose.h>


MapVec<PosVel> Pose::getValues() const
{
    return values;
}

void Pose::setValues(const MapVec<PosVel>& value)
{
    values = value;
}

long Pose::getTimestamp() const
{
    return timestamp;
}

long Pose::getTimeToThis() const
{
    return timeToThis;
}

long Pose::getTimeToNext() const
{
    return timeToNext;
}

bool Pose::hasValidTimes()
{
    return timeToNext > 0 && timestamp >= 0;
}

unsigned int Pose::size()
{
    return this->values.size();
}

bool Pose::match(Pose otherPose)
{
    if(this->size() == otherPose.size()) {

        if(this->size() > 0){
            auto thisPoseMap = this->getValues().getNameMap();
            auto otherPoseMap = otherPose.getValues().getNameMap();

            for(auto this_it = thisPoseMap.begin(), other_it = otherPoseMap.begin(); this_it != thisPoseMap.end(); this_it++, other_it++) {
                if(this_it->first != other_it->first)
                    return false;
            }
        }

        return true;
    }

    return false;
}

bool Pose::intersect(Pose otherPose)
{
    if(this->size() > 0 && otherPose.size() > 0) {

        auto thisPoseMap = this->getValues().getNameMap();
        auto otherPoseMap = otherPose.getValues().getNameMap();

        for(auto thisPair : thisPoseMap) {
            for(auto otherPair : otherPoseMap) {
                if(thisPair.first == otherPair.first)
                    return true;
            }
        }

    }

    return false;
}

bool Pose::append(Pose otherPose)
{

    for(unsigned int i = 0; i < otherPose.size(); i++){
        this->addPosVel(otherPose.getPosVel(i));
    }
    return true;
}

std::__cxx11::string Pose::toString()
{
    std::string str;

    for(PosVel joint : values.getElements())
        str += std::string(joint.jointName) + "(" + std::to_string(joint.pos) + "," + std::to_string(joint.vel) + ") ";

    return str;
}

void Pose::setTimeToNext(long value)
{
    //TODO: throw exception when value <= 0
    timeToNext = value;
}

void Pose::setTimestamp(long value)
{
    timestamp = value;
}


void Pose::setTimeToThis(long value)
{
    timeToThis = value;
}

Pose::Pose()
{
    values = MapVec<PosVel>();
}

bool Pose::isEmpty()
{
    return this->values.size() == 0;
}

Pose::Pose(const char* jsonFilePath)
{
    this->fromJsonFile(jsonFilePath);
}

Pose::Pose(const MapVec<PosVel>& _values)
{
    setValues(_values);
}

void Pose::addPosVel(const char* jointName, const PosVel& value)
{
    //value.jointName = jointName;
    values.add(jointName, value);
}

void Pose::addPosVel(const PosVel& value)
{
    values.add(value.jointName,value);
}

void Pose::fromJsonFile(const char* filePath)
{
    //TODO
}

void Pose::toJsonFile(const char* filePath)
{
    //TODO
}

void Pose::setPosVel(int index, PosVel posVel)
{
    values.set(index,posVel);
}

void Pose::setPosVel(const char* jointName, const PosVel& posVel)
{
    values.set(jointName,posVel);
}

PosVel Pose::getPosVel(int index) const
{
    return values.get(index);
}

PosVel Pose::getPosVel(const char* jointName) const
{
    return values.get(jointName);
}
