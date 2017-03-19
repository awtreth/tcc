#ifndef POSE_H
#define POSE_H

#include "IJsonObject.h"

#include <MapVec.h>
#include <string>
#include <JointCommand.h>

class PosVel {

    public:

    double pos;
    double vel;
    std::string jointName;
    //int jointIndex = 0;



    //PosVel(double _pos, double _vel) : pos(_pos), vel(_vel), jointName(""){}

    PosVel(double _pos, double _vel, const std::string _jointName) {
        pos = _pos;
        vel = _vel;
        jointName = _jointName;
    }

    // assume the object holds reusable storage, such as a heap-allocated buffer mArray
    PosVel& operator=(const PosVel& other) // copy assignment
    {
        if (this != &other) { // self-assignment check expected
            pos = other.pos;
            vel = other.vel;
            jointName = other.jointName;
        }
        return *this;
    }

    //PosVel(double _pos, double _vel, int idx) : pos(_pos), vel(_vel), jointIndex(idx){}

    //PosVel(double _pos, double _vel, std::string _jointName, int idx) : pos(_pos), vel(_vel), std::string(_jointName) {}

};


class Pose : public IJsonObject{

    //friend class Page;

    private:

    MapVec<PosVel> values;

    //in microssecons
    long timestamp = -1;
    long timeToNext = -1;
    long timeToThis = -1;

    void setTimeToThis(long value);

    long getTimeToThis() const;


    public:

    Pose();

    bool isEmpty();

    Pose(const char* jsonFilePath);

    Pose(const MapVec<PosVel>& value);

    void addPosVel(const char* jointName, const PosVel& value);

    void addPosVel(const PosVel& value);

    void setPosVel(int index, PosVel posVel);

    void setPosVel(const char* jointName, const PosVel& posVel);

    PosVel getPosVel(int index) const;

    PosVel getPosVel(const char* jointName) const;

    MapVec<PosVel> getValues() const;
    void setValues(const MapVec<PosVel>& value);

    //IJsonObject Interface
    virtual void fromJsonFile(const char* filePath);

    virtual void toJsonFile(const char* filePath);

    void setTimestamp(long value);
    void setTimeToNext(long value);

    long getTimestamp() const;
    long getTimeToNext() const;

    bool hasValidTimes();

    unsigned int size();

    bool match(Pose otherPose);

    bool intersect(Pose otherPose);

    bool append(Pose otherPose);

    std::string toString();

    std::vector<PosVelWriteJointCommand> toJointCommand();
};


#endif
