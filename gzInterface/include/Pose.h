#ifndef POSE_H
#define POSE_H

#include "IJsonObject.h"

#include <MapVec.h>


struct PosVel {
    double pos;
    double vel;

    PosVel(double _pos, double _vel) : pos(_pos), vel(_vel) {}
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

    void addPosVel(const char* jointName, PosVel value);

    void setPosVel(int index, PosVel posVel);

    void setPosVel(const char* jointName, PosVel posVel);

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

};


#endif
