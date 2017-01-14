#ifndef POSE_H
#define POSE_H

#include <vector>
#include <map>

struct PosVel {
    double pos;
    double vel;
};


class Pose {

    private:

    std::vector<PosVel> values;//em radianos

    long time; //in microssecons

    //std::map<std::string,int> jointNamesMap;

    public:

    Pose(int numberOfJoints);

    Pose(const char* jsonFilePath);

    Pose(std::vector<PosVel> allPosVel);

    void loadFromJsonFile(const char* filePath);

    void toJsonFile(const char* filePath);

    void setPosVel(int index);

    void setTime(long newTime);

    long getTime();

    PosVel getPosVel(int index);

    void setAllPosVel(std::vector<PosVel> allPosVel);

    std::vector<PosVel> getAllPosVel();

};


#endif
