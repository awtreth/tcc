#ifndef PAGE_H
#define PAGE_H

#include <Pose.h>


class Page : public IJsonObject{

    private:

    std::string motionName;

    std::string modelName;

    std::vector<Pose> poses;


    int currentPoseId = 0;

    int globalTimeCount = 0;
    int pageTimeCount = 0;
    int poseTimeCount = 0;

    int loopCount = 0;
    int numberOfLoops = 0;

    long pageDuration = 0;

    public:

    //Constructors
    Page(const std::vector<Pose>& value);
    Page();

    //Getters & Setters
    void setPose(int index, const Pose& pose);
    Pose& getPose(int index);

    std::string getMotionName() const;
    void setMotionName(const std::string& value);

    std::string getModelName() const;
    void setModelName(const std::string& value);

    std::vector<Pose> getPoses() const;
    void setPoses(const std::vector<Pose>& value);

    int getNumberOfLoops() const;
    void setNumberOfLoops(int value);

    //Getters Only
    int getLoopCount() const;
    long getPageDuration() const;
    int getGlobalTimeCount() const;
    int getPageTimeCount() const;
    int getPoseTimeCount() const;
    int getCurrentPoseId() const;

    // IJsonObject interface
    virtual void fromJsonFile(const char* filePath);
    virtual void toJsonFile(const char* filePath);

    //Misc
    int addPose(const Pose& newPose);
    unsigned int size();

    //Time First Settings Methods
    void setTimesByPeriod(long period);
    void setTimesByTimestamp(long lastTimeToNext);
    void setTimesByTimeToNext();
    bool checkTimes();

    //Time Final Settings Methods
    long roundPoseTimes(long resolution);
    long computePageDuration();

    //Time Flow Methods
    bool advanceTime(long tick);

    int advancePose();

    Pose currentPose() const;

    Pose nextPose() const;

    bool matchPoses(Page otherPage);

    bool intersectPoses(Page otherPage);

    std::string toString();

};



#endif
