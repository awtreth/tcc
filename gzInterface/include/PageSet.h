#ifndef PAGE_H
#define PAGE_H

#include <Pose.h>


class Page {

    private:

    std::vector<Pose> poses;

    int nRepetitions;

    public:

    Page(std::vector<Pose> poses);

    void setAllPoses(std::vector<Pose> poses);

    void loadPageFromJsonFile(const char* fileName);

    void setPose(int index, Pose pose);

    Pose getPose(int index);

};



#endif
