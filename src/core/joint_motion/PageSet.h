#ifndef PAGESET_H
#define PAGESET_H

#include <Pose.h>
#include <Page.h>
#include <MapVec.h>

class PageSet {

    private:

    MapVec<Page> pages;

    bool hasCurrentPoseFlag = false;

    Pose currentPose;

    public:

    bool hasCurrentPose() const;

    Page& getPage(const char* name);

    bool setPage(Page& otherPage);

    bool advanceTime(long tick);//TODO: transferir para outra classe

    bool hasFinished();//TODO: transferir para otura classe

    MapVec<Page> getPages() const;

    bool intersectPages(Page otherPage);

    Pose getCurrentPose();

    std::string toString();

    bool resetTime();//TODO: transferir para outra classe



};



#endif
