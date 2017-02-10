#ifndef PAGESET_H
#define PAGESET_H

#include <Pose.h>
#include <Page.h>
#include <MapVec.h>

class PageSet {

    private:

    MapVec<Page> pages;

    Pose currentPose;

    public:

    Page& getPage(const char* name);

    bool setPage(Page& otherPage);

    bool advanceTime(long tick);

    MapVec<Page> getPages() const;

    bool intersectPages(Page otherPage);

    Pose getCurrentPose() const;

    std::string toString();

    bool resetTime();

};



#endif
