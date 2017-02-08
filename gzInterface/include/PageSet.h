#ifndef PAGESET_H
#define PAGESET_H

#include <Pose.h>
#include <Page.h>
#include <MapVec.h>

class PageSet {

    private:

    MapVec<Page> pages;

    bool checkMerge(Page page);

    public:

    Page& getPage(const char* name);

    bool setPage(Page& otherPage);

    bool setPageSet(PageSet& otherPageSet);

    bool advanceTime(long tick);

    bool getCurrentPose();

    MapVec<Page> getPages() const;
};



#endif
