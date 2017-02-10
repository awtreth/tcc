#include <PageSet.h>

Pose PageSet::getCurrentPose() const
{
    return currentPose;
}

std::__cxx11::string PageSet::toString()
{
    std::string str;

    for(Page page : pages.getElements()){
        str += page.toString();
    }

    return str;
}

bool PageSet::resetTime()
{
    currentPose = Pose();

    bool status = false;

    for(Page& page : pages.getElements()){
        if(page.resetTime()){
            currentPose.append(page.currentPose());
            status = true;
        }

    }

    return status;
}

Page& PageSet::getPage(const char* name)
{
    return pages.get(name);
}

bool PageSet::setPage(Page& otherPage)
{
    if(pages.contains(otherPage.getModelName())){
        if(pages.get(otherPage.getModelName().c_str()).matchPoses(otherPage))
            pages.set(otherPage.getModelName(),otherPage);
        else
            return false;
    }else {
        if(this->intersectPages(otherPage))
            return false;
        else
            pages.add(otherPage.getModelName(),otherPage);
    }

    return true;
}

bool PageSet::advanceTime(long tick)
{

    currentPose = Pose();

    bool status = false;

    for(Page& page : pages.getElements()){
        if(page.advanceTime(tick)){
            currentPose.append(page.currentPose());
            status = true;
        }

    }

    return status;
}


MapVec<Page> PageSet::getPages() const
{
    return pages;
}

bool PageSet::intersectPages(Page otherPage)
{
    for(auto page : pages.getElements()){
        if(page.intersectPoses(otherPage))
            return true;
    }
    return false;
}
