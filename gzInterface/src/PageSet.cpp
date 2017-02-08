#include <PageSet.h>

bool PageSet::checkMerge(Page otherPage)
{
    for(auto page : pages.getElements()) {
        if(!page.matchPoses(otherPage))
            return false;
    }

    return true;
}

MapVec<Page> PageSet::getPages() const
{
    return pages;
}
