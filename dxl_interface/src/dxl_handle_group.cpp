#include <dxl_handle_group.h>
#include <algorithm>
#include <stdexcept>

dynamixel::DxlHandle &dynamixel::DxlHandleGroup::get(int id)
{
    for(DxlHandle& handle : handles){
        if(handle.getId()==id)
            return handle;
    }

    throw std::invalid_argument("No DxlHandle with id " + std::to_string(id));
}

bool dynamixel::DxlHandleGroup::hasHandle(int id)
{
    for(auto handle : handles){
        if(handle.getId()==id)
            return true;
    }
    return false;
}

bool dynamixel::DxlHandleGroup::addHandle(dynamixel::DxlHandle newHandle)
{
    if(hasHandle(newHandle.getId()))
        return false;

    handles.push_back(newHandle);
    return true;
}

std::vector<dynamixel::DxlHandle>& dynamixel::DxlHandleGroup::getHandles()
{
    return handles;
}
