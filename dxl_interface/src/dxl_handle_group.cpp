#include <dxl_handle_group.h>
#include <algorithm>
#include <stdexcept>

dxl_interface::DxlHandle &dxl_interface::DxlHandleGroup::get(int id)
{
    for(DxlHandle& handle : handles){
        if(handle.getId()==id)
            return handle;
    }

    throw std::invalid_argument("No DxlHandle with id " + std::to_string(id));
}

bool dxl_interface::DxlHandleGroup::hasHandle(int id)
{
    for(auto handle : handles){
        if(handle.getId()==id)
            return true;
    }
    return false;
}

bool dxl_interface::DxlHandleGroup::addHandle(dxl_interface::DxlHandle newHandle)
{
    if(hasHandle(newHandle.getId()))
        return false;

    handles.push_back(newHandle);
    return true;
}

std::vector<dxl_interface::DxlHandle>& dxl_interface::DxlHandleGroup::getHandles()
{
    return handles;
}
