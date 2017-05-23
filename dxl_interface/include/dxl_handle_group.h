#ifndef DXL_HANDLE_GROUP_H
#define DXL_HANDLE_GROUP_H

#include <dxl_interface/dxl_handle.h>
#include <dxl_interface/dxl_command.h>
#include <vector>

namespace dxl_interface {

class DxlHandleGroup {

public:

    DxlHandle& get(int id);
    DxlHandle& get(const char* name);

    bool hasHandle(int id);
    bool hasHandle(const char* name);

    std::vector<WriteCommand> getWriteCommands(bool clear = true);
    std::vector<ReadCommand> getReadCommands(bool clear = false);

    bool clearWriteCommands();
    bool clearReadCommands();

    DxlHandleGroup(){}

    bool addHandle(DxlHandle newHandle);

    std::vector<DxlHandle>& getHandles();

private:
    std::vector<DxlHandle> handles;

};

}

#endif
