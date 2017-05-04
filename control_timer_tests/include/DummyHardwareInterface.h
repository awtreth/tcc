#ifndef DUMMY_HARDWARE_INTERFACE_H
#define DUMMY_HARDWARE_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <IHardwareInterface.h>
#include <string>
#include <mutex>
#include <memory>

class DummyHardwareInterface : public IHardwareInterface {

protected:
    std::mutex mtx;
    std::string msg = "";

    void defaultRead();

    void defaultWrite();

public:
    DummyHardwareInterface();
    virtual ~DummyHardwareInterface(){}

    // IHardwareInterface interface

    virtual bool read();

    virtual bool write();

    std::string getMsg();

    void setMsg(const std::string &value);

    // RobotHW interface
public:
//    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
//    bool checkForConflict(const std::list<hardware_interface::ControllerInfo> &info) const;
//    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
//    void doSwitch(const std::list<hardware_interface::ControllerInfo> &, const std::list<hardware_interface::ControllerInfo> &);
};

typedef std::shared_ptr<DummyHardwareInterface> DummyHardwareInterfacePtr;


#endif
