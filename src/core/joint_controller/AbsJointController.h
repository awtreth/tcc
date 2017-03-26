#ifndef ABS_JOINT_CONTROLLER_H
#define ABS_JOINT_CONTROLLER_H

#include <vector>
#include <string>
#include <memory>

/**
 * @brief
 *
 */
class AbsJointController {

    protected:

    std::vector<std::string> jointNames;

    public:

    AbsJointController(){
        jointNames = std::vector<std::string>();
    }

    AbsJointController(const std::vector<std::string> _jointNames){
        jointNames = _jointNames;
    }

    /**
     * @brief
     *
     * @return int
     */
    int nJoints() { return jointNames.size(); }

    std::vector<std::string> getJointNames() const{return jointNames;}

    void setJointNames(const std::vector<std::string> &value){jointNames = value;}
};

typedef std::shared_ptr<AbsJointController> JointControllerPtr;

#endif
