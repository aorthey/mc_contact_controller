#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>
#include <mc_rtc/ros.h>
#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/MoveContactTask.h>
#include <mc_tasks/StabilityTask.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/Surface.h>

namespace mc_control
{

        struct MC_CONTROL_DLLAPI MCMyFirstController : public MCController
        {
                public:
                        MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);
                        virtual bool run() override;
                        virtual void reset(const ControllerResetData & reset_data) override;
                        void info();
                        void moveJointByName(std::string joint_name);
                private:
                        std::shared_ptr<ros::NodeHandle> ros_bridge;

                        int head_joint_index;
                        bool target_left;
                        Eigen::Vector3d comZero;



                        bool moved_left_foot = false;
                        bool moved_left_foot_to_wp = false;
                        // std::shared_ptr<mc_solver::BoundedSpeedConstr> bSpeedConstr;
                        // std::shared_ptr<mc_tasks::AddRemoveContactTask> aRContactTask;
                        bool removed_left_foot = false;
                        bool added_left_foot = false;

                        std::shared_ptr<mc_tasks::EndEffectorTask> task_left_hand;
                        std::shared_ptr<mc_tasks::EndEffectorTask> task_right_hand;
                        sva::PTransformd right_hand_current_pose;
                        sva::PTransformd left_hand_current_pose;

                        // std::shared_ptr<mc_tasks::CoMTask> comTask;
                        // std::shared_ptr<mc_tasks::StabilityTask> stableTask;
                        // std::shared_ptr<mc_tasks::MoveContactTask> mcTask;

                        // std::shared_ptr<mc_tasks::EndEffectorTask> footTask;

                        bool moved_com = false;
                        bool moved_right_foot = false;






        };


}
SIMPLE_CONTROLLER_CONSTRUCTOR("MyFirst", mc_control::MCMyFirstController)
