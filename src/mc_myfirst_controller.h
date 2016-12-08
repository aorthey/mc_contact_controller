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
        enum ContactState{
                PRE_CONTACT_BREAK=0,
                CONTACT_BREAK,
                ENDEFFECTOR_TRANSITION,
                CONTACT_MAKE,
                POST_CONTACT_MAKE
        };

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

                        Eigen::Vector3d comZero;

                        //specification of contact surfaces
                        std::shared_ptr<mc_tasks::EndEffectorTask> task_left_hand;
                        std::shared_ptr<mc_tasks::EndEffectorTask> task_right_hand;
                        sva::PTransformd left_hand_current_pose;
                        sva::PTransformd right_hand_current_pose;

                        std::shared_ptr<mc_tasks::EndEffectorTask> task_left_foot;
                        std::shared_ptr<mc_tasks::EndEffectorTask> task_right_foot;
                        sva::PTransformd left_foot_current_pose;
                        sva::PTransformd right_foot_current_pose;

                        std::shared_ptr<mc_rbdyn::Contact> contact_right_foot;
                        std::shared_ptr<mc_rbdyn::Contact> contact_left_foot;

                        ContactState contact_state;

                        // std::shared_ptr<mc_solver::BoundedSpeedConstr> bSpeedConstr;
                        // std::shared_ptr<mc_tasks::AddRemoveContactTask> aRContactTask;

                        std::shared_ptr<mc_tasks::CoMTask> comTask;
                        // std::shared_ptr<mc_tasks::StabilityTask> stableTask;
                        // std::shared_ptr<mc_tasks::MoveContactTask> mcTask;
                        // std::shared_ptr<mc_tasks::EndEffectorTask> footTask;
        };


}
SIMPLE_CONTROLLER_CONSTRUCTOR("MyFirst", mc_control::MCMyFirstController)
