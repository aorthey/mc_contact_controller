#include "mc_myfirst_controller.h"
#include <mc_rtc/logging.h>




namespace mc_control
{

        MCMyFirstController::MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
                  : MCController(robot_module, dt)
        {

                qpsolver->addConstraintSet(contactConstraint);
                qpsolver->addConstraintSet(kinematicsConstraint);
                qpsolver->addTask(postureTask.get());
                qpsolver->setContacts({});

                LOG_SUCCESS("MCMyFirstController robot loading " << robot().name());
                this->info();

                if(robot().name() == "hrp4"){
                        head_joint_index = robot().jointIndexByName("NECK_P");
                }else{
                        LOG_ERROR("This controller does not know how to handle the robot you are controlling (" << robot().name() << ")")
                        exit(0);
                }
                this->ros_bridge = mc_rtc::ROSBridge::get_node_handle();
                LOG_SUCCESS("MCMyFirstController init done " << this);


        }
        void MCMyFirstController::info()
        {
                //display robot info
                std::cout << "----- INFO -----" << std::endl;
                std::cout << "ROBOT:    " << robot().name() << std::endl;
                const rbd::MultiBody mb = robot().mb();
                std::cout << "#JOINTS : " << mb.nrJoints() << std::endl;
                std::cout << "#LINKS  : " << mb.nrBodies() << std::endl;
                const std::vector< rbd::Joint > joints = mb.joints();

                std::vector< rbd::Joint >::const_iterator iter;
                int jctr = 0;
                for( iter = joints.begin(); iter!=joints.end(); iter++){
                        std::cout << " [" << jctr++ << "]:" << (*iter).name() << std::endl;

                }
                std::cout << "----- INFO -----" << std::endl;
        }
        bool MCMyFirstController::run()
        {
                bool ret = MCController::run();
                std::vector<std::vector<double>> cur_obj = postureTask->posture();

                int head_joint_yaw = robot().jointIndexByName("NECK_P");
                int head_joint_pitch = robot().jointIndexByName("NECK_Y");
                cur_obj[head_joint_yaw][0] = robot().ql()[head_joint_yaw][0];
                cur_obj[head_joint_pitch][0] = robot().ql()[head_joint_pitch][0];

                postureTask->posture(cur_obj);
                return ret;
        }

        void MCMyFirstController::reset(const ControllerResetData & reset_data)
        {
                  MCController::reset(reset_data);
                  target_left = true;
                  switch_target();
        }

        void MCMyFirstController::switch_target()
        {
                  double target;
                  if(target_left) {
                        target = robot().qu()[head_joint_index][0];
                  } else {
                        target = robot().ql()[head_joint_index][0];
                  }
                  std::vector<std::vector<double>> cur_obj = postureTask->posture();
                  cur_obj[head_joint_index][0] = target;
                  postureTask->posture(cur_obj);
                  target_left = !target_left;
        }



}


