#include "mc_myfirst_controller.h"


namespace mc_control
{

        MCMyFirstController::MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
                  : MCController(robot_module, dt)
        {

                LOG_SUCCESS("MCMyFirstController robot loading " << robot().name());
                this->info();

                qpsolver->addConstraintSet(contactConstraint);
                //qpsolver->addConstraintSet(kinematicsConstraint);
                qpsolver->addConstraintSet(dynamicsConstraint);
                qpsolver->addTask(postureTask.get());

                comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex(), 2.0, 100.);
                qpsolver->addTask(comTask);

                postureTask->stiffness(1.);
                
                //qpsolver->setContacts({});
                //Set Contact on Surfaces
                qpsolver->setContacts({
                        {robots(), 0, 1, "LeftFoot", "AllGround"},
                        {robots(), 0, 1, "RightFoot", "AllGround"},
                        //{robots(), 0, 1, "LeftHand", "AllGround"},

                });



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

                std::cout << "#########################################" << std::endl;
                std::cout << "#JOINTS : " << mb.nrJoints() << std::endl;
                const std::vector< rbd::Joint > joints = mb.joints();
                std::vector< rbd::Joint >::const_iterator jiter;
                int jctr = 0;
                for( jiter = joints.begin(); jiter!=joints.end(); jiter++){
                        std::cout << " [" << jctr++ << "]:" << (*jiter).name() << std::endl;
                }

                std::cout << "#########################################" << std::endl;
                std::cout << "#LINKS  : " << mb.nrBodies() << std::endl;
                const std::vector< rbd::Body > bodies = mb.bodies();

                std::vector< rbd::Body >::const_iterator biter;
                int bctr = 0;
                for( biter = bodies.begin(); biter!=bodies.end(); biter++){
                        std::cout << " [" << bctr++ << "]:" << (*biter).name() << std::endl;
                }
                std::cout << "#########################################" << std::endl;
                std::cout << "#SURFACES" << std::endl;
                const std::map<std::string, mc_rbdyn::SurfacePtr> surfaces = robot().surfaces();
                std::map<std::string, mc_rbdyn::SurfacePtr>::const_iterator siter;
                int sctr = 0;
                for( siter = surfaces.begin(); siter!=surfaces.end(); siter++){
                        std::cout << " [" << sctr++ << "]:" << (*siter).first << std::endl;
                }
                std::cout << "#########################################" << std::endl;


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
                if(comTask->eval().norm() < 0.02) {
                        switch_target(); 
                }
                return ret;
        }

        void MCMyFirstController::reset(const ControllerResetData & reset_data)
        {
                //init task-COM to current COM
                MCController::reset(reset_data);

                
                comTask->reset();
                comZero = rbd::computeCoM(robot().mb(), robot().mbc());

                target_left = true;
                switch_target();
        }

        void MCMyFirstController::switch_target()
        {
                double posture_target;
                /// Posture task
                if(target_left) {
                        posture_target = robot().qu()[head_joint_index][0];
                } else {
                        posture_target = robot().ql()[head_joint_index][0];
                }
                std::vector<std::vector<double>> cur_obj = postureTask->posture();
                cur_obj[head_joint_index][0] = posture_target;
                postureTask->posture(cur_obj);

                /// COM task
                Eigen::Vector3d move(0., 0.12, 0.);
                Eigen::Vector3d target;

                if(target_left) {
                        target = comZero + move;
                }else{
                        target = comZero - move;
                }
                comTask->com(target);

                target_left = !target_left;

        }

}

