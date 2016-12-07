#include "mc_myfirst_controller.h"


namespace mc_control
{

        MCMyFirstController::MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
                  : MCController(robot_module, dt)
        {

                this->info();

                //*************************************************************
                // check that right robot is loaded
                //*************************************************************
                LOG_SUCCESS("MCMyFirstController robot loading " << robot().name());
                if(robot().name() != "hrp4"){
                        LOG_ERROR("This controller does not know how to handle the robot you are controlling (" << robot().name() << ")")
                        exit(0);
                }

                //*************************************************************
                qpsolver->addConstraintSet(contactConstraint);
                qpsolver->addConstraintSet(dynamicsConstraint);
                qpsolver->addConstraintSet(kinematicsConstraint);

                //*************************************************************
                //Posture task
                //*************************************************************
                postureTask->stiffness(1.);
                qpsolver->addTask(postureTask.get());

                
                //*************************************************************
                //COM task
                //*************************************************************
                //comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex(), 2.0, 100.);
                //qpsolver->addTask(comTask);

                //*************************************************************
                //Stability Task
                //*************************************************************
                // stableTask = std::make_shared<mc_tasks::StabilityTask>(robots());
                // qpsolver->addTask(stableTask);

                //*************************************************************
                //Endeffector Task
                //*************************************************************
                //left_hand = std::make_shared<mc_tasks::EndEffectorTask>("L_HAND_J1_LINK", robots(), robots().robotIndex(), 5.0, 10.);
                double stiffness = 5.0;
                double task_weight = 10.0;
                task_right_hand = std::make_shared<mc_tasks::EndEffectorTask>("R_F23_LINK",
                        robots(), robots().robotIndex(),
                        stiffness, task_weight);
                qpsolver->addTask(task_right_hand);
                task_left_hand = std::make_shared<mc_tasks::EndEffectorTask>("L_F23_LINK",
                        robots(), robots().robotIndex(),
                        stiffness, task_weight);
                qpsolver->addTask(task_left_hand);

                //*************************************************************
                //Set Contact task
                //*************************************************************
                qpsolver->setContacts({
                        {robots(), 0, 1, "LeftFoot", "AllGround"},
                        {robots(), 0, 1, "RightFoot", "AllGround"},
                        //{robots(), 0, 1, "LeftHand", "AllGround"},
                });


                //this->moveJointByName("NECK_P");
                //this->moveJointByName("NECK_Y");

                
                // qpsolver->addConstraintSet(selfCollisionConstraint);

                // selfCollisionConstraint.addCollisions(qpsolver, {
                //         {"RARM_LINK5", "RLEG_LINK2", 0.05, 0.01, 0.},
                //         {"RARM_LINK6", "RLEG_LINK2", 0.05, 0.01, 0.},
                //         {"RARM_LINK7", "RLEG_LINK2", 0.05, 0.01, 0.}
                // });
                
                // this->ros_bridge = mc_rtc::ROSBridge::get_node_handle();
                // LOG_SUCCESS("MCMyFirstController init done " << this);


        }

        void MCMyFirstController::moveJointByName(std::string joint_name){
                std::vector<std::vector<double>> cur_obj = postureTask->posture();
                int jind = robot().jointIndexByName(joint_name);
                cur_obj[jind][0] = robot().qu()[jind][0];
                postureTask->posture(cur_obj);
        }
        bool MCMyFirstController::run()
        {
                bool ret = MCController::run();
                //*************************************************************
                // Posture Task: move joint J to position 
                // (theta_des) \in R
                //*************************************************************
                this->moveJointByName("NECK_P");
                this->moveJointByName("NECK_Y");
                this->moveJointByName("L_ELBOW_P");

                //*************************************************************
                // Endeffectortask: Move endeffector EE to position
                // (x,y,z) \in R^3 x SO(3) = SE(3)
                //*************************************************************
                sva::PTransformd desired_pose(Eigen::Vector3d(0.5, -0.5, 1.0));
                task_right_hand->set_ef_pose(desired_pose);

                std::cout<< "[EE TASK] right hand dist to target:" <<
                        task_right_hand->eval().norm() <<std::endl;

                //*************************************************************
                // Move Contact Task
                //*************************************************************
                // if(!moved_com)
                // {
                //         if(comTask->eval().norm() < 5e-2 && comTask->speed().norm() < 1e-4)
                //         {
                //                 LOG_SUCCESS("Moved the CoM")
                //                 moved_com = true;
                //                 qpsolver->setContacts({
                //                         mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
                //                 });
                //                 qpsolver->addTask(efTask);
                //                 efTask->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0.4, 0, 0)));
                //         }
                //         return ret;
                // }
                // if(moved_com && !moved_left_foot)
                // {
                //         if(efTask->eval().norm() < 5e-2 && efTask->speed().norm() < 1e-4)
                //         {
                //                 LOG_SUCCESS("Moved the left foot")
                //                 moved_left_foot = true;
                //                 qpsolver->setContacts({
                //                 mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
                //                 mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
                //                 });
                //                 qpsolver->removeTask(efTask);
                //                 auto com = comTask->com();

                //                 com.x() =
                //                 (robot().mbc().bodyPosW[robot().bodyIndexByName("LLEG_LINK5")].translation().x()
                //                 +
                //                 robot().mbc().bodyPosW[robot().bodyIndexByName("RLEG_LINK5")].translation().x())/2;

                //                 com.y() = 0;
                //                 comTask->com(com);
                //         }
                //         return ret;
                // }
                return ret;

        }

        void MCMyFirstController::reset(const ControllerResetData & reset_data)
        {
                //init task-COM to current COM
                MCController::reset(reset_data);
                //task_left_hand->reset();
                task_right_hand->reset();
                //right_hand_current_pose = task_right_hand->get_ef_pose();

                //comTask->reset();
                //stableTask->reset();

                //comZero = rbd::computeCoM(robot().mb(), robot().mbc());
                //transformZero = efTask->get_ef_pose();

                // comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);

                //efTask = std::make_shared<mc_tasks::EndEffectorTask>("LLEG_LINK5", robots(), 0);
                // comTask->com(comTask->com() + Eigen::Vector3d(0,
                //                               -1*efTask->get_ef_pose().translation().y(),
                //                               0));
                //qpsolver->addTask(comTask);
        }

        // void MCMyFirstController::display_joints()
        // void MCMyFirstController::display_bodies()
        // void MCMyFirstController::display_surfaces()
        void MCMyFirstController::info()
        {
        
                //*************************************************************
                //display robot info
                //*************************************************************
                std::cout << "----- INFO START " << std::string(63, '-') << std::endl;
                std::cout << "ROBOT:    " << robot().name() << std::endl;
                const rbd::MultiBody mb = robot().mb();
                //*************************************************************
                std::cout << "#########################################" << std::endl;
                std::cout << "#JOINTS : " << mb.nrJoints() << std::endl;
                const std::vector< rbd::Joint > joints = mb.joints();
                std::vector< rbd::Joint >::const_iterator jiter;
                int jctr = 0;
                for( jiter = joints.begin(); jiter!=joints.end(); jiter++){
                        std::cout << " [" << jctr++ << "]:" << (*jiter).name() << std::endl;
                }
                //*************************************************************
                std::cout << "#########################################" << std::endl;
                std::cout << "#LINKS  : " << mb.nrBodies() << std::endl;
                const std::vector< rbd::Body > bodies = mb.bodies();

                std::vector< rbd::Body >::const_iterator biter;
                int bctr = 0;
                for( biter = bodies.begin(); biter!=bodies.end(); biter++){
                        std::cout << " [" << bctr++ << "]:" << (*biter).name() << std::endl;
                }
                //*************************************************************
                std::cout << "#########################################" << std::endl;
                std::cout << "#SURFACES" << std::endl;
                const std::map<std::string, mc_rbdyn::SurfacePtr> surfaces = robot().surfaces();
                std::map<std::string, mc_rbdyn::SurfacePtr>::const_iterator siter;
                int sctr = 0;
                for( siter = surfaces.begin(); siter!=surfaces.end(); siter++){
                        std::cout << " [" << sctr++ << "]:" << (*siter).first << std::endl;
                }
                std::cout << "#########################################" << std::endl;
                //*************************************************************


                std::cout << "----- INFO END   " << std::string(63, '-') << std::endl;
        }
}

