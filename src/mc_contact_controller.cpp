#include "mc_contact_controller.h"

namespace mc_control
{

        MCContactController::MCContactController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
                  : MCController(robot_module, dt)
        {
                //  virtual mc_rbdyn::Robot & robot();
                //  virtual mc_rbdyn::Robot & env();

                this->info();

                //*************************************************************
                // check that right robot is loaded
                //*************************************************************
                LOG_SUCCESS("MCContactController robot loading " << robot().name());
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
                //Stability Task
                //*************************************************************
                // stableTask = std::make_shared<mc_tasks::StabilityTask>(robots());
                // qpsolver->addTask(stableTask);

                //*************************************************************
                //Endeffector Task
                //*************************************************************
                double stiffness = 5.0;
                double task_weight = 10.0;
                task_right_hand = std::make_shared<mc_tasks::EndEffectorTask>("R_F23_LINK",
                        robots(), robots().robotIndex(),
                        stiffness, task_weight);

                //qpsolver->addTask(task_right_hand);

                task_left_hand = std::make_shared<mc_tasks::EndEffectorTask>("L_F23_LINK",
                        robots(), robots().robotIndex(),
                        stiffness, task_weight);

                //qpsolver->addTask(task_left_hand);


                LOG_SUCCESS("Added Hand EndEffectorTask");

                // Note: A EE-task is based on LINKS, while a contact is based on
                // surfaces!
                task_left_foot  = std::make_shared<mc_tasks::EndEffectorTask>
                        ("l_sole", 
                        robots(), 
                        robots().robotIndex(),
                        stiffness, 
                        task_weight);
                
                contact_left_foot = std::make_shared<mc_rbdyn::Contact>
                        (robots(), 
                         "LeftFoot", 
                         "AllGround");

                task_right_foot  = std::make_shared<mc_tasks::EndEffectorTask>
                        ("r_sole", 
                        robots(), 
                        robots().robotIndex(),
                        stiffness, 
                        task_weight);

                contact_right_foot = std::make_shared<mc_rbdyn::Contact>
                        (robots(), 
                         "RightFoot", 
                         "AllGround");

                //*************************************************************
                //Set Contact task
                //*************************************************************
                qpsolver->setContacts({*contact_left_foot, *contact_right_foot});

                contact_state = PRE_CONTACT_BREAK;

                
                // qpsolver->addConstraintSet(selfCollisionConstraint);
                // selfCollisionConstraint.addCollisions(qpsolver, {
                //         {"RARM_LINK5", "RLEG_LINK2", 0.05, 0.01, 0.},
                //         {"RARM_LINK6", "RLEG_LINK2", 0.05, 0.01, 0.},
                //         {"RARM_LINK7", "RLEG_LINK2", 0.05, 0.01, 0.}
                // });
                
                // this->ros_bridge = mc_rtc::ROSBridge::get_node_handle();
                // LOG_SUCCESS("MCContactController init done " << this);


        }

        void MCContactController::moveJointByName(std::string joint_name){
                std::vector<std::vector<double>> cur_obj = postureTask->posture();
                int jind = robot().jointIndexByName(joint_name);
                cur_obj[jind][0] = robot().qu()[jind][0];
                postureTask->posture(cur_obj);
        }
        bool MCContactController::run()
        {
                bool ret = MCController::run();
                //*************************************************************
                // Posture Task: move joint J to position 
                // (theta_des) \in R
                //*************************************************************
                //this->moveJointByName("NECK_P");
                //this->moveJointByName("NECK_Y");
                //this->moveJointByName("L_ELBOW_P");

                //*************************************************************
                // Endeffectortask: Move endeffector EE to position
                // (x,y,z) \in R^3 x SO(3) = SE(3)
                //*************************************************************
                //sva::PTransformd desired_pose(Eigen::Vector3d(0.5, -0.5, 1.0));
                //task_right_hand->set_ef_pose(desired_pose);

                //*************************************************************
                // Move Contact Task
                //*************************************************************
                double speed = 0.1;
                double stiffness = 2.0;
                double weight = 1000;
                double lf_height = 0.05;

                double lf_z = robot().mbc().bodyPosW[robot().bodyIndexByName("l_sole")].translation().z();
                switch (contact_state) {
                        case PRE_CONTACT_BREAK:
                                 //make sure COM is safe before removing contact
                                qpsolver->setContacts({*contact_left_foot, *contact_right_foot});
                                this->display_com();
                                std::cout << "PRE_CONTACT_BREAK:" << comTask->eval().norm() << "|" << comTask->speed().norm() << std::endl;
                                if(comTask->eval().norm() < 1e-3){
                                        LOG_SUCCESS("Moved COM.");
                                        LOG_SUCCESS("Starting contact Transition.");
                                        qpsolver->setContacts({*contact_right_foot});
                                        aRContactTask.reset(
                                                new mc_tasks::RemoveContactTask(robots(),
                                                                  bSpeedConstr,
                                                                  *contact_left_foot,
                                                                  speed,
                                                                  stiffness,
                                                                  weight)
                                        );
                                        qpsolver->addTask(aRContactTask);
                                        this->left_foot_z = task_left_foot->get_ef_pose().translation().z();
                                        //#### CONTINUE #########
                                        contact_state=CONTACT_BREAK;
                                }
                                break;
                        case CONTACT_BREAK:
                                // contact is broken if 

                                //double left_foot_z_current = task_left_foot->get_ef_pose().translation().z();

                                LOG_SUCCESS("Removing left foot contact " << lf_z);
                                //double lf_z = robot().mbc().bodyPosW[robot().bodyIndexByName("l_sole")].translation().z();
                                if( lf_z > this->left_foot_z + lf_height){
                                        LOG_SUCCESS("Left foot contact removed")
                                        qpsolver->removeTask(aRContactTask);

                                        mc_rbdyn::Contact targetContact(robots(), "LeftFoot", "AllGround", 
                                                        sva::PTransformd(Eigen::Vector3d(0.2, 0.1, 0)));

                                        //double posStiffness, 
                                        //double extraPosStiffness, 
                                        //double posWeight, 
                                        //double oriStiffness, 
                                        //double oriWeight, 
                                        //double preContactDist,
                                        task_move_contact = std::make_shared<mc_tasks::MoveContactTask>(robots(),
                                                         robot(),
                                                         env(),
                                                         targetContact,
                                                         5.0, 5.0, 1000,
                                                         3.0, 500,
                                                         lf_height,
                                                         mc_rbdyn::percentWaypoint(0.5, 0.5, 0.5, 0.1));
                                        qpsolver->addTask(task_move_contact);
                                        task_move_contact->toWaypoint();
                                        //#### CONTINUE #########
                                        contact_state=ENDEFFECTOR_TRANSITION;
                                }

                                break;
                        case ENDEFFECTOR_TRANSITION:
                                // move endeffector towards waypoint
                                // (transfer path)
                                if(task_move_contact->eval().norm() < 5e-2){
                                        LOG_SUCCESS("Moved the left foot to waypoint")
                                        task_move_contact->toPreEnv();
                                        contact_state = CONTACT_MAKE;
                                }
                                break;
                        case CONTACT_MAKE:
                                // from waypoint towards contact
                                if(task_move_contact->speed().norm() < 1e-4) {
                                        qpsolver->removeTask(task_move_contact);

                                        aRContactTask.reset(
                                                new mc_tasks::AddContactTask(robots(),
                                                bSpeedConstr,
                                                *contact_left_foot,
                                                speed,
                                                stiffness, 
                                                weight)
                                        );
                                        qpsolver->addTask(aRContactTask);
                                        contact_state = POST_CONTACT_MAKE;
                                }
                                break;
                        case POST_CONTACT_MAKE:
                                // make contact
                                LOG_SUCCESS("Putting down left foot contact " << lf_z);
                                //double lf_z = robot().mbc().bodyPosW[robot().bodyIndexByName("l_sole")].translation().z();
                                if( lf_z < 0.01){
                                        qpsolver->removeTask(aRContactTask);
                                        qpsolver->setContacts({*contact_left_foot, *contact_right_foot});
                                        contact_state = FINISHED_CONTACT_TRANSITION;
                                }
                                break;
                        case FINISHED_CONTACT_TRANSITION:
                                //LOG_SUCCESS("Contact Transition completed.")
                                break;
                }
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

        void MCContactController::display_joints( const rbd::MultiBody &mb){
                std::cout << "#JOINTS : " << mb.nrJoints() << std::endl;
                const std::vector< rbd::Joint > joints = mb.joints();
                std::vector< rbd::Joint >::const_iterator jiter;
                int jctr = 0;
                for( jiter = joints.begin(); jiter!=joints.end(); jiter++){
                        std::cout << " [" << jctr++ << "]:" << (*jiter).name() << std::endl;
                }
        }
        void MCContactController::display_bodies(const rbd::MultiBody &mb){
                std::cout << "#LINKS  : " << mb.nrBodies() << std::endl;
                const std::vector< rbd::Body > bodies = mb.bodies();

                std::vector< rbd::Body >::const_iterator biter;
                int bctr = 0;
                for( biter = bodies.begin(); biter!=bodies.end(); biter++){
                        std::cout << " [" << bctr++ << "]:" << (*biter).name() << std::endl;
                }
        }
        void MCContactController::display_surfaces( const std::map<std::string, mc_rbdyn::SurfacePtr> &surfaces){
                std::cout << "#SURFACES" << std::endl;
                std::map<std::string, mc_rbdyn::SurfacePtr>::const_iterator siter;
                int sctr = 0;
                for( siter = surfaces.begin(); siter!=surfaces.end(); siter++){
                        std::cout << " [" << sctr++ << "]:" << (*siter).first << std::endl;
                }
        }
        void MCContactController::display_com(){
                Eigen::Vector3d cur_com = rbd::computeCoM(robot().mb(), robot().mbc());
                std::cout << "COM:  (" << cur_com.x() << "," << cur_com.y() << "," << cur_com.z() << ")" << std::endl;
        }
        void MCContactController::info()
        {
        
                //  virtual mc_rbdyn::Robot & robot();
                //  virtual mc_rbdyn::Robot & env();
                //*************************************************************
                //display robot info
                //*************************************************************
                std::cout << "----- INFO START " << std::string(63, '-') << std::endl;
                std::cout << "ROBOT:    " << robot().name() << std::endl;
                //*************************************************************
                std::cout << std::string(80,'#') << std::endl;
                this->display_joints(robot().mb());
                //*************************************************************
                std::cout << std::string(80,'#') << std::endl;
                this->display_bodies(robot().mb());
                //*************************************************************
                std::cout << std::string(80,'#') << std::endl;
                this->display_surfaces(robot().surfaces());
                //*************************************************************
                //*************************************************************
                std::cout << std::string(80,'#') << std::endl;
                std::cout << "ENVIRONMENT:" << env().name() << std::endl;
                //*************************************************************
                std::cout << std::string(80,'#') << std::endl;
                this->display_joints(env().mb());
                //*************************************************************
                std::cout << std::string(80,'#') << std::endl;
                this->display_bodies(env().mb());
                //*************************************************************
                std::cout << std::string(80,'#') << std::endl;
                this->display_surfaces(env().surfaces());
                //*************************************************************
                this->display_com();
                std::cout << "----- INFO END   " << std::string(63, '-') << std::endl;
        }
        void MCContactController::reset(const ControllerResetData & reset_data)
        {
                //init task-COM to current COM
                MCController::reset(reset_data);

                task_left_hand->reset();
                task_right_hand->reset();
                task_left_foot->reset();
                task_right_foot->reset();

                left_hand_current_pose = task_left_hand->get_ef_pose();
                right_hand_current_pose = task_right_hand->get_ef_pose();

                left_foot_current_pose = task_left_foot->get_ef_pose();
                right_foot_current_pose = task_right_foot->get_ef_pose();

                //*************************************************************
                //COM task
                //*************************************************************
                //comTask->reset();
                comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
                Eigen::Vector3d T = task_left_foot->get_ef_pose().translation();
                comTask->com(comTask->com() + Eigen::Vector3d(0, -T.y(), 0));
                qpsolver->addTask(comTask);

                bSpeedConstr = std::make_shared<mc_solver::BoundedSpeedConstr>(robots(), 0, timeStep);
                qpsolver->addConstraintSet(*bSpeedConstr);
                //stableTask->reset();

                //comZero = rbd::computeCoM(robot().mb(), robot().mbc());
                //transformZero = efTask->get_ef_pose();

                //efTask = std::make_shared<mc_tasks::EndEffectorTask>("LLEG_LINK5", robots(), 0);
                // comTask->com(comTask->com() + Eigen::Vector3d(0,
                //                               -1*efTask->get_ef_pose().translation().y(),
                //                               0));
        }

}

