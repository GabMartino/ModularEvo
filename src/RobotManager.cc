
#include "RobotManager.h"


using namespace std;

namespace gazebo{

    /**
     *  This method is called when the model is charged in the simulator
     *
     */
    void RobotManager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
        cout<<"[ROBOT] Preparing Robot."<<endl;
        /// set the model
        this->model = _parent;
        this->sdf = _sdf;

        /// get the inititialization Time ( maybe useless, to delete)
        this->initTime_ = _parent->GetWorld()->SimTime().Double();

        /// get the first position of the model
        this->lastPosition = this->model->WorldPose();

        /// Create the API for the robot controll
        this->controllNode = transport::NodePtr(new transport::Node());
        this->controllNode->Init("robotManagerController");
        string controllTopicName = "/gazebo/default/robotManagerController";
        this->sub = this->controllNode->Subscribe(controllTopicName, &RobotManager::OnMsg, this);

        /// Create the API for the ACK
        this->nodeForSending = transport::NodePtr(new transport::Node());
        this->nodeForSending->Init();
        this->pub = this->nodeForSending->Advertise<gazebo::msgs::Vector3d>("~/responseTopic");


        /// Create the event handler to manages the changing environments
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(bind(&RobotManager::CheckUpdate, this, _1));

        cout<<"[ROBOT] Connection handlers prepared."<<endl;
    }


    /**
     * This method is called to handle the incoming messages
     *
     * @param _msg
     */
    void RobotManager::OnMsg(ConstVector3dPtr &_msg) {
        cout<<"[ROBOT] Request for the activation of a simulation."<<endl;
        if(!this->activeSimulation){
            if (_msg->x() == DIRECT) {
                cout<< "    Want to activate DIRECT Encoding."<<endl;

                /// send ack to the python manager
                sendAck(DIRECT_ACK);

                /// set the parameter of the connection
                this->simulation = DIRECT;

            }else if( _msg->x() == INDIRECT){
                cout<< "    Want to activate INDIRECT Encoding."<<endl;
                sendAck(INDIRECT_ACK);
                this->simulation = INDIRECT;

            }else if( _msg->x() == DIRECT_MOD){
                cout<< "    Want to activate DIRECT Encoding with modularity."<<endl;
                sendAck(DIRECT_MOD_ACK);
                this->simulation = DIRECT_MOD;

            }else if( _msg->x() == INDIRECT_MOD){
                cout<< "    Want to activate INDIRECT Encoding with modularity."<<endl;
                sendAck(INDIRECT_MOD_ACK);
                this->simulation = INDIRECT_MOD;

            }else{
                cerr<< "    Error: the kind of experiment chosen does not exists."<<endl;
                return;
            }

            externalInput.push_back(1);

            /// Create the brain
            this->brain_ = new CPG_NEAT_HYPERNEAT_Brain(this->model, this->sdf, this->simulation, this->externalInput);

            /// Activate the simulation
            this->activeSimulation = true;


        }else{
            cout<<" Resend ACK."<<endl;
            sendAck(this->simulation);
        }


    }

    /**
     * This method is called at every update of the simulator
     *
     */
    void RobotManager::CheckUpdate(const ::gazebo::common::UpdateInfo _info){
        if(this->activeSimulation){/// The simulation should be active

            /**
             * if actuationTime_ == 0 the brain is updated at every time step
             *
             */
            auto diff = _info.simTime - lastActuationTime_;
            if (diff.Double() > actuationTime_){
                this->brain_->update(_info, lastActuationTime_);// update the brain
                lastActuationTime_ = _info.simTime;
            }

            /**
             * testingTime_ is the time interval in which the robot is evaluated.
             *
             */
            auto diffTest = _info.simTime - lastTestTime_;
            if (diffTest.Double() > testingTime_){
                //distanceTravelled is the fitness value
                /// USE ONLY PLANE DISTANCE
                double distanceTravelled = distance(this->model->WorldPose(), this->lastPosition);

                distanceTravelled = distanceTravelled > 1.0 ? 1 : distanceTravelled;// normalize in a range [0,1]

                double fitness = this->brain_->stepOfTest(_info, distanceTravelled);/// set Fitness and go ahead with the simulation

                if(fitness){
                    /// send back to the manager the best fitness of the population
                    reportFitness(fitness);
                }
                lastTestTime_ = _info.simTime;

                this->lastPosition = this->model->WorldPose();/// reset the new position of the robot
            }
            //this->activeSimulation = false; //for test
        }


    }
    /**
     * Create a message with the fitness and report back
     *
     * @param fitness
     */
    void RobotManager::reportFitness(double fitness){
        this->pub->WaitForConnection();
        gazebo::msgs::Vector3d msg;
        gazebo::msgs::Set(&msg, ignition::math::Vector3d(58, 58, fitness));
        this->pub->Publish(msg);

    }
    /**
     * Create a message with the ack and send back
     *
     * @param ack
     */
    void RobotManager::sendAck(experiment ack){
        ///Prepare and send the ACK to the PythonManager
        this->pub->WaitForConnection();
        gazebo::msgs::Vector3d msg;
        gazebo::msgs::Set(&msg, ignition::math::Vector3d(ack, ack, ack));
        this->pub->Publish(msg);

    }
}

