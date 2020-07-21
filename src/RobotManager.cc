
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

        this->nodeForSendingNNMatrix = transport::NodePtr(new transport::Node());
        this->nodeForSendingNNMatrix->Init("MatrixSender");
        this->pubSenderMatrix = this->nodeForSendingNNMatrix->Advertise<gazebo::msgs::GzString_V>("/gazebo/default/nnMatrixTopic");


        /// Create the event handler to manages the changing environments
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(bind(&RobotManager::CheckUpdate, this, _1));

        cout<<"[ROBOT] Connection handlers prepared."<<endl;
    }


    /**
     * This method is called to handle the incoming messages
     *
     * @param _msg
     */
    void RobotManager::OnMsg(ConstVector3dPtr &_msg){

        if( this->ID == 0) {
            this->ID = (unsigned int) _msg->x();
        }

        cout<<"[ROBOT] Request for the activation of a simulation."<<endl;
        if(!this->activeSimulation and _msg->x() == this->ID){
            if (_msg->y() == DIRECT) {
                cout<< "    Want to activate DIRECT Encoding."<<endl;

                /// send ack to the python manager
                sendAck(DIRECT_ACK);

                /// set the parameter of the connection
                this->simulation = DIRECT;

            }else if( _msg->y() == INDIRECT){
                cout<< "    Want to activate INDIRECT Encoding."<<endl;
                sendAck(INDIRECT_ACK);
                this->simulation = INDIRECT;

            }else if( _msg->y() == DIRECT_MOD){
                cout<< "    Want to activate DIRECT Encoding with modularity."<<endl;
                sendAck(DIRECT_MOD_ACK);
                this->simulation = DIRECT_MOD;

            }else if( _msg->y() == INDIRECT_MOD){
                cout<< "    Want to activate INDIRECT Encoding with modularity."<<endl;
                sendAck(INDIRECT_MOD_ACK);
                this->simulation = INDIRECT_MOD;

            }else{
                cerr<< "    Error: the kind of experiment chosen does not exists."<<endl;
                return;
            }

            externalInput.push_back(1);
            bool useMemory = _msg->z();

            stringstream buffer;
            buffer<< "bestGenome_"<<this->ID<<"_"<<this->simulation<<".txt";
            string storageName = buffer.str();

            /// Create the brain
            this->brain_ = new CPG_NEAT_HYPERNEAT_Brain(this->model, this->sdf, this->simulation, this->externalInput, storageName, storageName, useMemory);

            /// Activate the simulation
            this->activeSimulation = true;
            this->brain_->ActivateTheBrain();// start the brain

        }else{
            cout<<" Resend ACK."<<endl;
            sendAck(this->simulation);
            return;
        }

        cout<<"[ROBOT] Simulation ACTIVATED."<<endl;
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
                this->brain_->update(_info.simTime.Double());// update the brain
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

                vector<double> fitness = this->brain_->stepOfTest(distanceTravelled);/// set Fitness and go ahead with the simulation

                if(fitness[0]){
                    /// send back to the manager the best fitness of the population
                    reportFitness(58, fitness[0]);//send total fitness
                    reportFitness(59, fitness[1]);//send modularity
                    sendNN(this->brain_->getAdjacentMatrixOfLastBestGenome(), this->brain_->getTypesNeuronsOfLastBestGenome());

                }
                lastTestTime_ = _info.simTime;

                this->lastPosition = this->model->WorldPose();/// reset the new position of the robot
                if(!this->brain_->GetBrainState()){
                    this->activeSimulation = false;
                    cout<<"[ROBOT] THE SIMULATION IS FINISHED. CLOSING PROCEDURE."<<endl;
                    // todo send ack to python manager
                    sendAck(CLOSE);
                    usleep(1000);
                    client::shutdown();

                }
            }
            //this->activeSimulation = false; //for multipleInstanceOpener
        }


    }
    /**
     * Create a message with the fitness and report back
     *
     * @param fitness
     */
    void RobotManager::reportFitness(unsigned int kindOfParam, double fitness){
        this->pub->WaitForConnection();
        if(kindOfParam == 58){
            gazebo::msgs::Vector3d msg;
            gazebo::msgs::Set(&msg, ignition::math::Vector3d(this->ID, kindOfParam, fitness));
            this->pub->Publish(msg);
        }else if(kindOfParam == 59){
            gazebo::msgs::Vector3d msg;
            gazebo::msgs::Set(&msg, ignition::math::Vector3d(this->ID, kindOfParam, fitness));
            this->pub->Publish(msg);
        }


    }
    RobotManager::~RobotManager(){
        delete this->brain_;
    }
    void RobotManager::sendNN(vector<vector<bool>> A, vector<string> types){
        cout<<"[ROBOT] Sending NN structure."<<endl;
        this->pubSenderMatrix->WaitForConnection();

        gazebo::msgs::GzString_V msg;// Create Message
        string* ID = msg.add_data();
        *ID = to_string(this->ID);

        for(int i = 0; i < A.size(); i++){
            string temp = "";
            for(int j = 0; j < A[i].size(); j++){
                if( A[i][j] ){
                    if(!temp.compare("")){
                        temp = to_string(i) + " " + types[i]+ "  " + to_string(j);
                    }else{
                        temp += " " + to_string(j);
                    }
                }else{
                    temp = to_string(i) + " " + types[i];
                }
            }
            if(temp.compare("")){
                string* adjacentRow = msg.add_data();
                *adjacentRow = temp;
            }
        }

        this->pubSenderMatrix->Publish(msg);
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
        gazebo::msgs::Set(&msg, ignition::math::Vector3d(this->ID, ack, ack));
        this->pub->Publish(msg);

    }
}

