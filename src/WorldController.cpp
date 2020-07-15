//
// Created by gabriele on 01/07/20.
//

#include "WorldController.h"

/**
 *  On Load of the World Plugin
 *
 * @param _world
 * @param _sdf
 */

void WorldController::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
    //INITIALIZING RECEVING TOPIC
    cout<<"[WORLD] Prepare the world."<<endl;
    this->world = _world;
    this->nodeForReceiving = transport::NodePtr(new transport::Node());
    this->nodeForReceiving->Init(this->world->Name());
    // Create a topic name
    std::string topicName = "~/receiveModels";
    // Subscribe to the topic, and register a callback
    this->sub = this->nodeForReceiving->Subscribe(topicName, &WorldController::OnMsg, this);


    //INITIALIZING RESPONSE TOPIC
    this->nodeForSending = transport::NodePtr(new transport::Node());
    this->nodeForSending->Init();
    this->pub = this->nodeForSending->Advertise<gazebo::msgs::Vector3d>("/gazebo/default/responseTopic");

    cout<<"[WORLD] Connections handlers prepared."<<endl;

}
/**
 * On receiving a new message on the topic set in method OnLoad
 *
 *
 * @param _msg
 */
void WorldController::OnMsg(ConstVector3dPtr &_msg){
    cout<< "[WORLD] Number of models " <<this->world->ModelCount()<<endl;
    if( this->world->ModelCount() < 2){
        cout<< "A new Model will be inserted"<< endl;
        this->world->InsertModelFile("model://Spider");

    }
    //Sending ack to the Python Manager
    cout<<"[WORLD] Sending ACK of inserting model"<<endl;

    this->pub->WaitForConnection();
    gazebo::msgs::Vector3d msg;// Create Message
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(37, 37, 37)); // 37 is the ACK for the insertion of a robot
    this->pub->Publish(msg);//publish

}