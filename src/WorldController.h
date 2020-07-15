//
// Created by gabriele on 01/07/20.
//

#ifndef EXPERIMENT_WORLDCONTROLLER_H
#define EXPERIMENT_WORLDCONTROLLER_H
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

using namespace std;
using namespace gazebo;
class WorldController : public WorldPlugin{


protected:
    /// \brief A node used for transport
    transport::NodePtr nodeForReceiving;
    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr sub;
    /// \brief Pointer to the model.
    physics::WorldPtr world;


    /// \brief a node used for sending
    transport::NodePtr nodeForSending;
    /// \brief a publisher used for response
    transport::PublisherPtr pub;


public:
    /// \brief Constructor
    WorldController() : WorldPlugin(){
        printf("World Is Started!\n");
    }
    /// \brief Initialize plugin
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    /// \brief handle messages
    void OnMsg(ConstVector3dPtr &_msg);
};


GZ_REGISTER_WORLD_PLUGIN(WorldController)
#endif //EXPERIMENT_WORLDCONTROLLER_H
