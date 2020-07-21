

#ifndef MODULAREVO_ROBOT_MANAGER_H
#define MODULAREVO_ROBOT_MANAGER_H


#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <map>

#include "CPG_NEAT_HYPERNEAT_Brain.h"

using namespace std;


namespace gazebo{
  class RobotManager : public ModelPlugin{
  protected:

          /// \brief this ID is used to distinguish from other instance of gzserve
          unsigned int ID = 0;

        /// \brief Pointer to the model
        physics::ModelPtr model;

        /// \brief Pointer to the model sdf
        sdf::ElementPtr sdf;


        /// \brief Node for transport used for handling the incoming messages
        transport::NodePtr nodeForSendingNNMatrix;
        /// \brief Subscriber for the incoming messages
        transport::PublisherPtr pubSenderMatrix;

        /// \brief Node for transport used for handling the incoming messages
        transport::NodePtr controllNode;
        /// \brief Subscriber for the incoming messages
        transport::SubscriberPtr sub;

        /// \brief Node for transport used for ACK messages
        transport::NodePtr nodeForSending;
        /// \brief Publisher for ACK messages
        transport::PublisherPtr pub;

        /// \brief Connector the simulator for handling the events
        event::ConnectionPtr updateConnection_;

        /// \brief position of the robot at the start of the multipleInstanceOpener
        ignition::math::Pose3d lastPosition;

        /// \brief time in which the robot is active (Maybe not used - to delete)
        double initTime_;
        /// \brief time interval of updating of the movements of the robot ( if is zero update in realtime)
        double actuationTime_ = 0;
        /// \brief time interval of the multipleInstanceOpener in seconds
        double testingTime_ = 10;
        /// \brief timestamp of the last update of the movements of the robot, used with actualtionTime_
        ::gazebo::common::Time lastActuationTime_ = 0;
        /// \brief timestamp of the last multipleInstanceOpener, used with testingTime_ to count the interval of Test
        ::gazebo::common::Time lastTestTime_ = 0;

        /// \brief pointer to the brain, ( it's possible to change it with an abstract class)
        CPG_NEAT_HYPERNEAT_Brain* brain_;

        /// \brief if it's true the simulation is active
        bool activeSimulation = false;

        /// \brief this string describe the kind of simulation, attached to the kind of brain
        experiment simulation;

        /// \brief this method report the fitness taken from the environment to the python manager
        void reportFitness(unsigned int kindOfParam, double fitness);

        /// \brief this is the external input derived from the environment
        vector<double> externalInput;


  public:
        ~RobotManager();
      /// Initialize all the Plugin
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      /// Handle the incoming messages
        void OnMsg(ConstVector3dPtr &_msg);
      /// Handler the changing environment of the simulator
        void CheckUpdate(const ::gazebo::common::UpdateInfo _info);

        void sendAck(experiment ack);
        void sendNN(vector<vector<bool>> A, vector<string> types);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotManager)
}


#endif //MODULAREVO_ROBOT_MANAGER_H