//
// Created by gabriele on 01/07/20.
//

#ifndef MODULAREVO_JOINTPOSITIONMOTOR_H
#define MODULAREVO_JOINTPOSITIONMOTOR_H

#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
using namespace gazebo;
using namespace std;
class JointPositionMotor {

    physics::JointPtr joint_;

    /// \brief Last update time, used to determine update step time
    protected: ::gazebo::common::Time prevUpdateTime_;
    /// \brief PID that controls this motor
    protected: ::gazebo::common::PID pid_;

    /// \brief Current position target
    protected: double positionTarget_;

        /// \brief Upper and lower position limits
    protected: double lowerLimit_;

        /// \brief
    protected: double upperLimit_;

        /// \brief Whether this joint can achieve a full range of motion,
        /// meaning it can flip from a positive to a negative angle. This is
        /// set to true whenever the total range is >/ 2 pi.
    protected: bool fullRange_;
    private: double noise_ = 0.01;
    public: JointPositionMotor(physics::JointPtr joint);

    public: void update(double value);
};


#endif //MODULAREVO_JOINTPOSITIONMOTOR_H
