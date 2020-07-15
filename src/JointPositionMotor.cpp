//
// Created by gabriele on 01/07/20.
//

#include "JointPositionMotor.h"


JointPositionMotor::JointPositionMotor(physics::JointPtr joint){
    this->joint_ = joint;
    this->upperLimit_ = std::fmin(M_PI, this->joint_->UpperLimit(0));
    this->lowerLimit_ = std::fmax(-M_PI, this->joint_->LowerLimit(0));
    this->fullRange_ = ((this->upperLimit_ - this->lowerLimit_ + 1e-12) >=
                        (2 * M_PI));
    auto pv = 1;
    auto iv = 0.1;
    auto dv = 0.01;
    auto iMax = 10;
    auto iMin = 0.0;
    auto cmdMax = 10.0;
    auto cmdMin = -10.0;
    this->pid_ = common::PID(pv, iv, dv, iMax, iMin, cmdMax, cmdMin);
    auto maxEffort = joint_->GetEffortLimit(0);
    joint_->SetParam("fmax", 0, maxEffort);
}

void JointPositionMotor::update(double value){

    // Motor noise in range +/- noiseLevel * actualValue
    value += ((2 * ignition::math::Rand::DblUniform() * this->noise_) -
               this->noise_) *
            value;
    value = std::fmin(std::fmax(1e-5, value), 0.99999);

    this->positionTarget_ = this->lowerLimit_ +
                            (value * (this->upperLimit_ - this->lowerLimit_));
    auto stepTime = this->joint_->GetWorld()->SimTime() - this->prevUpdateTime_;

    if (stepTime <= 0)
    {
        // Only respond to positive step times
        return;
    }
    this->prevUpdateTime_ = this->joint_->GetWorld()->SimTime();
    auto position = this->joint_->Position(0);

    // TODO Make sure normalized angle lies within possible range
    // I get the feeling we might be moving motors outside their
    // allowed range. Also something to be aware of when setting
    // the direction.

    if (this->fullRange_ and std::fabs(position - positionTarget_) > M_PI)
    {
        // Both the position and the position target will be in the range
        // [-pi, pi]. For a full range of motion joint, using an angle +- 2 PI
        // might result in a much shorter required movement. In this case we
        // best correct the current position to something outside the range.
        position += (position > 0 ? -2 * M_PI : 2 * M_PI);
    }

    auto error = position - this->positionTarget_;
    auto cmd = this->pid_.Update(error, stepTime);
    this->joint_->SetParam("vel", 0, cmd);
}