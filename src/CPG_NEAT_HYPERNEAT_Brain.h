//
// Created by gabriele on 24/06/20.
//

#ifndef EXPERIMENT_ACTUATORSCONTROLLER_H
#define EXPERIMENT_ACTUATORSCONTROLLER_H

#include <map>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "Brain.h"
#include "JointPositionMotor.h"
#include "NeuralNetwork.h"
#include "Learner_NEAT_HyperNEAT.h"
#include <multineat/MultiNEAT.h>
using namespace std;
using namespace gazebo;



/**
 * This class implements the Brain in which the robot is represented by a NeuralNetwork Class
 * Every joint of the robot is controlled by an Oscillator
 *
 */

class CPG_NEAT_HYPERNEAT_Brain : Brain{
protected:
    ///\brief map of joints with names (strings) and classes that can control that joint
    map<string, JointPositionMotor*> joints;

    /// \brief This neural network represent the robot
    NeuralNetwork* nn;

    /// \brief kind of encoding for the experiment
    experiment kindOfEncoding;

    /// \brief pointer to learner ( this could be abstract or father class if there are other implementations)
    Learner_NEAT_HyperNEAT* learner;

    /// \brief this method is just a bridge adapter between the output of the learner and the parameters of the Neural network
    vector<double> removeUnusedOutput(vector<double> out);

    vector<double> externalInput;

public:

    /// Constructor
    CPG_NEAT_HYPERNEAT_Brain(physics::ModelPtr model,  sdf::ElementPtr _sdf, experiment kindOfEncoding, vector<double> input);


    /// This method should be called at every increment of time step. Used for the multipleInstanceOpener of robot
    void update(const double actualTime);

    /**
     * This method should be called at the end of the multipleInstanceOpener of a single genome.
     * Set the fitness of the genome that was under multipleInstanceOpener (distance), take the next genome and put that under multipleInstanceOpener,
     * or initialize the next generation
     */
    vector<double> stepOfTest(const ::gazebo::common::UpdateInfo _info, double distance);

    vector<vector<bool>> getAdjacentMatrixOfLastBestGenome(){ return this->learner->adjacentMatrixOfNNOfLastBestGenome; };
    vector<string> getTypesNeuronsOfLastBestGenome(){ return this->learner->NeuronTypesOfLastBestGenome; };
};


#endif //EXPERIMENT_ACTUATORSCONTROLLER_H
