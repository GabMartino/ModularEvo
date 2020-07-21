//
// Created by gabriele on 26/06/20.
//

#ifndef MODULAREVO_NEURALNETWORK_H
#define MODULAREVO_NEURALNETWORK_H
#include "Oscillator.h"
#include "JointPositionMotor.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "utils.h"
#include <iostream>
#include <string>
#include <map>
using namespace std;

class NeuralNetwork {
private:

    /// \brief vector of the oscillators controlled by the NN with their names
    map<string, Oscillator*> oscillators;

    /// \brief used just to check if the parameters inserted are equals to the one necessary
    unsigned int numberOfParameters = 0;


    vector<vector<double>> mappingOfConnections;

public:
    /// Constructor take the names of the joint and the sdf model to build the neural network
    NeuralNetwork(vector<string> jointNames, sdf::ElementPtr _sdf);
    /// Destructor
    ~NeuralNetwork();
    /// Getter
    map<string, Oscillator*> getOscillators(){ return oscillators; };

    /// return the output of the oscillator at that time ( used after the update)
    double getOutputOfOscillator(string key);

    /// \brief This method will print the connections of each neurons
    void printNeuralNetwork();

    unsigned int getNumberOfParameters(){ return this->numberOfParameters; };

    /// \brief This method will update the time position of each neuron ( Oscillator)
    void update(const double actualTime);

    /// \brief set the parameters of the network from each equations of the first Oscillators to the last
    void setParameters(vector<double> param);

    /// \brief Create the CPPN inputs
    void fetchMapOfConnections();
    /// \brief print the CPPN inputs
    void printMapOfConnections();
    /// \brief return the CPPN inputs
    vector<vector<double>> getMapOfConnections(){ return mappingOfConnections;};

};


#endif //MODULAREVO_NEURALNETWORK_H
