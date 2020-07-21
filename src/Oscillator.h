//
// Created by gabriele on 26/06/20.
//

#ifndef MODULAREVO_OSCILLATOR_H
#define MODULAREVO_OSCILLATOR_H
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <random>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
using namespace std;


typedef std::vector< double > state_type;

class Oscillator {

    std::random_device rd;


    /// \brief parameters that controll the oscillation
    double out, wout, biasout, gain;
    double lastTickOfTime;
    double previous_time;
    vector<vector<double>> ode_matrix;
    vector<double> actualState;

    vector<double> nextState;
    /// \brief Runge-Kutta 45 stepper
    boost::numeric::odeint::runge_kutta4< state_type > stepper;

    /// \brief name of the oscillator
    string name;

    /// close oscillators
    vector<Oscillator*> adjacent_oscillators;

public:
        Oscillator(string name);

        void setStateEquations();
        void finalize();
        void update(const double actualTime);

        /// \brief add a connections to another oscillator
        void addConnection(Oscillator* o);

        /// \brief print connections with other oscillator
        void printAllParameters();

        /// \brief Getters
        double getOutput(){ return out;};
        double getX(){ return this->actualState[0];};
        double getY(){ return this->actualState[1];};
        string getName(){return this->name;};


        /// \brief set weight of a connection
        void setOutsideConnection(string o, double value);

        /// \brief Setters

        void setWout(double wout){ this->wout = wout; };
        void setBiasOut(double biasout){ this->biasout = biasout; };
        void setGain(double gain){ this->gain = gain; };

        /// return close Oscillators
        vector<Oscillator*> getAdjacentOscillators(){ return adjacent_oscillators; };

        /// set all parameters of the oscillator
        void setParameters(vector<double> param);


};


#endif //MODULAREVO_OSCILLATOR_H
