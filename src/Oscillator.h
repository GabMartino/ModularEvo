//
// Created by gabriele on 26/06/20.
//

#ifndef MODULAREVO_OSCILLATOR_H
#define MODULAREVO_OSCILLATOR_H
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
using namespace std;

class Oscillator {
    /// \brief parameters that controll the oscillation
    double x, wxy, biasx;
    double y, wyx, biasy;
    double out, wout, biasout, gain;
    double lastTickOfTime;

    /// \brief name of the oscillator
    string name;
    /// \brief weights of the link to the near oscillators
    vector<double> adjacent_oscillators_weights;
    /// close oscillators
    vector<Oscillator*> adjacent_oscillators;


public:
        Oscillator(string name);


        void update();

        /// \brief add a connections to another oscillator
        void addConnection(Oscillator* o);

        /// \brief print connections with other oscillator
        void printConnections();

        /// \brief Getters
        double getOutput(){ return out;};
        double getX(){ return x;};
        string getName(){return this->name;};


        /// \brief set weight of a connection
        void setOutsideConnection(string o, double value);

        /// \brief Setters
        void setX(double x){ this->x = x; };
        void setY(double y){ this->y = y; };
        void setWxy(double wxy){ this->wxy = wxy; };
        void setBiasX(double biasx){ this->biasx = biasx; };
        void setWyx(double wyx){ this->wyx = wyx; };
        void setBiasY(double biasy){ this->biasy = biasy; };
        void setWout(double wout){ this->wout = wout; };
        void setBiasOut(double biasout){ this->biasout = biasout; };
        void setGain(double gain){ this->gain = gain; };

        /// return close Oscillators
        vector<Oscillator*> getAdjacentOscillators(){ return adjacent_oscillators; };

        /// set all parameters of the oscillator
        void setParameters(vector<double> param);


};


#endif //MODULAREVO_OSCILLATOR_H
