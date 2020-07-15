//
// Created by gabriele on 03/07/20.
//

#ifndef MODULAREVO_LEARNER_NEAT_HYPERNEAT_H
#define MODULAREVO_LEARNER_NEAT_HYPERNEAT_H

#include <multineat/MultiNEAT.h>
#include <iostream>
#include "utils.h"
using namespace std;

class Learner_NEAT_HyperNEAT {
protected:

    //NEAT PARAMETERS
    NEAT::Parameters* params;
    NEAT::Genome* gen;
    NEAT::Population* pop;
    vector<NEAT::Species>* species;
    vector<NEAT::Genome*> genomes;


    /// \brief genome of the population currently under simulation
    unsigned int genomeIndex = 0;
    /// \brief Counter of the generations
    public: unsigned int generation = 1;
    experiment kindOfEncoding;
    bool useModularity = true;
    double alfa = 0.5;
    //USED IF HYPERNEAT
    vector<vector<double>> mappingOfConnections;
public:

    Learner_NEAT_HyperNEAT(experiment kindOfEncoding, unsigned int n_input, unsigned int n_output);
    void setAlfa(double a){ this->alfa = a; };
    void setMapOfConnections(vector<vector<double>> map){ this->mappingOfConnections = map;};
    double step(double fitness);
    vector<double>  getOutput(vector<double> input);
    vector<double> fromGenomeToNNOutputNEAT(NEAT::Genome gen, vector<double> input);
    vector<double> fromGenomeToNNOutputHyperNEAT(NEAT::Genome gen, vector<double> input);
private:
    void fetchGenomes();
    double getBestFitnessOfPopulation();
};


#endif //MODULAREVO_LEARNER_NEAT_HYPERNEAT_H
