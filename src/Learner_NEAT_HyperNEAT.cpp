//
// Created by gabriele on 03/07/20.
//

#include "Learner_NEAT_HyperNEAT.h"


Learner_NEAT_HyperNEAT::Learner_NEAT_HyperNEAT(experiment kindOfEncoding, unsigned int n_input, unsigned int n_output){
    this->kindOfEncoding = kindOfEncoding;

    /// \brief set some parameters for NEAT
    this->params = new NEAT::Parameters();
    this->params->PopulationSize = 10;
    this->params->CrossoverRate = 0.5;
    this->params->SurvivalRate = 0.2;
    this->params->MutateAddNeuronProb = 0.25;
    this->params->MutateAddLinkProb = 0.4;
    this->params->MutateRemLinkProb = 0.0;
    this->params->AllowLoops = false;
    this->params->MutateAddLinkFromBiasProb = 0.3;
    this->params->WeightMutationMaxPower = 0.001;
    this->params->WeightReplacementMaxPower = 0.3;
    this->params->MutateWeightsSevereProb = 0.0;
    this->params->WeightMutationRate = 0.25;
    this->params->WeightReplacementRate = 0.9;
    this->params->OverallMutationRate = 1.0;

    this->params->ActivationFunction_UnsignedSigmoid_Prob = 0.3;

    this->params->ActivationFunction_Tanh_Prob = 0.3;
    this->params->ActivationFunction_SignedGauss_Prob = 0.3;
    this->params->OldAgeTreshold = 35;
    this->params->RouletteWheelSelection = false;
    this->params->WeightDiffCoeff = 0.1;
    this->params->RecurrentProb = 0.0;
    this->params->RecurrentLoopProb = 0.0;

    /// Set the shape of the genomes
    this->gen = new NEAT::Genome(0, n_input, 0, n_output, false, NEAT::TANH,
                                 NEAT::TANH, 0, *this->params, 0);

    /// Create a population from the first genome
    this->pop = new NEAT::Population(*(this->gen), *(this->params), true, 1, 0);

    fetchGenomes();

}
double Learner_NEAT_HyperNEAT::step(double fitness){

    double bestFitness = 0;

    if(this->kindOfEncoding == DIRECT_MOD or this->kindOfEncoding == INDIRECT_MOD){
        /// fetch the NN from the genome of which we want to test the modularity
        NEAT::Genome gen = (*(this->genomes[genomeIndex - 1]));
        NEAT::NeuralNetwork* net = new NEAT::NeuralNetwork();
        gen.BuildPhenotype(*net);

        fitness = alfa*fitness + (1 - alfa)*ModularityFactor(*net);

    }

    cout<<"Genome number "<<genomeIndex - 1<< " has fitness "<< fitness<<endl;
    /// \brief set the fitness of the genome under test
    (*(this->genomes[genomeIndex - 1])).SetFitness(fitness);


    //If the whole population has been tested go to the next generation
    if(genomeIndex >= this->params->PopulationSize){
        genomeIndex = 0;
        bestFitness = getBestFitnessOfPopulation();
        (*(this->pop)).Epoch();
        generation++;
        cout<<"Generation number "<<generation<<endl;
        fetchGenomes();
    }
    return bestFitness;

}
void Learner_NEAT_HyperNEAT::fetchGenomes(){
    this->genomes.clear();
    /// GET THE VECTOR OF THE GENOMES
    this->species = &((*this->pop).m_Species);
    //fetch all the genomes
    for( int i = 0 ; i<(*(this->species)).size(); i++){
        for (int j = 0; j< (*(this->species))[i].m_Individuals.size() ; j++){
            this->genomes.push_back(&((*(this->species))[i].m_Individuals[j]));
        }
    }

}
double Learner_NEAT_HyperNEAT::getBestFitnessOfPopulation(){
    double max = 0;
    for(int i = 0; i < this->genomes.size(); i++){
        if( max < this->genomes[i]->GetFitness()){
            max = this->genomes[i]->GetFitness();
        }
    }
    return max;
}


vector<double> Learner_NEAT_HyperNEAT:: getOutput(vector<double> input){
    NEAT::Genome gen = *(this->genomes[genomeIndex++]);
    vector<double> out;
    /// Extract the output of the genome
    if(this->kindOfEncoding == DIRECT or this->kindOfEncoding == DIRECT_MOD  ){
        out = fromGenomeToNNOutputNEAT(gen, input);
    }else if (this->kindOfEncoding == INDIRECT or this->kindOfEncoding == INDIRECT_MOD){
        out = fromGenomeToNNOutputHyperNEAT(gen, input);
    }
    return out;

}
vector<double> Learner_NEAT_HyperNEAT::fromGenomeToNNOutputNEAT(NEAT::Genome gen, vector<double> input){

    /// Create a new NN from the Genome
    NEAT::NeuralNetwork* net = new NEAT::NeuralNetwork();
    gen.BuildPhenotype(*net);

    /// Test it
    (*net).Flush();
    (*net).Input(input);
    (*net).Activate();

    /// take and report the output
    vector<double> o = (*net).Output();
    o.pop_back(); // remove the bias
    delete net;

    return o;

}


vector<double> Learner_NEAT_HyperNEAT::fromGenomeToNNOutputHyperNEAT(NEAT::Genome gen, vector<double> input){
    vector<double> o;

    /// Create a new NN from the Genome
    NEAT::NeuralNetwork* net = new NEAT::NeuralNetwork();
    gen.BuildPhenotype(*net);
    vector<vector<double>>::iterator it = mappingOfConnections.begin();
    int j = 0;
    while( it != mappingOfConnections.end()) {
        vector<double> CPPNInput = *it;
        CPPNInput.push_back(input[0]);
        /// Test it
        (*net).Flush();
        (*net).Input(CPPNInput);
        (*net).Activate();
        /// take and report the output
        vector<double> out = (*net).Output();

        for( int j = 0; j < out.size() - 1; j++){// without bias
            o.push_back(out[j]);
        }

        it++;
    }

    delete net;

    return o;

}