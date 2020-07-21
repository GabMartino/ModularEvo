//
// Created by gabriele on 03/07/20.
//

#include "Learner_NEAT_HyperNEAT.h"


Learner_NEAT_HyperNEAT::Learner_NEAT_HyperNEAT(experiment kindOfEncoding, unsigned int n_input, unsigned int n_output){

    this->kindOfEncoding = kindOfEncoding;
    initParams();
    /// Set the shape of the genomes
    this->gen = new NEAT::Genome(0, n_input, 0, n_output, false, NEAT::TANH,
                                 NEAT::TANH, 0, *this->params, 0);
    /// Create a population from the first genome
    this->pop = new NEAT::Population(*(this->gen), *(this->params), true, 1, 0);
    fetchGenomes();
}

void Learner_NEAT_HyperNEAT::initParams(){
    /// \brief set some parameters for NEAT
    this->params = new NEAT::Parameters();
    this->params->PopulationSize = 10;
    this->params->CrossoverRate = 0.25;
    this->params->SurvivalRate = 0.2;
    this->params->MutateAddNeuronProb = 0.10;
    this->params->MutateAddLinkProb = 0.4;
    //this->params->MutateRemLinkProb = 0.0001;
    this->params->AllowLoops = false;
    this->params->MutateAddLinkFromBiasProb = 0.05;
    this->params->WeightMutationMaxPower = 0.005;
    this->params->WeightReplacementMaxPower = 0.1;
    this->params->MutateWeightsSevereProb = 0.0;
    this->params->WeightMutationRate = 0.25;
    this->params->WeightReplacementRate = 0.1;
    this->params->OverallMutationRate = 1.0;

    this->params->ActivationFunction_SignedSigmoid_Prob = 0.3;
    this->params->ActivationFunction_Tanh_Prob = 0.3;
    this->params->ActivationFunction_SignedGauss_Prob = 0.3;
    this->params->ActivationFunction_SignedSine_Prob = 0.3;

    this->params->OldAgeTreshold = 35;
    this->params->RouletteWheelSelection = false;
    this->params->WeightDiffCoeff = 0.1;
    this->params->RecurrentProb = 0.0;
    this->params->RecurrentLoopProb = 0.0;
    this->params->EliteFraction = 0.02;

}
Learner_NEAT_HyperNEAT::Learner_NEAT_HyperNEAT(experiment kindOfEncoding, unsigned int n_input, unsigned int n_output, string starterGenomeFileName){
    this->kindOfEncoding = kindOfEncoding;
    initParams();
    if(checkIfTheFileIsNotEmpty(starterGenomeFileName)){

    }else{
        cout<<"ERROR: THE FILE DOES NOT EXITS."<<endl;
        return;
    }
    this->gen = new NEAT::Genome(starterGenomeFileName.c_str());
    this->pop = new NEAT::Population(*(this->gen), *(this->params), true, 1, 0);
    fetchGenomes();
}
vector<double> Learner_NEAT_HyperNEAT::step(double fitness){
    vector<double> bestFitness;
    bestFitness.push_back(0);//TotalFitness
    bestFitness.push_back(0);//ModularityFactor
    if(this->startLearning){
        if(this->kindOfEncoding == DIRECT_MOD or this->kindOfEncoding == INDIRECT_MOD){
            /// fetch the NN from the genome of which we want to multipleInstanceOpener the modularity
            NEAT::Genome gen = (*(this->genomes[genomeIndex - 1]));
            NEAT::NeuralNetwork* net = new NEAT::NeuralNetwork();
            gen.BuildPhenotype(*net);
            fitness = alfa*fitness + (1 - alfa)*ModularityFactor(*net);
        }

        cout<<"[LEARNER] Genome number "<<genomeIndex - 1<< " has fitness "<< fitness<<endl;
        /// \brief set the fitness of the genome under multipleInstanceOpener
        (*(this->genomes[genomeIndex - 1])).SetFitness(fitness);


        //If the whole population has been tested go to the next generation
        if(genomeIndex >= this->params->PopulationSize){
            if( generation >= this->maxNumberOfGenerations){
                /**
                 * Fetch the best Genome of the last generation and save it
                 *
                 */
                int bestGenomeIndex = 0;
                double max = 0;
                for(int i = 0; i < this->genomes.size(); i++){
                    if( max < this->genomes[i]->GetFitness()){
                        max = this->genomes[i]->GetFitness();
                        bestGenomeIndex = i;
                    }
                }
                NEAT::Genome gen = (*(this->genomes[bestGenomeIndex]));

                cout<<"[LEARNER] Writing the Best Genome on file."<<endl;

                gen.Save(this->bestGenomeFileName.c_str());
                cout<<"[LEARNER] Best Genome saved."<<endl;
                this->startLearning = false;// stop the learner
                bestFitness = getBestFitnessOfPopulation();
            }else{
                genomeIndex = 0;
                bestFitness = getBestFitnessOfPopulation();
                (*(this->pop)).Epoch();
                generation++;
                cout<<"[LEARNER] Generation number "<<generation<<endl;
                fetchGenomes();
            }

        }
    }else{
        cout<<"THE LEARNER IS NOT ACTIVE. PLEASE START THE LEARNER"<<endl;
    }

    return bestFitness;

}

Learner_NEAT_HyperNEAT::~Learner_NEAT_HyperNEAT(){
    delete params;
    delete gen;
    delete pop;
    delete species;
    for ( int i = 0; i< genomes.size(); i++){
        delete genomes[i];
    }
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
vector<double> Learner_NEAT_HyperNEAT::getBestFitnessOfPopulation(){
    vector<double> max;
    max.push_back(0);
    max.push_back(0);

    int bestGenomeIndex = 0;
    for(int i = 0; i < this->genomes.size(); i++){
        if( max[0] < this->genomes[i]->GetFitness()){
            max[0] = this->genomes[i]->GetFitness();
            bestGenomeIndex = i;
        }
    }

    NEAT::Genome gen = (*(this->genomes[bestGenomeIndex]));
    NEAT::NeuralNetwork* net = new NEAT::NeuralNetwork();
    gen.BuildPhenotype(*net);
    max[1] = ModularityFactor(*net);

    this->adjacentMatrixOfNNOfLastBestGenome = createAdjacentMatrix(*net);
    this->NeuronTypesOfLastBestGenome = getTypeOfNeurons(*net);
    delete net;

    return max;
}


vector<double> Learner_NEAT_HyperNEAT:: getOutput(vector<double> input){
    vector<double> out;
    if(this->startLearning){
        NEAT::Genome gen = *(this->genomes[genomeIndex++]);

        /// Extract the output of the genome
        if(this->kindOfEncoding == DIRECT or this->kindOfEncoding == DIRECT_MOD  ){
            out = fromGenomeToNNOutputNEAT(gen, input);
        }else if (this->kindOfEncoding == INDIRECT or this->kindOfEncoding == INDIRECT_MOD) {
            out = fromGenomeToNNOutputHyperNEAT(gen, input);
        }
    }else{
        cout<<"THE LEARNER IS NOT ACTIVE. PLEASE START THE LEARNER"<<endl;
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
    unsigned int i = (*net).CalculateNetworkDepth();
    while( i != 0){
        (*net).Activate();
        i--;
    }

    /// take and report the output
    vector<double> o = (*net).Output();
    delete net;

    return o;

}


vector<double> Learner_NEAT_HyperNEAT::fromGenomeToNNOutputHyperNEAT(NEAT::Genome gen, vector<double> input){
    vector<double> o;

    /// Create a new NN from the Genome
    NEAT::NeuralNetwork* net = new NEAT::NeuralNetwork();
    gen.BuildPhenotype(*net);
    vector<vector<double>>::iterator it = mappingOfConnections.begin();

    while( it != mappingOfConnections.end()) {
        vector<double> CPPNInput = *it;
        CPPNInput.push_back(input[0]);// add the external input
        /// Test it
        (*net).Flush();
        (*net).Input(CPPNInput);
        unsigned int i = (*net).CalculateNetworkDepth();
        while( i != 0){
            (*net).Activate();
            i--;
        }
        //(*net).Activate();
        /// take and report the output
        vector<double> out = (*net).Output();

        for( int j = 0; j < out.size(); j++){
            o.push_back(out[j]);
        }

        it++;
    }

    delete net;

    return o;

}