//
// Created by gabriele on 03/07/20.
//
#include "utils.h"

vector<int> castFromStringToCoordinates(string coordinates){

    int n = coordinates.length();
    char char_array[n + 1];
    vector<int> out;
    // copying the contents of the
    // string to char array
    strcpy(char_array, coordinates.c_str());
    for(int i = 0; i < n ; i++){
        if( char_array[i] == '-'){
            out.push_back(-((int) char_array[i+1] - '0'));
            i++;
        }else{
            out.push_back(((int)char_array[i] - '0'));
        }
    }
    return out;
}


double distance(ignition::math::Pose3d a, ignition::math::Pose3d b){

    return std::sqrt(std::pow(a.Pos().X() - b.Pos().X(), 2) +
                     std::pow(a.Pos().Y() - b.Pos().Y(), 2));


}
vector<string> getTypeOfNeurons(NEAT::NeuralNetwork& net){
    vector<string> neuronType;
    for (int i = 0; i < net.m_neurons.size(); i++){
        NEAT::NeuronType type = net.m_neurons[i].m_type;
        if( type == NEAT::INPUT ){
            neuronType.push_back("I");
        }else if( type == NEAT::BIAS){
            neuronType.push_back("B");
        }else if( type == NEAT::HIDDEN){
            neuronType.push_back("H");
        }else if( type == NEAT::OUTPUT){
            neuronType.push_back("O");
        }
    }

    return neuronType;
}
vector<vector<bool>> createAdjacentMatrix(NEAT::NeuralNetwork& net){
    int numberOfNeurons = net.m_neurons.size();
    vector<vector<bool>> A;
    for(int i = 0; i< numberOfNeurons; i++){
        vector<bool> connectionsVector;
        for( int j = 0; j< numberOfNeurons; j++){
            if( j != i ){
                if( ConnectionExists(net, i, j) != -1){
                    connectionsVector.push_back(1);
                }else{
                    connectionsVector.push_back(0);
                }
            }else{
                connectionsVector.push_back(0);
            }
        }
        A.push_back(connectionsVector);
    }


    return A;
}

double ModularityFactor(NEAT::NeuralNetwork& net){
    vector<vector<bool>> A = createAdjacentMatrix(net);
    int numberOfNeurons = net.m_neurons.size();
    //printAdjacentMatrix(A);
    //directional STRUCTURAL SIMILARITY
    double S1 = 0;
    int N1 = 0;
    //cout<<" Input combinations"<<endl;
    vector<vector<int>> combinationInput = comb(net.m_num_inputs, 2);
    for(int i = 0; i< combinationInput.size(); i++){

        if( net.m_neurons[combinationInput[i][0]].m_type != NEAT::BIAS and net.m_neurons[combinationInput[i][1]].m_type != NEAT::BIAS){
            //cout<<combinationInput[i][0] << " "<< combinationInput[i][1]<< " " << net.m_neurons[combinationInput[i][0]].m_type<< " "<< net.m_neurons[combinationInput[i][1]].m_type<<endl;
            S1 += connectionsInCommon(A, combinationInput[i][0], combinationInput[i][1]);
            N1 +=1;
        }

    }

    if( S1 and N1){

        N1 *= numberOfNeurons - net.m_num_inputs;
        S1 = S1/N1;
    }

    int N2 = 0;
    double S2 = 0;
    //cout<<" Hidden combinations"<<endl;
    combinationInput = comb((numberOfNeurons - net.m_num_outputs - net.m_num_inputs) , 2);
    for(int i = 0; i< combinationInput.size(); i++){
        //cout<<net.m_num_inputs + combinationInput[i][0] << " "<< net.m_num_inputs +combinationInput[i][1]<< " " << net.m_neurons[net.m_num_inputs + combinationInput[i][0]].m_type<< " "<< net.m_neurons[net.m_num_inputs + combinationInput[i][1]].m_type<<endl;

        S2 += connectionsInCommon(A, net.m_num_inputs + combinationInput[i][0], net.m_num_inputs + combinationInput[i][1]);
        N2 +=1;
    }

    /*
    //Modularity of hidden
    for(int i = net.m_num_inputs; i< numberOfNeurons - net.m_num_outputs; i++){
        for(int j = i; j < numberOfNeurons; j++){
            if( j > i){
                NEAT::NeuronType typeA = net.m_neurons[i].m_type;
                NEAT::NeuronType typeB = net.m_neurons[j].m_type;
                if(typeA == NEAT::HIDDEN and ( typeB == NEAT::HIDDEN or typeB == NEAT::OUTPUT ) ){
                    S2 += connectionsInCommon(A, i, j);
                    N2 += 1;
                }
            }
        }
    }
    */
    if(N2 and S2){
        N2 *= ( net.m_num_outputs + 1);
        S2 = S2/N2;
    }
    double TotalStructuralUnsimilarity = 1 - S1 - S2;
    //cout<<"Inputs mod, Hiddens Modularity"<<endl;
    //cout<<S1<< " "<<S2<<endl;
    return TotalStructuralUnsimilarity;

}
int sizeOfNeighborhood(vector<vector<bool>> A, unsigned int node){
    int counter = 0;
    for( int k = 0; k< A[node].size(); k++){
        if ( A[node][k]){
            counter++;
        }
    }
    return counter;

}
int connectionsInCommon(vector<vector<bool>> A, unsigned int nodeA, unsigned int nodeB){
    int counter = 0;
    for( int k = 0; k< A[nodeA].size(); k++){
        if ( A[nodeA][k] and A[nodeB][k]){
            counter++;
        }
    }

    return counter;

}
int ConnectionExists(NEAT::NeuralNetwork& net, int a_from, int a_to)
{
    for (unsigned int i = 0; i < net.m_connections.size(); i++)
    {
        if ((net.m_connections[i].m_source_neuron_idx == a_from)
            && (net.m_connections[i].m_target_neuron_idx == a_to))
        {
            return i;
        }
    }

    return -1;
}
void printAdjacentMatrix(vector<vector<bool>> A){
    for(int i = 0; i < A.size(); i++){
        for(int j = 0; j < A[0].size(); j++){
            cout<<A[i][j]<<" ";
        }
        cout<<endl;
    }

}
vector<vector<int>> comb(int N, int K){
    vector<vector<int>> combinations;
    if( N >= K ){

        std::string bitmask(K, 1); // K leading 1's
        bitmask.resize(N, 0); // N-K trailing 0's

        // print integers and permute bitmask
        do {
            vector<int> comb;
            for (int i = 0; i < N; ++i) // [0..N-1] integers
            {
                if (bitmask[i]) comb.push_back(i);
            }
            combinations.push_back(comb);
        } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
    }
    return combinations;
}

bool checkIfTheFileIsNotEmpty(string filename){
    FILE * pFile;
    pFile = fopen (filename.c_str(),"r");
    if (pFile!=NULL){
        fclose (pFile);
        return true;
    }else{
        return false;
    }
}