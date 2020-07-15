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

double ModularityFactor(NEAT::NeuralNetwork& net){

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

    //directional STRUCTURAL SIMILARITY
    double TotalStructuralUnsimilarity = 0;

    for(int i = 0; i< numberOfNeurons; i++) {
        for (int j = 0; j < numberOfNeurons; j++) {
            if( i != j){
                double normalization = (net.m_num_inputs + 1)*(net.m_num_outputs + (net.m_neurons.size() - net.m_num_inputs - net.m_num_outputs ));
                if( normalization ){
                    TotalStructuralUnsimilarity += connectionsInCommon(A, i, j)/normalization;
                }
            }

        }
    }
    TotalStructuralUnsimilarity = 1 - TotalStructuralUnsimilarity;
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
int ConnectionExists(NEAT::NeuralNetwork& net, int a_to, int a_from)
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