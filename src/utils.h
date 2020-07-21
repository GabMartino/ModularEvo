


#ifndef UTILS_H
#define UTILS_H
#include <multineat/MultiNEAT.h>

#include <vector>
#include <string>
#include <stdio.h>
#include <cstring>
#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
using namespace std;


enum experiment{
    VOID = 0,
    CLOSE = 1,
    INSERTION_ACK = 37,
    DIRECT = 45,
    INDIRECT = 125,
    DIRECT_MOD = 86,
    INDIRECT_MOD = 101,
    DIRECT_ACK = 46,
    INDIRECT_ACK = 126,
    DIRECT_MOD_ACK = 87,
    INDIRECT_MOD_ACK = 102
};

vector<int> castFromStringToCoordinates(string coordinates);

double distance(ignition::math::Pose3d a, ignition::math::Pose3d b);


double ModularityFactor(NEAT::NeuralNetwork& net);

int ConnectionExists(NEAT::NeuralNetwork& net, int a_to, int a_from);
int sizeOfNeighborhood(vector<vector<bool>> A, unsigned int node);
int connectionsInCommon(vector<vector<bool>> A, unsigned int nodeA, unsigned int nodeB);
void printAdjacentMatrix(vector<vector<bool>> A);
vector<vector<bool>> createAdjacentMatrix(NEAT::NeuralNetwork& net);
vector<string> getTypeOfNeurons(NEAT::NeuralNetwork& net);
vector<vector<int>> comb(int N, int K);

bool checkIfTheFileIsNotEmpty(string filename);
#endif /* UTILS_H */