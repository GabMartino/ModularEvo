//
// Created by gabriele on 26/06/20.
//

#include "NeuralNetwork.h"


NeuralNetwork::NeuralNetwork(vector<string> jointNames, sdf::ElementPtr _sdf){

    cout<<"[NN] Initialization of the Neural Network. Getting the connections from the sdf."<<endl;
    /// Create the vector of Oscillators with their names
    for( auto i = 0; i < jointNames.size() ; i++){
        oscillators[jointNames[i]] = new Oscillator(jointNames[i]);
        numberOfParameters += 7;
    }

    /// Extract the morphology of the NN of the robot from the SDF Model
    sdf::ElementPtr connections = _sdf->GetElement("connections");
    sdf::ElementPtr connect = connections ? connections->GetElement("connect") : nullptr;
    while(connect){
        sdf::ElementPtr to = connect->GetElement("to");
        while(to){
            /// Create connection between two oscillator
            oscillators[connect->GetAttribute("name")->GetAsString()]->addConnection(oscillators[to->GetAttribute("name")->GetAsString()]);
            numberOfParameters += 1;

            /// Add a reversed connection if is mutual
            if(connect->GetAttribute("mutual")->GetAsString().compare("True")){
                oscillators[to->GetAttribute("name")->GetAsString()]->addConnection(oscillators[connect->GetAttribute("name")->GetAsString()]);
                //numberOfParameters += 1; // the mutual weight is already included
            }
            to = to->GetNextElement("to");
        }
        connect = connect->GetNextElement("connect");
    }
    /**
     * THis is important to create the matrix of the differential equations after set the connections
     */
    for( auto i = 0; i < jointNames.size() ; i++){
        oscillators[jointNames[i]]->finalize();
    }


    cout<<"[NN] Create the CPPN inputs in case the HYPERNEAT is used."<<endl;
    /// Create the CPPN inputs
    fetchMapOfConnections();

}
/**
 * This method will print the names of the neuron (Oscillators) and their connections
 *
 */
void NeuralNetwork::printNeuralNetwork(){
    map<string, Oscillator*>::iterator it = oscillators.begin();
    while( it != oscillators.end()){
        it->second->printAllParameters();
        it++;
    }
}

/**
 * Get the output of a single Oscillator given its name
 *
 * @param key
 * @return
 */

double NeuralNetwork::getOutputOfOscillator(string key){
    return oscillators[key]->getOutput();
}

/**
 * This method should be called at every increment of time step.
 * Used for the multipleInstanceOpener of robot
 *
 */
void NeuralNetwork::update(const double actualTime){

    /**
     * This two while loop are important to be set in consecutio,
     * because the first solved the approximated differential equations,
     * the second update the value of the state variables
     *
     */
    map<string, Oscillator*>::iterator it = oscillators.begin();

    while( it != oscillators.end()){
        it->second->update(actualTime);
        it++;
    }
    it = oscillators.begin();
    while( it != oscillators.end()){
        it->second->setStateEquations();
        it++;
    }
    //exit(0);
}

/**
 * In direct encoding the first parameters are the parameters of each oscillators.
 * The last parameters are the ones of the connections between the oscillators
 *
 *
 */
void NeuralNetwork::setParameters(vector<double> param){

    if( param.size() != numberOfParameters ){
        cout<<param.size() << "   "<< numberOfParameters<<endl;
        cerr<< "[NN] Wrong number of parameters"<<endl;
        return;
    }
    //For each oscillator present in the robot
    map<string, Oscillator*>::iterator it = oscillators.begin();
    vector<double>::iterator j = param.begin();
    while( it != oscillators.end() ){
        //take 7 parameters that are the one for each oscillator
        vector<double>::iterator first = j;
        vector<double>::iterator last = j + 7;
        vector<double> oscillatorParams(first, last);

        (*(it->second)).setParameters(oscillatorParams);// set the seven parameteres
        it++;
        if( it != oscillators.end())
            j = j + 7;
    }
    it = oscillators.begin();
    map<string, string> alreadySetMutualConnections;
    while( it != oscillators.end() ){

        vector<Oscillator*> temp = it->second->getAdjacentOscillators();
        for( auto o = 0 ; o < temp.size() ; o++) {
            if (alreadySetMutualConnections[it->first] != temp[o]->getName() )
            {
                it->second->setOutsideConnection(temp[o]->getName(), *j);
                temp[o]->setOutsideConnection(it->first, -(*j)); // add the mutual weight that is the opposite
                alreadySetMutualConnections[it->first] = temp[o]->getName();
                alreadySetMutualConnections[temp[o]->getName()] = it->first;
                j++;
            }
        }
        it++;

    }

}

/**
 * Build the map of the inputs used for the eventual CPPN.
 * The inputs represent the equations of the oscillators or the weights between the oscillators
 *
 */
void NeuralNetwork::fetchMapOfConnections(){

    map<string, Oscillator*>::iterator it = oscillators.begin();
    int j = 0;
    while( it != oscillators.end()){
        double x1 = castFromStringToCoordinates(it->first)[0];
        double y1 = castFromStringToCoordinates(it->first)[1];
        double z1;
        for (int i = 0; i< 3 ; i++){
            if( j % 3 == 0){
                z1 = 1;
            }else if( j % 3 == 1){
                z1 = -1;
            }else if( j % 3 == 2 ){
                z1 = 0;
            }
            double x2 = 0;
            double y2 = 0;
            vector<double> row = { x1, y1, z1, x2, y2};
            mappingOfConnections.push_back(row);
            j++;
        }
        it++;
    }
    it = oscillators.begin();
    while( it != oscillators.end()){
        // parsing of a node
        double x1 = castFromStringToCoordinates(it->first)[0];
        double y1 = castFromStringToCoordinates(it->first)[1];
        double z1 = 1;
        for( int i = 0 ; i< it->second->getAdjacentOscillators().size() ; i++) {

            double x2 = castFromStringToCoordinates(it->second->getAdjacentOscillators()[i]->getName())[0];
            double y2 = castFromStringToCoordinates(it->second->getAdjacentOscillators()[i]->getName())[1];
            vector<double> row = { x1, y1, z1, x2, y2};
            /**
             * Check if already exists a connection, if exstis not create a new one in the map
             * because the resulting weight is simply the opposite
             *
             */
            bool insert = true;
            for( int j = 0; j < mappingOfConnections.size() ; j++){
                if(mappingOfConnections[j][2] == 1 ){
                    if( mappingOfConnections[j][0] == x2 and mappingOfConnections[j][1] == y2 and  mappingOfConnections[j][3] == x1 and mappingOfConnections[j][4] == y1){
                        insert = false;
                    }
                }
            }
            if(insert){
                mappingOfConnections.push_back(row);
            }

        }
        it++;
    }

}
/**
 * This method simply print all the CPPN input of the network
 *
 */
void NeuralNetwork::printMapOfConnections(){

    for( int i = 0; i< mappingOfConnections.size(); i++){
        for (int j = 0; j < mappingOfConnections[i].size(); j++){
            cout<< mappingOfConnections[i][j] << " ";

        }
        cout<<endl;
    }

}