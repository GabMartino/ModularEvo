//
// Created by gabriele on 26/06/20.
//

#include "Oscillator.h"

Oscillator::Oscillator(string name){
    srand(0);

    // Initialize parameteres
    this->name = name;
    x = rand()/ double(RAND_MAX);
    wxy = rand()/ double(RAND_MAX);
    biasx = rand()/ double(RAND_MAX);
    y = rand()/ double(RAND_MAX);
    wyx = rand()/ double(RAND_MAX);
    biasy = rand()/ double(RAND_MAX);
    out = rand()/ double(RAND_MAX);
    wout = rand()/ double(RAND_MAX);
    biasout = rand()/ double(RAND_MAX);
    gain = rand()/ double(RAND_MAX);


}
/**
 *
 *          CHECK THIS FUNCTION IF IS RIGHT
 *
 *
 *
 *
 */
void Oscillator::update(){

    double dxdt = wxy*y + biasx;
    double dydt = wyx*x + biasy;

    double sumOfOtherOscillatorValues = 0;
    for( auto i = 0 ; i < adjacent_oscillators.size(); i++ ){
        sumOfOtherOscillatorValues += adjacent_oscillators_weights[i]*adjacent_oscillators[i]->getX();
    }

    x = x + dxdt + sumOfOtherOscillatorValues;
    y = y + dydt;

    x = std::min(std::max(double(-100), x), 100.0);
    y = std::min(std::max(double(-100), y), 100.0);

    out = (wout*x - biasout)* gain;
}

void Oscillator::addConnection(Oscillator* o){

    //add to the vector of connections
    adjacent_oscillators.push_back(o);

    // the connection is initilized randomly
    adjacent_oscillators_weights.push_back(rand()/ double(RAND_MAX));

}


void Oscillator::printConnections(){
    cout<<"Connections of "<< this->name<<endl;
    for( auto i = 0; i < adjacent_oscillators.size(); i++){
        cout<<"    "<<adjacent_oscillators[i]->getName()<<endl;

    }

}


void Oscillator::setOutsideConnection(string o, double value){

    vector<Oscillator*>::iterator it = adjacent_oscillators.begin();
    int j = 0;
    for( it ; it != adjacent_oscillators.end(); it++){
        if( !((*it)->name).compare(o)){
            adjacent_oscillators_weights[j] = value;
            break;
        }
        j++;
    }
    
}


//Order of parameters
// wxy, biasx, wyx, biasy, wout, biasout, gain
void Oscillator::setParameters(vector<double> param){
    // dxdt = wxy*y + biasx;
    this->setWxy(param[0]);
    this->setBiasX(param[1]);
    //dydt = wyx*x + biasy;
    this->setWyx(param[2]);
    this->setBiasY(param[3]);
    //out = (wout*x - biasout)* gain;
    this->setWout(param[4]);
    this->setBiasOut(param[5]);
    this->setGain(param[6]);
}