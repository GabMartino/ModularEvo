//
// Created by gabriele on 26/06/20.
//

#include "Oscillator.h"

Oscillator::Oscillator(string name){
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1, 1);
    // Initialize parameteres
    this->name = name;

    this->actualState.push_back(dis(gen));//X
    this->actualState.push_back(dis(gen));//Y

    out = dis(gen);
    wout = dis(gen);
    biasout = dis(gen);
    gain = dis(gen);

    this->previous_time = 0;

}


void Oscillator::update(const double actualTime){
    // Stepper. The result is saved in x. Begin time t, time step dt
    double dt = (actualTime - this->previous_time);
    this->previous_time = actualTime;


    state_type x;
    x.push_back(this->getX());
    x.push_back(this->getY());
    for( auto i = 0; i < adjacent_oscillators.size(); i++){
        x.push_back(adjacent_oscillators[i]->getX());
    }
    x.push_back(0); // for the bias
    //vector<double> dxdt;
    // Perform one step
    stepper.do_step(
            [this](const state_type &x, state_type &dxdt, double t)
            {
                for(unsigned int  i = 0; i < this->ode_matrix.size(); i++)
                {
                    dxdt[i] = 0;
                    for(unsigned int  j = 0; j < this->ode_matrix[i].size(); j++)
                    {
                        dxdt[i] += x[j]*this->ode_matrix[i][j];
                    }
                }
            },
            x,
            actualTime,
            dt);

    this->nextState = x;

}
void Oscillator::setStateEquations(){

    this->actualState = this->nextState;
    //cout<<this->actualState[0]<< " " << this->actualState[1]<<endl;

    out = (2.0)/(1.0 + std::pow(2.718, -2.0 * this->getX())) - 1;
    /*
    out = (wout*this->getX() - biasout)* gain;
    out = std::min(std::max(double(-1), out), 1.0);

    */
}
void Oscillator::finalize(){
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1, 1);
    /**
     * x' = 0 +  wxy*y + SUM I (wxxi*xi) + bias
     * y' = wyx*x + 0 +    zero      + bias
     * the weight are random
     */

    vector<double> row1;
    row1.push_back(0);//0*x
    row1.push_back(dis(gen));//wxy
    for( auto i = 0 ; i < adjacent_oscillators.size(); i++ ){
        row1.push_back(dis(gen));//At first are random
    }
    row1.push_back(dis(gen));//biasx

    vector<double> row2;
    row2.push_back(dis(gen));//wyx
    row2.push_back(0);// 0*y
    for( auto i = 0 ; i < adjacent_oscillators.size(); i++ ){
        row2.push_back(0);
    }
    row2.push_back(dis(gen));// biasy

    this->ode_matrix.push_back(row1);
    this->ode_matrix.push_back(row2);
    //print
    /*
    for( int i = 0; i< this->ode_matrix.size() ; i++){
        for (int j = 0 ; j< this->ode_matrix[i].size() ; j++){
            cout<<this->ode_matrix[i][j]<< " ";
        }
        cout<<endl;
    }
     */

}
void Oscillator::addConnection(Oscillator* o){

    //add to the vector of connections
    adjacent_oscillators.push_back(o);


}


void Oscillator::printAllParameters(){
    cout<<endl;
    cout<<" Wxy: "<<this->ode_matrix[0][1]<< " biasx: "<<this->ode_matrix[0][this->ode_matrix[0].size() - 1]<<endl;
    cout<<" Wyx: "<<this->ode_matrix[1][0]<< " biasy: "<<this->ode_matrix[1][this->ode_matrix[1].size() - 1]<<endl;
    cout<<" Gain: "<<gain<< " Wout:"<<wout<< " biasOut:"<<biasout<<endl;
    cout<<"Connections of "<< this->name<<endl;
    for( auto i = 0; i < adjacent_oscillators.size(); i++){
        cout<<"    "<<adjacent_oscillators[i]->getName()<<" weight: "<<this->ode_matrix[0][2 + i] <<endl;

    }

}


void Oscillator::setOutsideConnection(string o, double value){

    vector<Oscillator*>::iterator it = adjacent_oscillators.begin();
    int j = 0;
    int count = 0;
    for( it ; it != adjacent_oscillators.end(); it++){
        if( !((*it)->name).compare(o)){
            //cout<<"Set weight of "<<this->name<< " to "<<o<<" "<<value<<endl;
            this->ode_matrix[0][2 + count] = value; // add the parameters of the weights to the matrix
            break;
        }
        count++;
        j++;
    }
    
}


//Order of parameters
// wxy, biasx, wyx, biasy, wout, biasout, gain
void Oscillator::setParameters(vector<double> param){

    // dxdt = wxy*y + biasx;
    this->ode_matrix[0][1] = param[0];

    this->ode_matrix[0][this->ode_matrix[0].size() - 1] = param[1];

    this->ode_matrix[1][0] = param[2];

    this->ode_matrix[1][this->ode_matrix[1].size() - 1] = param[3];

    //out = (wout*x - biasout)* gain;
    this->setWout(param[4]);
    this->setBiasOut(param[5]);
    this->setGain(param[6]);
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1, 1);
    /*
     * reset state
     */
    this->actualState.push_back(dis(gen));//X
    this->actualState.push_back(dis(gen));//Y
}