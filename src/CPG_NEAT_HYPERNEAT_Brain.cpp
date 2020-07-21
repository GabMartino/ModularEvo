//
// Created by gabriele on 24/06/20.
//

#include "CPG_NEAT_HYPERNEAT_Brain.h"

CPG_NEAT_HYPERNEAT_Brain::CPG_NEAT_HYPERNEAT_Brain( physics::ModelPtr model, sdf::ElementPtr _sdf, experiment kindOfEncoding, vector<double> input){

    cout<<"[BRAIN] Preparing the brain."<<endl;
    /**
     * CREATE A MAPPING OF THE JOINTS OF THE ROBOT
     * ( SAVE THE NAMES OF THE JOINTS )
     */

    /// get the joints from the model
    physics::Joint_V joint_vector = model->GetJoints();
    vector<string> jointNames;
    /// get the name of each joint and create a proper mapping for each one represented by the JointPositionMotor class
    sdf::ElementPtr encoding = _sdf->GetElement("jointEncoding");
    while(encoding){
        for( auto i = 0; i < joint_vector.size(); i++){
            if(!joint_vector[i]->GetName().compare(encoding->GetAttribute("jointName")->GetAsString())) {
                joints[encoding->GetAttribute("jointCode")->GetAsString()] = new JointPositionMotor(joint_vector[i]);
                jointNames.push_back(encoding->GetAttribute("jointCode")->GetAsString());

                break;
            }
        }
        encoding = encoding->GetNextElement("jointEncoding");
    }
    /**
     * CREATE THE NEURAL NETWORK THAT REPRESENTS THE TOPOLOGY OF THE ROBOT
     *
     * ( USE THE NAMES OF THE JOINTS FETCHED BEFORE FOR THE MAPPING,
     *  USE THE SDF MODEL TO BUILD THE TOPOLOGY)
     *
     */

    cout<<"[BRAIN] Preparing the NN."<<endl;
    /// Create the NeuralNetwork that will represent the robot
    this->nn = new NeuralNetwork(jointNames, _sdf);
    //this->nn->printMapOfConnections();

    /// set the learning algorithm and its encoding
    this->kindOfEncoding = kindOfEncoding;

    /**
     * THIS INPUT SHOULD BE TAKEN FROM THE WORLD AS INDICATOR OF THE ENVIRONMENTAL CHANGE
     *
     */

    this->externalInput = input;

    cout<<"[BRAIN]";
    if( this->kindOfEncoding == DIRECT or this->kindOfEncoding == DIRECT_MOD){    //NEAT

        cout<<"     Set NEAT Parameters for DIRECT Encoding"<<endl;

        /// Initialize the learning algorithm
        this->learner = new Learner_NEAT_HyperNEAT(this->kindOfEncoding, this->externalInput.size()+1, this->nn->getNumberOfParameters());

        this->nn->setParameters(this->learner->getOutput(this->externalInput));

    }else if( this->kindOfEncoding == INDIRECT or this->kindOfEncoding == INDIRECT_MOD){ //HYPERNEAT

        cout<<"     Set HyperNEAT Parameters for INDIRECT Encoding."<<endl;

        unsigned int n_input = (this->nn->getMapOfConnections())[0].size() + this->externalInput.size() + 1;
        unsigned int n_output = 3; // values of the equations
        this->learner = new Learner_NEAT_HyperNEAT(this->kindOfEncoding, n_input, n_output);

        this->learner->setMapOfConnections(this->nn->getMapOfConnections());

        this->nn->setParameters(removeUnusedOutput(this->learner->getOutput(this->externalInput)));


    }else{
        cout<<"     ERROR choosing kind of encoding."<<endl;
        return;
    }

    cout<<"[BRAIN] Brain Prepared."<<endl;
}


/**
 * This method update the NN representing the robot, and use that new parameters to update
 * the underneath controller in time.
 *
 * @param _info
 */
void CPG_NEAT_HYPERNEAT_Brain::update(const double actualTime) {
    /// update the NN
    this->nn->update(actualTime);

    /// update each Oscillators with the new values from the NN
    map<string, JointPositionMotor*>::iterator it = this->joints.begin();
    while( it != this->joints.end()){

        it->second->update(this->nn->getOutputOfOscillator(it->first));
        it++;
    }
}

/**
 * This method takes the fitness as input, set that to the genome that was under multipleInstanceOpener and prepare the next
 *
 * @param _info
 * @param distance
 */
vector<double> CPG_NEAT_HYPERNEAT_Brain::stepOfTest(const ::gazebo::common::UpdateInfo _info, double distance){

    vector<double> checkBestFitnessAvailable = this->learner->step(distance);

    if(this->kindOfEncoding == DIRECT or this->kindOfEncoding == DIRECT_MOD){

        this->nn->setParameters(this->learner->getOutput(this->externalInput));

    }else if(this->kindOfEncoding == INDIRECT or this->kindOfEncoding == INDIRECT_MOD){
        vector<double> params = removeUnusedOutput(this->learner->getOutput(this->externalInput));

        this->nn->setParameters(params);

    }
    return checkBestFitnessAvailable;
}

vector<double> CPG_NEAT_HYPERNEAT_Brain::removeUnusedOutput(vector<double> out){
    vector<double> o;
    bool flag = false;
    int j = 0;
    for(int i = 0;i < out.size(); i){
        if( i < this->nn->getOscillators().size()* 3*3){// number of oscillator* equations that describe each oscillators
            o.push_back(out[i]);// w
            o.push_back(out[i+1]);// b
            if( j % 3 == 2){
                o.push_back(out[i+2]);// add also the gain if is the last function
                flag = true;
            }
            if(flag){
                j = 0;
            }else{
                j++;
            }

            flag = false;

        }else{
            o.push_back(out[i]);
        }
        i = i + 3;
    }

    return o;

}