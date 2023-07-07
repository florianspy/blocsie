#pragma once
#ifndef rand_H
#define rand_H
#include <boost/random.hpp>
// This is a global generator that doen't get reset 
static boost::taus88 generator=boost::taus88();
static std::vector<boost::taus88> vect;
static bool init=false;
class RandomNoise{
private:
    boost::variate_generator<boost::taus88 &, boost::normal_distribution<float>> *var_nor;
public:
	/* Constructor creating a random number generator
	@param stdev = standard deviation used
	*/
    RandomNoise(float stdev = 1.0f){
        if(stdev != 0.0f){
            boost::normal_distribution<float> dist(0.0f,stdev);
            var_nor = new boost::variate_generator<boost::taus88 &, boost::normal_distribution<float>>(generator,dist);
        }
    }
	/* Constructor creating 8 a random number generator stored in vect
	@param stdev = standard deviation used
	*/
    RandomNoise(int id,float stdev){
        if(init == false){
            for(int i=0;i<8;i++){
                 vect.push_back(boost::taus88(rand()%1000));                 
            }
            init=true;
        }
        if(stdev != 0.0f){
            boost::normal_distribution<float> dist(0.0f,stdev);
            var_nor = new boost::variate_generator<boost::taus88 &, boost::normal_distribution<float>>(vect[id],dist);
        }
    }
	/* This function generate a random gaussian 
	@param stdev = is not used!
	*/
    virtual float Generate(float stdev) {
            return (*var_nor)();
    }
};

class RandomGaussian {
public: 
    RandomGaussian(){}
	/* This function generate a random gaussian 
	@param stdev = is the standard deviation for the gaussian with mean = 0
	*/
    float Generate(float stdev){
        boost::normal_distribution<float> dist(0.0f,stdev);
        boost::variate_generator<boost::taus88 &, boost::normal_distribution<float>> var_nor(generator,dist);
        return var_nor();
    }
};
#endif
