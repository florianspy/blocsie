// g++  -I/opt/ros/melodic/include -I/home/catkin_ws/devel/include/lidarmatmsg -I/home/catkin_ws/src/interpolate/src -Ofast -g3 -c -Wall -fmessage-length=0 -std=c++14 -MMD -MP -MF  -c noisegn.cpp /home/catkin_ws/src/interpolate/src/FileHandler.cpp  -march=native
//g++ -L/opt/ros/melodic/lib -L/opt/ros/melodic/lib/x86_64-linux-gnu -o "noisegn" ./noisegn.o ./FileHandler.o -ltf2 -ltf -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lroscpp_serialization -lopencv_calib3d -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lmessage_filters -lrostime -lroscpp -lboost_filesystem -lboost_system -lcv_bridge -lrosconsole_backend_interface -lrosconsole -ltf2_ros -lpthread -lboost_thread -lgsl -lgslcblas -lrosbag  -lm -lrt -ldl -lconsole_bridge  -lrosbag_storage -lcv_bridge -lopencv_core -lopencv_imgproc  -march=native
//rosbag write of msg based on msg instance timestamps only works well if you used before the 
//rosbagrewrite.py file that puts the message into the rosbag based on the msg timestamps and not on the received time 
#pragma once
#ifndef rangen_H
#define rangen_H
#include <vector>
#include <string>
#include <iostream>
#include <boost/random.hpp>
#include <sys/stat.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include "Scanmat.h"
#include "FileHandler.h"
#define foreach BOOST_FOREACH
// This struct stores all the parameters passed through the frame_id
// This is a global generator that doen't get reset to inital seed 
static boost::taus88 generator=boost::taus88();
static std::vector<boost::taus88> vect;
static bool init=false;
class RandomNoise{
private:
    boost::variate_generator<boost::taus88 &, boost::normal_distribution<float>> *var_nor;
public:
    RandomNoise(float stdev = 1.0f){
        if(stdev != 0.0f){
            boost::normal_distribution<float> dist(0.0f,stdev);
            var_nor = new boost::variate_generator<boost::taus88 &, boost::normal_distribution<float>>(generator,dist);
        }
    }
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
    virtual float Generate(float stdev) {
            return (*var_nor)();
    }
};

class RandomNoisePercent : public RandomNoise{
public: 
    RandomNoisePercent(){}
    float Generate(float stdev) override{
        boost::normal_distribution<float> dist(0.0f,stdev);
        boost::variate_generator<boost::taus88 &, boost::normal_distribution<float>> var_nor(generator,dist);
        return var_nor();
    }
};
// This struct stores all the parameters passed through the frame_id
struct Parameters {
   std::string lidartopic;
   float maxDist;
   double stepw_dist_model;//stepwidth regarding distance between two values in the data driven model
   int stepw_deg_model;//stepwidth regarding degree between two values in the data driven model
   int am_materials;//amount of materials we have for the data driven model
   double max_range;//max distance value of the data driven model 
   int max_deg;//max degree value of the data driven model
   double conststd;
};

class RangeSensor{
private:
	sensor_msgs::LaserScan msgWithNoise;
	Parameters param;
	std::vector<float> ranges;
	RandomNoise *rnpabove;
	std::vector<std::vector<std::vector<double>>> std_devtable;
	
public:
	RangeSensor(double st_dist,int st_deg,int am_materials,double max_range,int max_deg,double conststd){		
		for(int i=0;i<am_materials;i++){
			std_devtable.push_back({});
		}		
		rnpabove = new RandomNoisePercent();
		param.stepw_dist_model=st_dist;
		param.stepw_deg_model=st_deg;
		param.am_materials=am_materials;
		param.max_deg=max_deg;
		param.max_range=max_range;
		param.conststd=conststd;
	};
	bool insert_datamodel_matdata(int mat_index,std::vector<std::vector<double>> sd_mat);
	sensor_msgs::LaserScan GenerateConstStdMsg(const lidarmatmsg::Scanmat::ConstPtr& msg);
	sensor_msgs::LaserScan GenerateDataDrivenStdMsg(const lidarmatmsg::Scanmat::ConstPtr& msg);
	void InitializeMessage(const lidarmatmsg::Scanmat::ConstPtr& msg);
	void GenerateConstStd(const lidarmatmsg::Scanmat::ConstPtr& msg);
	void GenerateDataDrivenStd(const lidarmatmsg::Scanmat::ConstPtr& msg);
};
sensor_msgs::LaserScan RangeSensor::GenerateConstStdMsg(const lidarmatmsg::Scanmat::ConstPtr& msg){
        ranges = std::vector<float>(msg->ranges); // Copy ranges from the message to the variable ranges
        param.maxDist = msg->range_max;
        param.lidartopic= msg->header.frame_id;
        GenerateConstStd(msg); // Using parsed parameters to apply appropriate accuracies and precisions to ranges
        InitializeMessage(msg);//fill msg

        return msgWithNoise;
}

sensor_msgs::LaserScan RangeSensor::GenerateDataDrivenStdMsg(const lidarmatmsg::Scanmat::ConstPtr& msg){
        ranges = std::vector<float>(msg->ranges); // Copy ranges from the message to the variable ranges
        param.maxDist = msg->range_max;
        param.lidartopic= msg->header.frame_id;
        GenerateDataDrivenStd(msg); // Using parsed parameters to apply appropriate accuracies and precisions to ranges
        InitializeMessage(msg);//fill msg
        return msgWithNoise;
}

void RangeSensor::InitializeMessage(const lidarmatmsg::Scanmat::ConstPtr& msg){
	msgWithNoise.angle_min = msg->angle_min,
	msgWithNoise.angle_max = msg->angle_max,
	msgWithNoise.angle_increment = msg->angle_increment,
	msgWithNoise.time_increment = msg->time_increment,
	msgWithNoise.scan_time = msg->scan_time,
	msgWithNoise.range_min = msg->range_min,
	msgWithNoise.range_max = msg->range_max,
	msgWithNoise.ranges = ranges,
	msgWithNoise.header.stamp = msg->header.stamp,
	msgWithNoise.intensities = msg->intensities,
	msgWithNoise.header.frame_id=param.lidartopic;
}

void RangeSensor::GenerateConstStd(const lidarmatmsg::Scanmat::ConstPtr& msg){
	float precisionnoise = 0.0;
	for(unsigned int l=0;l<ranges.size();l++){
		if(ranges[l]==0.0 || ranges[l]==param.maxDist){
				//nothing to be done
		}
		else{
				precisionnoise=rnpabove->Generate(param.conststd);
				ranges[l]=ranges[l]+ precisionnoise;
				//value with noise is below or above max /min range
				if(ranges[l]>param.maxDist){
						ranges[l]=param.maxDist;
				}
				if(ranges[l]<0.0){
						ranges[l]=0.0;
				}
		}
	}
}
void RangeSensor::GenerateDataDrivenStd(const lidarmatmsg::Scanmat::ConstPtr& msg){
    float precisionnoise = 0.0;
	int degindex;
	int disindex=0;
	for(unsigned int l=0;l<ranges.size();l++){
		if(ranges[l]==0.0 || ranges[l]==param.maxDist){
				//nothing to be done
		}
		else{		
			degindex=abs(msg->angle[l]/param.stepw_deg_model);
			double ind = abs(msg->angle[l]/(param.stepw_deg_model*1.0));
			//is it closer to the lower or the higher value of the lookuptable, this checks if its closer to the higher value and then sets the value to look at accordingly
			if(ind-((int)ind)>0.5){				
				degindex++;
			}
			ind=ranges[l]/param.stepw_dist_model;
			disindex=ranges[l]/param.stepw_dist_model;
			//https://www.quora.com/How-do-you-get-the-number-after-the-decimal-point-in-C
			//is it closer to the lower or the higher value of the lookuptable, this checks if its closer to the higher value and then sets the value to look at accordingly
			if(ind-((int)ind)>0.5){				
				disindex++;
			}			
			float std_dev=std_devtable[msg->material[l]][disindex][degindex];
			if(std_dev != -1){
				if(std_dev<0){std_dev=0;}
							precisionnoise=rnpabove->Generate(std_dev);
							ranges[l]=ranges[l]+ precisionnoise;
			}
			else{
				ranges[l]=0;
			}
			//std::cout<<"material"<<msg->material[l]<<"angle "<<msg->angle[l];
			//value with noise is below or above max /min range
			if(ranges[l]>param.maxDist){
					ranges[l]=param.maxDist;
			}
			if(ranges[l]<0.0){
					ranges[l]=0.0;
			}
		}
	}
}

bool RangeSensor::insert_datamodel_matdata(int mat_index,std::vector<std::vector<double>> sd_mat){
	if(mat_index>param.am_materials){
		return false;
	}
	unsigned int dimy=param.max_range/param.stepw_dist_model+1;
	unsigned int dimx=param.max_deg/param.stepw_deg_model+1;
	if(sd_mat.size()!=dimy){
		std::cout<<"Too few/many distances, required:"<<dimy<<std::endl;
		return false;
	}
	for(unsigned int i=0;i<sd_mat.size();i++){
		if(sd_mat[i].size()!=dimx){
			std::cout<<"Too few/many degree values at distanceindex:"<<i<<", required:"<<dimx<<" provided:"<<sd_mat[i].size()<<std::endl;
			return false;
		}
	}
	std_devtable[mat_index].insert(std_devtable[mat_index].end(),sd_mat.begin(),sd_mat.end());
	return true;
}
#endif
