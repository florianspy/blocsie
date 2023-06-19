// g++  -I/opt/ros/melodic/include -I/home/catkin_ws/devel/include/lidarmatmsg -I/home/catkin_ws/src/interpolate/src -Ofast -g3 -c -Wall -fmessage-length=0 -std=c++14 -MMD -MP -MF  -c rangen.cpp /home/catkin_ws/src/interpolate/src/FileHandler.cpp  -march=native
//g++ -L/opt/ros/melodic/lib -L/opt/ros/melodic/lib/x86_64-linux-gnu -o "rangen" ./rangen.o ./FileHandler.o ./imagedis.o -ltf2 -ltf -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lroscpp_serialization -lopencv_calib3d -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lmessage_filters -lrostime -lroscpp -lboost_filesystem -lboost_system -lcv_bridge -lrosconsole_backend_interface -lrosconsole -ltf2_ros -lpthread -lboost_thread -lgsl -lgslcblas -lrosbag  -lm -lrt -ldl -lconsole_bridge  -lrosbag_storage -lcv_bridge -lopencv_core -lopencv_imgproc  -march=native
//rosbag write of msg based on msg instance timestamps only works well if you used before the 
//rosbagrewrite.py file that puts the message into the rosbag based on the msg timestamps and not on the received time 
#include <vector>
#include <string>
#include <iostream>
#include <boost/random.hpp>
#include <sys/stat.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include "Scanmat.h"
#include "FileHandler.h"
#include "imagedis.h"
#define foreach BOOST_FOREACH
// This struct stores all the parameters passed through the frame_id
// This is a global generator that doen't get reset to begining everytime it is used
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
	RangeSensor(std::string sensorname,double st_dist,int st_deg,int am_materials,double max_range,int max_deg,double conststd){
		if(sensorname == ""){
			for(int i=0;i<am_materials;i++){
				std_devtable.push_back({});
			}


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