#include "rangen.h"

#define foreach BOOST_FOREACH


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
	bool first=true;
	int degindex;
	int disindex=0;
        for(unsigned int l=0;l<ranges.size();l++){
                if(ranges[l]==0.0 || ranges[l]==param.maxDist){
                        //nothing to be done
                }
                else{		

			degindex=abs(msg->angle[l]/param.stepw_deg_model);	
			disindex=ranges[l]/param.stepw_dist_model;
	
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

double ReturnDiff(std::vector<double> vec)
{
    return (vec[1] - vec[0]);
}

int main(int argc, char* argv[]){
	if(argc < 4){
		std::cout<<"Not enough arguments\n";
		return -1;
	}
	std::string bagPath = argv[1];
	std::string outputPath = argv[2];
	//amount of bags you want to create
	int numSamples = atoi(argv[3]);
	int amount_of_materials =4;
	int deg_step_width=5;
	double dist_step_width=0.5;
	double max_range=10;
	double conststd=0.02;
	int max_deg=85;
	std::vector<std::vector<double>> test;
	for(int k=0;k<21;k++){
		test.push_back({});
		for(int i=0;i<18;i++){
			float a=0.01;
			test[k].push_back(a);
		}
	}	
	//get the data from data model and fill it in
 	std::string path_to_res = static_cast<std::string>("/home/catkin_ws/src/interpolate/src/");
	FileHandler reader(path_to_res+"SICK_white_interpolate.txt", ' ', ':');
	deg_step_width = ReturnDiff(reader.col_info_vec());
	dist_step_width = ReturnDiff(reader.row_info_vec());
	std::cout<<dist_step_width<<std::endl;
	RangeSensor lidar=RangeSensor(std::string(""),dist_step_width,deg_step_width,amount_of_materials,max_range,max_deg,conststd);
	lidar.insert_datamodel_matdata(0,reader.file_data());
	reader.ReadFile(path_to_res+"SICK_grey_interpolate.txt");
	lidar.insert_datamodel_matdata(1,reader.file_data());
	reader.ReadFile(path_to_res+"SICK_black_interpolate.txt");
	lidar.insert_datamodel_matdata(2,reader.file_data());
	reader.ReadFile(path_to_res+"SICK_alu_interpolate.txt");
	lidar.insert_datamodel_matdata(3,test);
	rosbag::Bag bagIn;
	rosbag::Bag bagOut,bagOut2;
	bagIn.open(bagPath, rosbag::bagmode::Read);
	std::string lidarmatandangtopic ="/scan_frontmanda";
	std::string lidaroutputtopic = "/scan_front";
	std::string setting1 = outputPath + "";
	int status1 = mkdir(setting1.c_str(),0777);
	SubscribeAndPublish subpub = SubscribeAndPublish();
	rosbag::View viewIn(bagIn);
    std::vector<std::string> caminfo;
    caminfo.push_back(std::string("/caminfo"));  
	std::vector<std::string> imgtopics;
    imgtopics.push_back(std::string("/camera/color/image_raw"));	
	//get first caminfo topic
    foreach(rosbag::MessageInstance const msg, viewIn){
         if(msg.getTopic()==caminfo[0]){
               const sensor_msgs::CameraInfoPtr& inf = msg.instantiate<sensor_msgs::CameraInfo>();
               subpub.caminfo(inf);
			   std::cout<<"Break"<<std::endl;
				break;
        }  
    }
	//https://answers.ros.org/question/28766/rosbag-info-c-api/
	std::cout<<viewIn.size()<<std::endl;
	int percentage=viewIn.size()/10;
	// less then 10 message in rosbag
	if(percentage == 0){
			percentage=1;
	}
	for(int i=0;i<numSamples;i++){
			std::string path = setting1 + "/noise-conststd"+std::to_string(i)+"-.bag";
			std::string path2 = setting1 + "/noise-datastd"+std::to_string(i)+"-.bag";
			bagOut.open(path, rosbag::bagmode::Write);
			bagOut2.open(path2, rosbag::bagmode::Write);
			int count =0;
			foreach(rosbag::MessageInstance const msg, viewIn){
				count++;
				if(count%percentage==0){
					std::cout<<(count*1.0)/viewIn.size()*100.0<<"\% done"<<std::endl;
				}
				if(msg.getTopic() == lidarmatandangtopic){
					lidarmatmsg::Scanmat::ConstPtr msgPtr = msg.instantiate<lidarmatmsg::Scanmat>();
					bagOut.write(lidaroutputtopic, msgPtr->header.stamp,lidar.GenerateConstStdMsg(msgPtr));
					bagOut2.write(lidaroutputtopic, msgPtr->header.stamp,lidar.GenerateDataDrivenStdMsg(msgPtr));
				}
				else if(msg.getTopic()==imgtopics[0]){
					sensor_msgs::ImageConstPtr imsg = msg.instantiate<sensor_msgs::Image>();
					bagOut.write(msg.getTopic(), imsg->header.stamp,subpub.camimg(imsg));
					bagOut2.write(msg.getTopic(), imsg->header.stamp,subpub.camimg(imsg));

				}
				else if(msg.getTopic() == lidaroutputtopic){
				}
				else{
					if(msg.getTopic() == "/clock"){
						bagOut.write(msg.getTopic(), msg.getTime(), msg);
						bagOut2.write(msg.getTopic(), msg.getTime(), msg);
					}
					else{
						//std::cout<<msg.getTime()<<" ";
						bagOut.write(msg.getTopic(),msg.getTime(), msg);
						bagOut2.write(msg.getTopic(), msg.getTime(), msg);						
					}
				}
			}
			bagOut.close();
			bagOut2.close();
	}
    return 0;
}
