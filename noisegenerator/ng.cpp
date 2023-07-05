#include "rangeng.h"
#include "camng.h"

double ReturnDiff(std::vector<double> vec)
{
    return (vec[1] - vec[0]);
}
int compare(std::string a, std::vector<std::string> vec){
	for(unsigned int i=0;i<vec.size();i++){
		if(vec[i]==a){
			return i+1;
		}
	}
	return 0;
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
	int amount_of_materials_range=4;
	int deg_step_width_range;// degrees between two values in the lookup table
	double dist_step_width_range;//distance in m between two values in the lookup table
	double max_range_range;// maximum value of the distance in lookup table
	int max_deg_range;// maximum value of the degree in lookup table
	double conststd=0.02;
	//get the data from data model and fill it in
 	std::string path_to_model = static_cast<std::string>("/home/catkin_ws/src/interpolate/src/");
	FileHandler reader(path_to_model+"SICK_white_interpolate.txt", ' ', ':');
	deg_step_width_range = ReturnDiff(reader.col_info_vec());
	dist_step_width_range = ReturnDiff(reader.row_info_vec());
	max_range_range=reader.row_info_vec().back();
	max_deg_range=reader.col_info_vec().back();	
	RangeSensor lidar=RangeSensor(dist_step_width_range,deg_step_width_range,amount_of_materials_range,max_range_range,max_deg_range,conststd);	
	std::vector<std::string> lidarmatandangtopics;
	lidarmatandangtopics.push_back(std::string("/scan_frontmanda2"));
	lidarmatandangtopics.push_back(std::string("/scan_frontmanda"));
	std::vector<std::string> lidaroutputtopics;
	lidaroutputtopics.push_back(std::string("/scan_front2"));
	lidaroutputtopics.push_back(std::string("/scan_front"));
	std::string setting1 = outputPath + "";
	mkdir(setting1.c_str(),0777);
	std::vector<RGB> colorcams;
	colorcams.push_back(RGB());
	colorcams.push_back(RGB());
	int amount_of_materials_dcam=4;
	int deg_step_width_dcam;// degrees between two values in the lookup table
	double dist_step_width_dcam;//distance in m between two values in the lookup table
	double max_range_dcam;// maximum value of the distance in lookup table
	int max_deg_dcam;// maximum value of the degree in lookup table
	deg_step_width_dcam = ReturnDiff(reader.col_info_vec());
	dist_step_width_dcam = ReturnDiff(reader.row_info_vec());
	max_range_dcam=reader.row_info_vec().back();
	max_deg_dcam=reader.col_info_vec().back();
	DepthCam depthcam = DepthCam(dist_step_width_dcam,deg_step_width_dcam,amount_of_materials_dcam,max_range_dcam,max_deg_dcam);
	std::vector<std::string> caminfos;
	caminfos.push_back(std::string("/cam_info2"));  
	caminfos.push_back(std::string("/cam_info"));  
	std::vector<std::string> imgtopics;
	imgtopics.push_back(std::string("/camera/color/image_raw2"));	
	imgtopics.push_back(std::string("/camera/color/image_raw"));	
	std::vector<std::string> depthtopics;
	depthtopics.push_back(std::string("/depth"));	
	depthtopics.push_back(std::string("/depth2"));	
	std::vector<std::string> depthcaminfos;
	depthcaminfos.push_back(std::string("/caminfo2"));  
	depthcaminfos.push_back(std::string("/caminfo"));  
	lidar.insert_datamodel_matdata(0,reader.file_data());
	depthcam.insert_datamodel_matdata(0,reader.file_data());
	reader.ReadFile(path_to_model+"SICK_grey_interpolate.txt");
	lidar.insert_datamodel_matdata(1,reader.file_data());
	depthcam.insert_datamodel_matdata(1,reader.file_data());
	reader.ReadFile(path_to_model+"SICK_black_interpolate.txt");
	lidar.insert_datamodel_matdata(2,reader.file_data());
	depthcam.insert_datamodel_matdata(2,reader.file_data());
	reader.ReadFile(path_to_model+"SICK_alu_interpolate.txt");
	lidar.insert_datamodel_matdata(3,reader.file_data());
	depthcam.insert_datamodel_matdata(3,reader.file_data());
	std::vector<DepthCam> depthcams;
	depthcams.push_back(depthcam);depthcams.push_back(depthcam);
	std::vector<RangeSensor> rangesens;
	rangesens.push_back(lidar);rangesens.push_back(lidar);
	//get first caminfo topic we require it for the distortion information which will be applied on each image
	rosbag::Bag bagIn;
	rosbag::Bag bagOut,bagOut2;
	bagIn.open(bagPath, rosbag::bagmode::Read);
	rosbag::View viewIn(bagIn);
    	foreach(rosbag::MessageInstance const msg, viewIn){
		int res =compare(msg.getTopic(),caminfos);
		int res2 =compare(msg.getTopic(),depthcaminfos);
		if(res){
		       const sensor_msgs::CameraInfoPtr& inf = msg.instantiate<sensor_msgs::CameraInfo>();
		       colorcams[res-1].caminfo(inf);
		}  
		if(res2){
		       const sensor_msgs::CameraInfoPtr& inf = msg.instantiate<sensor_msgs::CameraInfo>();
		       depthcams[res2-1].depthcaminfo(inf);
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
			int res=0,res2=0,res3=0,res4=0;
			foreach(rosbag::MessageInstance const msg, viewIn){
				count++;
				if(count%percentage==0){
					std::cout<<(count*1.0)/viewIn.size()*100.0<<"\% done"<<std::endl;
				}
				res =compare(msg.getTopic(),imgtopics);
				res2 =compare(msg.getTopic(),depthtopics);
				res3 =compare(msg.getTopic(),lidarmatandangtopics);
				res4 =compare(msg.getTopic(),lidaroutputtopics);
				if(res3){					
					lidarmatmsg::Scanmat::ConstPtr msgPtr = msg.instantiate<lidarmatmsg::Scanmat>();
					bagOut.write(lidaroutputtopics[res3-1], msgPtr->header.stamp,rangesens[res3-1].GenerateConstStdMsg(msgPtr));
					bagOut2.write(lidaroutputtopics[res3-1],msgPtr->header.stamp,rangesens[res3-1].GenerateDataDrivenStdMsg(msgPtr));
				}
				else if(res4){
					
				}
				else if(res){
					sensor_msgs::ImageConstPtr imsg = msg.instantiate<sensor_msgs::Image>();
					bagOut.write(msg.getTopic(), imsg->header.stamp,colorcams[res-1].camimg(imsg));
					bagOut2.write(msg.getTopic(), imsg->header.stamp,colorcams[res-1].camimg(imsg));
				}
				else if(res2){
					lidarmatmsg::Cammat::ConstPtr imsg = msg.instantiate<lidarmatmsg::Cammat>();
					bagOut.write(msg.getTopic(), imsg->header.stamp,depthcams[res2-1].depth(imsg));
					bagOut2.write(msg.getTopic(), imsg->header.stamp,depthcams[res2-1].depth(imsg));
				}
				else{
					bagOut.write(msg.getTopic(), msg.getTime(), msg);
					bagOut2.write(msg.getTopic(), msg.getTime(), msg);
					
				}
			}
			bagOut.close();
			bagOut2.close();
	}
	return 0;
}
