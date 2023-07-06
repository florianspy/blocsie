#include <cmath>
#include <vector>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <chrono>
#include <thread>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "rangeng.h"
#include "Cammat.h"

#define foreach BOOST_FOREACH
#define NUM_THREADS 8

using namespace cv;
using namespace std;

std::vector<std::thread> threads;


class RGB{
public:
	RGB(){
		cameraMatrix = Mat::eye(3, 3,  CV_32F);
		distorsionMatrix = Mat::zeros(5, 1,  CV_32F);   
		nb=0.2*0.2;
		na=1.0/30.0;   
	}        
	int splitLoop(int tid){
		//Clipped_noisy_images_Heteroskedastic_modeling_and_.pdf page 7
		//so we add up the poission distrubiton a*P(chi*y(x)) with the gaussian distrubiton sqrt(b)*ng(x)
		//keep in mind P(chi*y()) and ng(x) are actual functions
		//http://www.kaspercpa.com/statisticalreview.htm#:~:text=Multiplying%20a%20random%20variable%20by%20a%20constant%20increases%20the%20variance,the%20random%20variables%20are%20independent.
		//for the variance the following is valid c^2*variance(x)=variance(c*x)
		//=> variance(sqrt(b)*ng(x))=b*variance(ng(x))=b as variance of ng(x) is 1
		//=> variance(P(chi*y)/chi))=chi*y/chi^2=y/chi=a*y => variance(P(z/255)*255)=255^2*z/255=255*z
		//=>variance=a*y+b
		//for the mean the following applies E(c*x)=c*E(x)
		//=> E(sqrt(b)*ng(x))=0 if ng(x) was with zero mean
		//=> E(P(chi*y)/chi)=chi*y/chi=y => E(P(z/255)*255)=z
		//=> E=y
		//for the variance of this term we get a*(chi*)+ b
		//the second summand is the gaussian variance with zero mean and multiplied by sqrt(b) which lead to a variance of b
		//the first summand is the possion variance with chi*y mean  which is multiplied bya
	   vector<boost::taus88> enginevec;
	   vector<boost::random::poisson_distribution<int>> bpd;
	   float nla=na;
	   float m;
	   for(int i=1;i<256;i++){
		   m= (i/(nla));
		   enginevec.push_back(boost::taus88());
		   
		   boost::random::poisson_distribution<int>::param_type newParams=boost::random::poisson_distribution<int>::param_type(m);
		   boost::random::poisson_distribution<int> test;
		   test.param(newParams);
		   bpd.push_back(test);
	   }
	   float sqb=sqrt(nb);
	   vector<boost::variate_generator<boost::taus88,boost::random::poisson_distribution<int>>> vdlist;
	   for(int i=1;i<256;i++){
		   //newParams=boost::random::poisson_distribution<int>::param_type(m);
		   //bpd.param(newParams);
		   boost::variate_generator<boost::taus88,boost::random::poisson_distribution<int> > vd(enginevec[i-1],bpd[i-1]);
		   vdlist.push_back(vd);
	   }

	   boost::taus88 engine2;
	   boost::normal_distribution<float> nd(0,1);
	   boost::variate_generator<boost::taus88,boost::normal_distribution<float> > var_nor(engine2, nd);
	   uchar z;          
	   cv::Vec3b* ptr;
	   int k=0;
	   int x=0;
	   int y=0;
	   std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
	   std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
	   begin1 = std::chrono::steady_clock::now();
	   int start = (tid * img_ptr->image.rows/NUM_THREADS);
	   int end = (start + img_ptr->image.rows/NUM_THREADS);
	   //as each thread handles other parts of the image the values of ptr will differ for the threads however after each run they will be the same.
	   if(nla != 0){
		   for (y = start; y < end;y++){
			    ptr = img_ptr->image.ptr<cv::Vec3b>(y);
			    for (x = 0; x < img_ptr->image.cols;x++){
					for(k=0;k<3;k++){
					    z=ptr[x][k];
					    if(z!= 0){
						   //https://stackoverflow.com/questions/2078474/how-to-use-boost-normal-distribution-classes
						   ptr[x][k]=sqb*var_nor()+vdlist[z-1]()*nla;
					    }
					    else{
							ptr[x][k]+=sqb*var_nor();
					    }

					}
			    }
			}
		   end1 = std::chrono::steady_clock::now();
		   //std::cout << "It took thread"<<tid<<" "<<std::chrono::duration_cast<std::chrono::milliseconds>(now_ms.time_since_epoch()).count()<<" " << std::chrono::duration_cast<std::chrono::microseconds>(end1 - begin1).count() << "[µs]" << std::endl;
	   }
	   else{
		   for (y = start; y < end;y++){
				  ptr = img_ptr->image.ptr<cv::Vec3b>(y);
				  for (x = 0; x < img_ptr->image.cols;x++){
						for(k=0;k<3;k++){
							ptr[x][k]+=sqb*var_nor();
						}
				  }
		   }
	   }
	   return 1;
    }

	//create maping for distortion by passing the distortion coeffienct and the two matrix where the pixel mapping is stored
	void createdistortion(int rows,int cols,float _k1,float _k2,float _k3,float _p1,float _p2,float _fx,float _fy,float _cx,float _cy,bool additionalfill){
		float s,t,r2,d1,newx,newy;
		cout<<"Parameters k1 k2 k3 p1 p2 fx cx fy cy: "<<_k1<<" "<<_k2<<" "<<_k3<<" "<<_p1<<" "<<_p2<<" "<<_fx<<" "<<_cx<<" "<<_fy<<" "<<_cy<<endl;
		//https://ksimek.github.io/2013/08/13/intrinsic/ true pinhole distortion fx and fy have the same value
		map_x=Mat(rows,cols,  CV_32FC1,Scalar(-1));
		map_y=Mat(rows,cols,  CV_32FC1,Scalar(-1));	   
		for (int y = 0; y < rows;y++){
			for (int x = 0; x < cols;x++){
				//https://github.com/osrf/gazebo/blob/feff7f080147c9feab40dfab7b4eb5a4937cae8a/gazebo/rendering/Distortion.cc in gazebo lines 546-558
				//https://de.mathworks.com/help/vision/ug/camera-calibration.html
				//cols*0.5 =cx
				//https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#undistortpoints
				s = (x - _cx)/(_fx);
				t = (y - _cy)/(_fy);
				r2= s*s+t*t;
				d1 =1+ _k1*r2+_k2*r2*r2+_k3*r2*r2*r2;
				//float d3 = 1/(1-_k1*r2+_k2*r2*r2+_k3*r2*r2*r2);
				//map_x.at<float>(y,x)=(s+2*_p1*s*t+_p2*(r2+2*s*s))*d3*_fx+_cx;
				//map_y.at<float>(y,x)=(t+_p1*(r2+2*t*t)+2*_p2*s*t)*d3*_fy+_cy;
				newx=(s*d1+_p2*(r2+2*s*s)+2*_p1*s*t)*_fx+_cx;
				newy=(t*d1+_p1*(r2+2*t*t)+2*_p2*s*t)*_fy+_cy;
				float florx=floor(newx);
				float flory=floor(newy);
				float ceilx=ceil(newx);
				float ceily=ceil(newy);
				if(florx<cols && florx>=0 && flory<rows && flory>=0){
					 map_x.at<float>(flory,florx)=x;
					 map_y.at<float>(flory,florx)=y;
				}
				if(florx<cols && florx>=0 && ceily<rows && ceily>=0){
					 map_x.at<float>(ceily,florx)=x;
					 map_y.at<float>(ceily,florx)=y;
				}
				if(ceilx<cols && ceilx>=0 && flory<rows && flory>=0){
					 map_x.at<float>(flory,ceilx)=x;
					 map_y.at<float>(flory,ceilx)=y;
				}
				if(ceilx<cols && ceilx>=0 && ceily<rows && ceily>=0){
					 map_x.at<float>(ceily,ceilx)=x;
					 map_y.at<float>(ceily,ceilx)=y;
				}
			}
		}
		cout<<"part without autofill done"<<endl;
		if(additionalfill == true){
			Mat map_x2=map_x.clone();
			Mat map_y2=map_y.clone();
			for (int y = 0; y < rows;y++){
				for (int x = 0; x < cols;x++){
					if(map_x.at<float>(y,x)== -1){
							float value=0;
							float valuey=0;
							int datan=0;
							int value2=0;
							//sometimes there is data that can be read and no segmentation fault occurs when we set it to cols and not rows
							if(y+1<rows){
									value2=map_x.at<float>(y+1,x);
									if(value2!=-1){
											valuey=valuey+map_y.at<float>(y+1,x);
											value=value+value2;
											datan=datan+1;
									}
									if(x+1<cols){
											value2=map_x.at<float>(y+1,x+1);
											if(value2!=-1){
													valuey=valuey+map_y.at<float>(y+1,x+1);
													value=value+value2;
													datan=datan+1;
											}
									}
									if(x-1>0){
											value2=map_x.at<float>(y+1,x-1);
											if(value2!=-1){
													valuey=valuey+map_y.at<float>(y+1,x-1);
													value=value+value2;
													datan=datan+1;
											}
									}
							}
							if(y-1>0){
									value2=map_x.at<float>(y-1,x);
									if(value2!=-1){
											valuey=valuey+map_y.at<float>(y-1,x);
											value=value+value2;
											datan=datan+1;
									}
									if(x+1<cols){
											value2=map_x.at<float>(y-1,x+1);
											if(value2!=-1){
													valuey=valuey+map_y.at<float>(y-1,x+1);
													value=value+value2;
													datan=datan+1;
											}
									}
									if(x-1>0){
											value2=map_x.at<float>(y-1,x-1);
											if(value2!=-1){
													valuey=valuey+map_y.at<float>(y-1,x-1);
													value=value+value2;
													datan=datan+1;
											}
									}
							}
							if(x+1<cols){
									value2=map_x.at<float>(y,x+1);
									if(value2!=-1){
											valuey=valuey+map_y.at<float>(y,x+1);
											value=value+value2;
											datan=datan+1;
									}
							}
							if(x-1>0){
									value2=map_x.at<float>(y,x-1);
									if(value2!=-1){
											valuey=valuey+map_y.at<float>(y,x-1);
											value=value+value2;
											datan=datan+1;
									}
							}
							if(value != 0){
									map_x2.at<float>(y,x)=value/datan;
									map_y2.at<float>(y,x)=valuey/datan;
							}
					}
				}
			}
			map_x=map_x2.clone();
			map_y=map_y2.clone();
		}
	}
	//ImageConstPtr or CompressedImageConstPtr
	sensor_msgs::ImagePtr camimg(const sensor_msgs::ImageConstPtr& message){
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		if(!cameraMatrix.empty()){            
			if(message->encoding.compare(string("bgra8"))==0 || message->encoding.compare(string("rgb8"))==0){
				// directly convert bgra8 to bgr if we dont convert here later part of the image will not get noise as we noise a 3xwidthxlength matrix but the matrix transmitted would be 4xlengthxwidth(a channel!)
				img_ptr=cv_bridge::toCvCopy(message, "bgr8");
			}              
			else{
				cout<<"Invalid color encoding"<<endl;
				return img_ptr->toImageMsg();
			}
			end = std::chrono::steady_clock::now();
			std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
			end = std::chrono::steady_clock::now();
			//Realistic lens  distortion  rendering
			if(p1!=_p1 || p2!=_p2 || k1!=_k1 ||k2!=_k2 ||k3!=_k3 ){
				cout<< "Creating Distortion : "<<img_ptr->image.rows  << endl;
				createdistortion(img_ptr->image.rows,img_ptr->image.cols,_k1,_k2,_k3,_p1,_p2,_fx,_fy,_cx,_cy,true);
				cout<< "Done Creating Distortion : "  << endl;
				p1=_p1;
				p2=_p2;
				k1=_k1;
				k2=_k2;
				k3=_k3;
				//reset only required when camera matrix changed
			}
			end = std::chrono::steady_clock::now();
			begin1 = std::chrono::steady_clock::now();
			threads.clear();
			for (int i = 0; i < NUM_THREADS; i++) {
				threads.push_back(std::thread(&RGB::splitLoop,this,i));
			}
			for (auto &th : threads) {
				th.join();
			}
			end = std::chrono::steady_clock::now();
			remap(img_ptr->image, img_ptr->image, map_x, map_y, INTER_LINEAR);
			Mat mask;
			inRange(img_ptr->image, Scalar(0, 0, 0), Scalar(0, 0, 0), mask);
			img_ptr->image.setTo(Scalar(255, 255,255),mask);		
			//undistort the image again?
			/*Mat cameraMatrix2 = Mat::eye(3, 3,  CV_32F);
			cameraMatrix2.at<float>(0,0)=cameraMatrix.at<float>(0,0);cameraMatrix2.at<float>(1,1)=cameraMatrix.at<float>(1,1);
			cameraMatrix2.at<float>(0,2)=cameraMatrix.at<float>(0,2);cameraMatrix2.at<float>(1,2)=cameraMatrix.at<float>(1,2);
			cv::undistort(img_ptr->image,undistort, cameraMatrix2, distorsionMatrix);
			img_ptr->image=undistort;*/
			begin1 = std::chrono::steady_clock::now();
			image_msg = img_ptr->toImageMsg();
			image_msg->encoding="bgr8";
			end = std::chrono::steady_clock::now();
			std::cout << "Time for image proccessing = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
			return image_msg;
	    }
    }

	void caminfo(const sensor_msgs::CameraInfoPtr& message){
		cameraMatrix.at<float>(0,0)=message->K[0];
		cameraMatrix.at<float>(0,2)=message->K[2];
		cameraMatrix.at<float>(1,1)=message->K[4];
		cameraMatrix.at<float>(1,2)=message->K[5];
		if(message->D.size() != 0){
			distorsionMatrix.at<float>(0, 0)=message->D[0];
			distorsionMatrix.at<float>(0, 1)=message->D[1];
			distorsionMatrix.at<float>(0, 2)=message->D[2];
			distorsionMatrix.at<float>(0, 3)=message->D[3];
			distorsionMatrix.at<float>(0, 4)=message->D[4];
		}
		_fx = cameraMatrix.at<float>(0,0);
		_fy = cameraMatrix.at<float>(1,1);
		_cx = cameraMatrix.at<float>(0,2);
		_cy = cameraMatrix.at<float>(1,2);
		_k1 = distorsionMatrix.at<float>(0, 0);
		_k2 = distorsionMatrix.at<float>(0, 1);
		_p1 = distorsionMatrix.at<float>(0, 2);
		_p2 = distorsionMatrix.at<float>(0, 3);
		_k3 = distorsionMatrix.at<float>(0, 4);
		height = message->height;
		width = message->width;
		cout<<"Caminfo"<<_fx<<" fy"<< _fy <<"cx "<< _cx <<"cy "<< _cy <<"k1 "<< _k1  <<"k2 "<< _k2<<"k3 "<< _k3<<endl;
	}
	Mat getmap_x(){return map_x;}
	Mat getmap_y(){return map_y;}
private:
	float nb,na;
	Mat undistort,cut;
	Mat map_x,map_y;
	sensor_msgs::ImagePtr image_msg;
	cv_bridge::CvImagePtr img_ptr;
	//distortion coefficients required to recognize changes in distortion coefficients
	float k1=-1,k2=-1,k3=-1,p1=0,p2=0;
	float _k1=0,_k2=0,_k3=0,_p1=0,_p2=0,_fx,_fy,_cx,_cy;
	int width,height;
	Mat cameraMatrix;
	Mat distorsionMatrix;
};

vector<string> split_id (const string &s, char delim) {
    vector<string> result;
    stringstream ss (s);
    string item;
    while (getline (ss, item, delim)) {
        result.push_back (item);
    }
    return result;
}
class DepthCam
{
public:
	DepthCam(double st_dist,int st_deg,int am_materials,double max_range,int max_deg){
		depthcameraMatrix = Mat::eye(3, 3,  CV_32F);
		depthdistorsionMatrix = Mat::zeros(5, 1,  CV_32F);
		//engine=boost::kreutzer1986( time(0) );
		stepw_dist_model=st_dist;
		stepw_deg_model=st_deg;
		am_materials_=am_materials;
		max_deg_=max_deg;
		max_range_=max_range;
		rnpabove= new  RandomGaussian();
		for(int i=0;i<am_materials;i++){
			std_devtable.push_back({});
		}
	}
	//create maping for distortion by passing the distortion coeffienct and the two matrix where the pixel mapping is stored
	void createdistortion(int rows,int cols,float _k1,float _k2,float _k3,float _p1,float _p2,float _fx,float _fy,float _cx,float _cy){
		RGB dis=RGB();
		dis.createdistortion(rows,cols,_k1,_k2,_k3,_p1,_p2,_fx,_fy,_cx,_cy,true);
		dmap_x=dis.getmap_x();
		dmap_y=dis.getmap_y();
	}
	sensor_msgs::ImagePtr depth(const lidarmatmsg::Cammat::ConstPtr& message){
		vector<string> frameid=split_id(message->header.frame_id,' ');
		sensor_msgs::ImagePtr msg(new sensor_msgs::Image());
		msg->header=message->header;
		msg->height=message->height;
		msg->width=message->width;			
		msg->encoding=message->encoding;			
		msg->is_bigendian=message->is_bigendian;			
		msg->step=message->step;
		msg->data=message->depth;	
		if(message->encoding.compare(std::string("mono8")) == 0){
			depthimage = cv_bridge::toCvCopy(msg,"mono8")->image;
			float multiplyer= 1;
			depthimage.convertTo(depthimage,CV_32FC1,multiplyer);
		}
		else if(message->encoding.compare(string("32FC1"))==0){
			depthimage = cv_bridge::toCvCopy(msg,"32FC1")->image;
		}
		else{
			cout<<"Invalid depth encoding"<<endl;
			return cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthimage).toImageMsg();
		}
		//check if a value was written into the depth dfx
		if(_dfx!=-1){
			if(dp1!=_dp1 || dp2!=_dp2 || dk1!=_dk1 ||dk2!=_dk2 ||dk3!=_dk3 ){
				createdistortion(depthimage.rows,depthimage.cols,_dk1,_dk2,_dk3,_dp1,_dp2,_dfx, _dfy, _dcx,_dcy);
				dp1=_dp1;
				dp2=_dp2;
				dk1=_dk1;
				dk2=_dk2;
				dk3=_dk3;
				//reset only required when camera matrix changed
			}
			//distortion due to camera
			remap(depthimage, depthimage, dmap_x,dmap_y, INTER_LINEAR);
			float precisionnoise = 0.0;
			int degindex;
			int disindex=0;
			for(unsigned int l=0;l<depthimage.total();l++){
				if(depthimage.at<float>(l)>max_range_){
					depthimage.at<float>(l)=max_range_;
				}
				if(depthimage.at<float>(l)==0.0 || depthimage.at<float>(l)==max_range_){
						//nothing to be done
				}
				else{	
					int mat=(int)message->mat[l];
					int ang=(int)message->ang[l];
					degindex=abs(ang/stepw_deg_model);	
					double ind = abs(ang/(stepw_deg_model*1.0));
					//is it closer to the lower or the higher value of the lookuptable, this checks if its closer to the higher value and then sets the value to look at accordingly				

					if(ind-((int)ind)>0.5){				
						degindex++;

					}
					disindex=depthimage.at<float>(l)/stepw_dist_model;
					ind=depthimage.at<float>(l)/stepw_dist_model;
					if(ind-((int)ind)>0.5){				
						disindex++;
					}		
					float std_dev=std_devtable[mat][disindex][degindex];
					if(std_dev != -1){
						if(std_dev<0){std_dev=0;}
							precisionnoise=rnpabove->Generate(std_dev);
							depthimage.at<float>(l)=depthimage.at<float>(l)+ precisionnoise;
					}
					else{
						depthimage.at<float>(l)=0;
					}

					//value with noise is below or above max /min range
					if(depthimage.at<float>(l)>max_range_){
						depthimage.at<float>(l)=max_range_;
					}
					if(depthimage.at<float>(l)<0.0){
						depthimage.at<float>(l)=0.0;
					}
				}
			}
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthimage).toImageMsg();
			msg->header=message->header;
			msg->header.frame_id=frameid[0];
			//gives the depth message the latest timestamp of a color image
			(*msg).encoding="32FC1";
			 cv::imshow("Distorted", depthimage);
			cv::waitKey(1);
			return msg;
		}
	}
	void depthcaminfo(const sensor_msgs::CameraInfoPtr& message){
		cout<<"In depthcaminfo";
		depthcameraMatrix.at<float>(0,0)=message->K[0];
		depthcameraMatrix.at<float>(0,2)=message->K[2];
		depthcameraMatrix.at<float>(1,1)=message->K[4];
		depthcameraMatrix.at<float>(1,2)=message->K[5];
		if(message->D.size() != 0){
			depthdistorsionMatrix.at<float>(0, 0)=message->D[0];
			depthdistorsionMatrix.at<float>(0, 1)=message->D[1];
			depthdistorsionMatrix.at<float>(0, 2)=message->D[2];
			depthdistorsionMatrix.at<float>(0, 3)=message->D[3];
			depthdistorsionMatrix.at<float>(0, 4)=message->D[4];
		}
		_dfx = depthcameraMatrix.at<float>(0,0);
		_dfy = depthcameraMatrix.at<float>(1,1);
		_dcx = depthcameraMatrix.at<float>(0,2);
		_dcy = depthcameraMatrix.at<float>(1,2);
		_dk1 = depthdistorsionMatrix.at<float>(0, 0);
		 _dk2 = depthdistorsionMatrix.at<float>(0, 1);
		 _dp1 = depthdistorsionMatrix.at<float>(0, 2);
		 _dp2 = depthdistorsionMatrix.at<float>(0, 3);
		 _dk3 = depthdistorsionMatrix.at<float>(0, 4);
		dheight = message->height;
		dwidth = message->width;
		cout<<"End depthcaminfo2";
	}
	bool insert_datamodel_matdata(int mat_index,std::vector<std::vector<double>> sd_mat){
		if(mat_index>am_materials_){
			return false;
		}
		unsigned int dimy=max_range_/stepw_dist_model+1;
		unsigned int dimx=max_deg_/stepw_deg_model+1;
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
private:
        //for depth
        Mat dmap_x,dmap_y;
        Mat depthimage;
        //distortion coefficients required to recognize changes in distortion coefficients
        float dk1=-1,dk2=-1,dk3=-1,dp1=-1,dp2=-1;
        float _dk1,_dk2,_dk3,_dp1,_dp2,_dfx=-2,_dfy,_dcx,_dcy;
        int dwidth,dheight;
        Mat depthcameraMatrix;
        Mat depthdistorsionMatrix;
	std::vector<std::vector<std::vector<double>>> std_devtable;
	double stepw_dist_model;
	int stepw_deg_model;
	int am_materials_;
	int max_deg_;
	double max_range_;
	RandomGaussian *rnpabove;
};


