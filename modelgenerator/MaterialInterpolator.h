#ifndef MATERIAL_INTERPOLATOR_H_
#define MATERIAL_INTERPOLATOR_H_

#include <map>

#include "FileHandler.h"


class MaterialInterpolator
{
public:
//ref_material_1 dist_ref_1 must contain the values for 0 degree in the first column
MaterialInterpolator(std::vector<std::vector<double>> ref_material_1, std::vector<double> distance_ref,float k1,float k2, std::vector<double> material_intensity,std::vector<double> deg_intensity);
//ref_material_1 dist_ref_1 ref_material_2 dist_ref_2 must contain the values for 0 degree in the first column
    MaterialInterpolator(std::vector<std::vector<double>> ref_material_1,std::vector<double> dist_ref_1,std::vector<std::vector<double>> ref_material_2,std::vector<double> dist_ref_2,float p1,float p2,float p3,float p4,std::vector<double>  material_intensity,std::vector<double> deg);
MaterialInterpolator(std::vector<std::vector<double>> ref_material_1,std::vector<double> dist_ref_1,std::vector<double> deg_ref_1,double maxrang,double step_width_interp_dist,std::vector<double>  material_intensity,std::vector<double> deg,bool withcrit);
    ~MaterialInterpolator();
    // Getter functions
    std::vector<double> intensity_data();
    void change_material_intensity_data(std::vector<double> material_intensity_data,std::vector<double> deg);
    std::map<int, std::vector<std::vector<double>>> material_map();
    std::vector<double> interpolated_deg(){
		std::vector<double> toreturn;
		for(int degree=0;degree<90;degree=degree+step_width_interpolate_deg_){ 
			toreturn.push_back(degree);
		}
		return toreturn;
    }
    std::vector<double> interpolated_distance(){
		std::vector<double> toreturn;
		for(double y=0;y<maxrange+0.01;y=y+step_width_interp_dist_){ 
			toreturn.push_back(y);
		}	
		return toreturn;
    }
    void PrintInterpolateIntensities();
    void Printrefmat1(){
		std::cout<<"Ref material1 data"<<std::endl;
		//print angles
		for(int i=0;i<deg_ref_1_.size();i++){
			std::cout<<deg_ref_1_[i]<<" ";
		}
		std::cout<<std::endl;
		for(int i=0;i<ref_material_1_.size();i++){
			std::cout<<distance_ref_1_[i]<<" ";
			for(int j=0;j<ref_material_1_[i].size();j++){
				std::cout<<ref_material_1_[i][j]<<" ";
			}
			std::cout<<std::endl;
		}
    }
    void PrintpredictedMat(){
        std::cout<<"Predicted material values"<<std::endl;
		//print angles
		for(int i=0;i<90;i=i+step_width_interpolate_deg_){
			std::cout<<i<<" ";
		}
		std::cout<<std::endl;
		double dis=0;
		for(int i=0;i<material_sigma_.size();i++){
			std::cout<<dis<<" ";dis=dis+step_width_interp_dist_;
			for(int j=0;j<material_sigma_[i].size();j++){
				std::cout<<material_sigma_[i][j]<<" ";
			}
			std::cout<<std::endl;
		}
    }
    std::vector<std::vector<double>> mat_sigma();//return calculated predicted sigma values
	std::vector<std::vector<double>> sigma_with_deg_dis();//return calculated predicted sigma values with the row and column infos
private:
    int step_width_interpolate_deg_=5;//before change ensure that old data was stored away
    double step_width_interp_dist_=0.1;
    double maxrange=3.5;
    bool withcrit_;//this feature is only used for ref material interpolation and enables a check if at the distance for which we calculated a sigma value a signal would still be sensed due to the intensity of the reflected signal at that position. For this check the intensity divided by distance^2 = a is divided by the intensity/distance^2 of the last distance where data still could be received. If the result is >= 1 data is received if not a -1 gets written for sigma value
    void InterpolateSICK();
    void InterpolateLDS();
    void InterpolateRefmat();
    std::map<int, std::vector<std::vector<double>>> material_map_;
    std::vector<std::vector<double>> material_sigma_;//the calculated predicted sigma values of the material
    std::vector<double> distance_ref_1_;//the read in distances of the data of reference material 1
    std::vector<double> deg_ref_1_;//the read in degrees of the data of reference material 1)
    std::vector<double> material_intensity_;//the read in relative intensities of the material which we want to predict sigma values for (relative to white at 0 degree)
    std::vector<double> material_intensity_inter_;//the interpolated relative intensities of the material
    std::vector<double> deg_intensity_;//the corresponding degrees to the intensity valeus
    std::vector<std::vector<double>> ref_material_1_;//the read in standard deviations of the reference material (reference material 1 here white)
    std::vector<std::vector<double>> ref_material_2_;//the read in standard deviations of the reference material (reference material 2 here black)
    std::vector<double> distance_ref_2_;//the read in distances of the data of the reference material 2
    float k1_,k2_,p1_,p2_,p3_,p4_;
    void InterpolateIntensities(std::vector<double>& material_intensity_data,std::vector<double> deg);
};

#endif
