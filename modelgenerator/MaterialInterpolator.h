#ifndef MATERIAL_INTERPOLATOR_H_
#define MATERIAL_INTERPOLATOR_H_


#include "FileHandler.h"


class MaterialInterpolator
{
public:
	/** @brief Constructor for creating a lookup table based on the scaling approach. 
	* Use the function sigma_with_deg_dis() and pass the result to the function WriteFile called on an object of type FileHandler to the write the results into a file.
	*
	*  @param ref_material contains a matrix contain the standard deviation values of reference material (whitepaper) used for the scaling approach, it must contain the values for 0 degree in the first column
	*  @param distance_ref contains the distance for the matrix of standard deviation values of the reference material
	*  @param k1 is the k1 parameter estimate for the scaling approach
	*  @param k2 is the k2 parameter estimate for the scaling approach
	*  @param material_intensity are the intensity values of the material we want to estimate the standard deviation values for, they must be measured relative to the intensity values of reference material at 0 degree at the same distance
	*  @param deg_intensity are corresponding degree values to the material_intensity vector  
	*  @param maxrange the maxium distance for which data should be generated (max degree is always 90)
	*  @param step_width_interp_dist the step width regarding distance between two values of the generated standard deviations
	*  @param step_width_interpolate_deg the step width regarding degrees between two values of the generated standard deviations
	*/
	MaterialInterpolator(std::vector<std::vector<double>> ref_material, std::vector<double> distance_ref,float k1,float k2, std::vector<double> material_intensity,std::vector<double> deg_intensity,double maxrange,double step_width_interp_dist,double step_width_interpolate_deg);
	/** @brief Constructor for creating a lookup table based on the offset approach. 
	* Use the function sigma_with_deg_dis() and pass the result to the function WriteFile called on an object of type FileHandler to the write the results into a file.
	*
	*  @param ref_material_1 contains a matrix contain the standard deviation values of the first reference material (whitepaper) used for the offset approach, it must contain the values for 0 degree in the first column
	*  @param dist_ref_1 contains the distance data for the matrix of standard deviation values of the first reference material
	*  @param ref_material_2 contains a matrix contain the standard deviation values of the second reference material (blackepaper) used for the offset approach, it must contain the values for 0 degree in the first column
	*  @param dist_ref_2 contains the distance data for the matrix of standard deviation values of the second reference material
	*  @param relintref2_0deg contains the relative intensity value at 0 degree for ref_material_2, calculate by deviding two intensity values measured at the same distance.
	*	These values are, intensity of the material passed via ref_material_2 and the intensity of the material passed via ref_material_1 (also measured at 0 degree)
	*  @param p1 is the p1 parameter estimate for the offset approach
	*  @param p2 is the p2 parameter estimate for the offset approach
	*  @param p3 is the p3 parameter estimate for the offset approach
	*  @param p4 is the p4 parameter estimate for the offset approach
	*  @param material_intensity are the intensity values of the material we want to estimate the standard deviation values for, they must be measured relative to the intensity values of reference material at 0 degree at the same distance
	*  @param deg_intensity are corresponding degree values to the material_intensity vector  
	*  @param maxrange the maxium distance for which data should be generated (max degree is always 90)
	*  @param step_width_interp_dist the step width regarding distance between two values of the generated standard deviations
	*  @param step_width_interpolate_deg the step width regarding degrees between two values of the generated standard deviations
	*/
	MaterialInterpolator(std::vector<std::vector<double>> ref_material_1,std::vector<double> dist_ref_1,std::vector<std::vector<double>> ref_material_2,std::vector<double> dist_ref_2,double relintref2_0deg,float p1,float p2,float p3,
	float p4,std::vector<double>  material_intensity,std::vector<double> deg_intensity,double maxrange,double step_width_interp_dist,double step_width_interpolate_deg);
	/** @brief Constructor for creating a lookup table based on interpolation of values . 
	* Use the function sigma_with_deg_dis() and pass the result to the function WriteFile called on an object of type FileHandler to the write the results into a file.
	*
	*  @param material contains a matrix contain the standard deviation values for which interpolation should be applied
	*  @param dis contains the distance data for the matrix passed via material
	*  @param deg contains the degree data for the matrix passed via material
	*  @param material_intensity are the intensity values of the material we want to interpolate the standard deviation values for, this parameter is only used when withcrit =true which will write a -1 for the standard deviation at positions where the intensity would drop below a transhold
	*  @param deg_intensity are corresponding degree values to the material_intensity vector, this parameter is only used when withcrit =1 which will write a -1 for the standard deviation at positions where the intensity would drop below a transhold
	*  @param maxrange the maxium distance for which data should be generated (max degree is always 90)
	*  @param step_width_interp_dist the step width regarding distance between two values of the generated standard deviations
	*  @param step_width_interpolate_deg the step width regarding degrees between two values of the generated standard deviations
	*  @param withcrit when set to true, a -1 will be written for the standard deviation at positions where the intensity would drop below a transhold
	*/
	MaterialInterpolator(std::vector<std::vector<double>> material,std::vector<double> dis,std::vector<double> deg,std::vector<double>  material_intensity,std::vector<double> deg_intensity,double maxrange,double step_width_interp_dist,double step_width_interpolate_deg,bool withcrit);
	/** @brief Constructor for creating a lookup table based on interpolation of values for wheel odometry data. 
	* Use the function sigma_with_deg_dis() and pass the result to the function WriteFile called on an object of type FileHandler to the write the results into a file.
	*
	*  @param material contains a matrix contain the standard deviation values for which interpolation should be applied
	*  @param vel contains the velocity data for the matrix passed via material
	*  @param mu contains the mu (dynamic friction) data for the matrix passed via material (given as percentage)
	*  @param maxvel the maxium velocity for which data should be generated (max mu is always 100%=1)
	*  @param step_width_interp_vel the step width regarding velocity between two values of the generated standard deviations
	*  @param step_width_interpolate_mu the step width regarding mu between two values of the generated standard deviations
	*/
	MaterialInterpolator(std::vector<std::vector<double>> material,std::vector<double> vel,std::vector<double> mu,double maxvel,double step_width_interp_vel,double step_width_interpolate_mu);
	//Destructor	
	~MaterialInterpolator();
	//returns the interpolated degree values
	std::vector<double> interpolated_deg();
	//returns the interpolated distance values
	std::vector<double> interpolated_distance();
	//print the interpolated intensities
	void PrintInterpolateIntensities();
	//prints the matrix used for interpolation only usable with 3rd Constructor!
	void PrintMat(); 
	//print the interpolated / predicted matrix
	void PrintpredictedMat();
	//return calculated predicted sigma values
	std::vector<std::vector<double>> mat_sigma();
	//return calculated predicted sigma values with the row and column infos
	std::vector<std::vector<double>> sigma_with_deg_dis();
private:
	//predict the data of the material based on the offset approach (used within 2nd constructor)
	void InterpolateOffset();
	//predict the data of the material based on the scaling approach (used within 1st constructor)
	void InterpolateScaling();
	//predict the data of the material based on the simple interpolation approach (used within 3rd constructor)
	void InterpolateSimple();
	//interpolates the intensity values up to 90 degrees based on the step_width passed to the constructor between two degree values
	void InterpolateIntensities(std::vector<double>& material_intensity_data,std::vector<double> deg);
	bool withcrit_;//this feature is only used for ref material interpolation and enables a check if at the distance for which we calculated a sigma value a signal would still be sensed due to the intensity of the reflected signal at that position. For this check the intensity divided by distance^2 = a is divided by the intensity/distance^2 of the last distance where data still could be received. If the result is >= 1 data is received if not a -1 gets written for sigma value
	float k1_,k2_,p1_,p2_,p3_,p4_;//the parameters for scaling and offset approach
	double step_width_interpolate_deg_;//the step width between two degree (mu for 4th Constructor) values
	double step_width_interp_dist_;//the step width between two distance values
	double maxrange,maxdeg;//maximum range and maximum degree value (90 for the 1-3 Constructor 100 as it refers to mu for the 4th Constructor
	double blackrelint;//relative to first reference material at 0 degree intensity value for second material at 0 degree
	std::vector<std::vector<double>> material_sigma_;//the calculated predicted sigma values of the material
	std::vector<double> distance_ref_1_;//the read in distances of the data of reference material 1
	std::vector<double> deg_ref_1_;//the read in degrees of the data of reference material 1)
	std::vector<double> material_intensity_;//the read in relative intensities of the material which we want to predict sigma values for (relative to white at 0 degree)
	std::vector<double> material_intensity_inter_;//the interpolated relative intensities of the material
	std::vector<double> deg_intensity_;//the corresponding degrees to the intensity valeus
	std::vector<std::vector<double>> ref_material_1_;//the read in standard deviations of the reference material (reference material 1 here white)
	std::vector<std::vector<double>> ref_material_2_;//the read in standard deviations of the reference material (reference material 2 here black)
	std::vector<double> distance_ref_2_;//the read in distances of the data of the reference material 2

};

#endif
