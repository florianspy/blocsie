#include "MaterialInterpolator.h"
#include <iomanip>
//we need double values here as our red in values have 16 digits and therefore can only be transfered when using double not float
double interpolate(double low_val,double low_deg,double up_val,double up_deg,double intmed_deg)
{
    double m=(up_val-low_val)/(up_deg-low_deg);
    double t=low_val-m*low_deg;
    return (m*intmed_deg+t);
}
//sd_hdg_ld = stand dev high degree at lower distance
//sd_ldg_hd = sd lower degree at high distance
double interpolatewithangle(double sd_ldg_ld,double sd_ldg_hd,double sd_hdg_ld,double sd_hdg_hd,double ld,double hd,double ldg,double hdg,double interdeg,double interdis){
//10 deg 0.1 10 deg 0.3 -> interpolate for 0.2  20 deg 0.1 20 deg 0.3 -> interpolate for 0.2
//
	double sd_ldg=interpolate(sd_ldg_ld,ld,sd_ldg_hd,hd,interdis);
	double sd_hdg=interpolate(sd_hdg_ld,ld,sd_hdg_hd,hd,interdis);
	double value=interpolate(sd_ldg,ldg,sd_hdg,hdg,interdeg);
	/*double sd_ldg=interpolate(sd_ldg_ld,ldg,sd_hdg_ld,hdg,interdeg);
	double sd_hdg=interpolate(sd_ldg_hd,ldg,sd_hdg_hd,hdg,interdeg);
	double value=interpolate(sd_ldg,ld,sd_hdg,hd,interdis);*/
	return value;
}

MaterialInterpolator::MaterialInterpolator(std::vector<std::vector<double>> ref_material_1, std::vector<double> distance_ref, float k1,float k2, std::vector<double>  material_intensity,std::vector<double> deg,double maxrange,double step_width_interp_dist,double step_width_interpolate_deg) :ref_material_1_(ref_material_1),distance_ref_1_(distance_ref),k1_(k1),k2_(k2),material_intensity_(material_intensity),deg_intensity_(deg)

{
	step_width_interpolate_deg_=step_width_interpolate_deg;
	step_width_interp_dist_=step_width_interp_dist;
	this->maxrange=maxrange;
	InterpolateIntensities(material_intensity,deg_intensity_);
	maxdeg=90;
	InterpolateLDS();
}

MaterialInterpolator::MaterialInterpolator(std::vector<std::vector<double>> ref_material_1,std::vector<double> dist_ref_1,std::vector<std::vector<double>> ref_material_2,std::vector<double> dist_ref_2,double relintref2_0deg,float p1,float p2,float p3,float p4,std::vector<double>  material_intensity,std::vector<double> deg,double maxrange,double step_width_interp_dist,double step_width_interpolate_deg) : ref_material_1_(ref_material_1),distance_ref_1_(dist_ref_1),ref_material_2_(ref_material_2),distance_ref_2_(dist_ref_2),p1_(p1),p2_(p2),p3_(p3),p4_(p4),deg_intensity_(deg)
{
	step_width_interpolate_deg_=step_width_interpolate_deg;
	step_width_interp_dist_=step_width_interp_dist;
	this->maxrange=maxrange;
	this->blackrelint=relintref2_0deg;
	InterpolateIntensities(material_intensity,deg_intensity_);
	maxdeg=90;
	InterpolateSICK();
}

MaterialInterpolator::MaterialInterpolator(std::vector<std::vector<double>> material,std::vector<double> dis,std::vector<double> deg,std::vector<double>  material_intensity,std::vector<double> deg_intensity,double maxrange,double step_width_interp_dist,double step_width_interpolate_deg,bool withcrit):ref_material_1_(material),distance_ref_1_(dis),deg_ref_1_(deg),deg_intensity_(deg_intensity),withcrit_(withcrit)
{
	step_width_interpolate_deg_=step_width_interpolate_deg;
	step_width_interp_dist_=step_width_interp_dist;
	this->maxrange=maxrange;
	InterpolateIntensities(material_intensity,deg_intensity_);
	maxdeg=90;
	InterpolateRefmat();
}
MaterialInterpolator::MaterialInterpolator(std::vector<std::vector<double>> material,std::vector<double> dis,std::vector<double> mu,double maxvel,double step_width_interp_dist,double step_width_interpolate_mu):ref_material_1_(material),distance_ref_1_(dis),deg_ref_1_(mu){
	step_width_interp_dist_=step_width_interp_dist;
	maxrange=maxvel;
	step_width_interpolate_deg_=step_width_interpolate_mu;
	int dimy=maxrange/step_width_interp_dist_;
	maxdeg=100.0;
	int dimx=maxdeg/step_width_interpolate_mu;
	for(int rows=0;rows<dimy+1;rows++){
	    material_sigma_.push_back({});
	}
	int degindex=0; 
	int lastvalue=distance_ref_1_.size()-1;
	for(int deg=0;deg<dimx;deg++){
		int row=0;
		//find the degreeindex for interpolation
		for(degindex=0;degindex<deg_ref_1_.size();degindex++){
			if(step_width_interpolate_deg_*deg<=deg_ref_1_[degindex]){			
					break;
			}
		}
		if(degindex==deg_ref_1_.size()-1){
			degindex=degindex-1;
		}
		if(degindex==deg_ref_1_.size()){
			degindex=degindex-2;
		}
		for(double y=0;y<=maxrange+0.01;y=y+step_width_interp_dist_){
			bool foundlarger=false;		
			for(int x=0;x<distance_ref_1_.size();x++){	
				if(distance_ref_1_[x]>y){
					int distindx=x;			
					if(distindx == 0){distindx=1;}
					float value =-1;
					if(ref_material_1_[distindx-1][degindex+1] == -1 ||ref_material_1_[distindx][degindex+1] == -1 || ref_material_1_[distindx-1][degindex] == -1 || ref_material_1_[distindx][degindex] == -1){}
					else{	

						value = interpolatewithangle(ref_material_1_[distindx-1][degindex+1],ref_material_1_[distindx][degindex+1],ref_material_1_[distindx-1][degindex],ref_material_1_[distindx][degindex],distance_ref_1_[distindx-1],distance_ref_1_[distindx],deg_ref_1_[degindex],deg_ref_1_[degindex+1],step_width_interpolate_deg_*deg,y);
						if(value < 0){
							value=0;
						}
					}
					material_sigma_.at(row).push_back(value);
					foundlarger=true;
					break;
				}
			}
			if(foundlarger==false){
					float value =-1;
					if(ref_material_1_[lastvalue-1][degindex+1] == -1 ||ref_material_1_[lastvalue][degindex+1] == -1 || ref_material_1_[lastvalue-1][degindex] == -1 || ref_material_1_[lastvalue][degindex] == -1){}
					else{

					value = interpolatewithangle(ref_material_1_[lastvalue-1][degindex],ref_material_1_[lastvalue][degindex],ref_material_1_[lastvalue-1][degindex+1],ref_material_1_[lastvalue][degindex+1],distance_ref_1_[lastvalue-1],distance_ref_1_[lastvalue],deg_ref_1_[degindex],deg_ref_1_[degindex+1],step_width_interpolate_deg_*deg,y);}
					if(value < 0){
						value=0;
					}
					material_sigma_.at(row).push_back(value);
					/*if(deg == 0){std::cout<<y<<" "<<value<<" "<<distance_ref_1_[lastvalue-1]<<" "<<ref_material_1_[lastvalue-1][degindex+1]<<" "<<ref_material_1_[lastvalue][degindex+1]<<" "<<ref_material_1_[lastvalue][degindex]<<" "<<deg_ref_1_[degindex]<<" "<<deg_ref_1_[degindex+1]<<std::endl;}*/
			}
	    		row=row+1;}
	}
}




MaterialInterpolator::~MaterialInterpolator()
{

}

void MaterialInterpolator::change_material_intensity_data(std::vector<double> material_intensity_data,std::vector<double> deg)
{
    InterpolateIntensities(material_intensity_data,deg);
}

double calcblackscaleforoffset(double intensity,double blackzero,double p3,double p4){
	return blackzero/intensity*p3+p4;	
}

//blackrelint should become a parameter later
void MaterialInterpolator::InterpolateSICK()
{
	
	int dimy=maxrange/step_width_interp_dist_;
	int dimx=90/step_width_interpolate_deg_;
	for(int rows=0;rows<dimy+1;rows++){
	    material_sigma_.push_back({});
	}
	int blackl=distance_ref_2_.size()-1;
	//deg is not the degrees but the index for the array in which the degrees are stored 
	for(int deg=0;deg<dimx;deg++){
		int row=0;
		for(double y=0;y<=maxrange+0.01;y=y+step_width_interp_dist_){
		    double shiftwhite=1.0/material_intensity_inter_.at(deg)*100.0*p1_+p2_;
		    bool foundoffset=false;
		    for(int x=0;x<distance_ref_1_.size();x++){
			if(distance_ref_1_[x]-shiftwhite>y){
				if(x==0){x=1;}	
				double mt=(ref_material_1_[x][0]-ref_material_1_[x-1][0])/(distance_ref_1_[x]-distance_ref_1_[x-1]);					
				double t=ref_material_1_[x][0]-mt*(distance_ref_1_[x]-shiftwhite);				
				double value=y*mt+t;	
				material_sigma_.at(row).push_back(value);
				foundoffset=true;
				break;
			}
		    }
		   if(foundoffset == false){
			for(int x=0;x<distance_ref_2_.size();x++){
				if(distance_ref_2_[x]>y){
					if(x==0){x=1;}	
					double mt=(ref_material_2_[x][0]-ref_material_2_[x-1][0])/(distance_ref_2_[x]-distance_ref_2_[x-1]);					
					double t=ref_material_2_[x][0]-mt*(distance_ref_2_[x]);				
					double value=y*mt+t;	
					double scalblack=calcblackscaleforoffset(material_intensity_inter_.at(deg)/100.0,blackrelint,p3_,p4_);
					value=value*scalblack;
					material_sigma_.at(row).push_back(value);
					foundoffset=true;
					break;
				}
			}
		    }
		    if(foundoffset == false){
				double mt=(ref_material_2_[blackl][0]-ref_material_2_[blackl-1][0])/(distance_ref_2_[blackl]-distance_ref_2_[blackl-1]);					
				double t=ref_material_2_[blackl][0]-mt*(distance_ref_2_[blackl]);				
				double value=y*mt+t;	
				double scalblack=calcblackscaleforoffset(material_intensity_inter_.at(deg)/100.0,blackrelint,p3_,p4_);
				value=value*scalblack;
				material_sigma_.at(row).push_back(value);
				foundoffset=true;
				break;
		    }
		    row=row+1;
		}
	}
}
//this is the scaling and prediction stuff 
void MaterialInterpolator::InterpolateLDS(){
    int dimy=maxrange/step_width_interp_dist_;
    int dimx=90/step_width_interpolate_deg_;
    material_sigma_.clear();
    //lds circitical value at
    float critical_val=17.9/pow(2.158,2);
   
    //remember we start at 0 so we need +1 more storage fields
    for(int rows=0;rows<dimy+1;rows++){
      	    material_sigma_.push_back({});
    }
    for(int deg=0;deg<dimx;deg++){
	    //order is distance then angles inside the material matrix so we need to put the data in rowwise and not colwise which would use deg
	    int row=0;
	    double scalingf_grey=(k1_/material_intensity_inter_.at(deg)*100.0+k2_);
        //double values we need to add 0.01 to get last value	    
	    for(double y=0;y<=maxrange+0.01;y=y+step_width_interp_dist_){
		//look for closest distance value in reference material
	       for(int x=0;x<distance_ref_1_.size();x++){			
			if(distance_ref_1_[x]>y){
				int distindx=x;
				if(distindx == 0){
					distindx=1;
				}
				float othervalue=material_intensity_inter_.at(deg)/pow(y,2);
				//if value of intensity devided by range is below this critical value we will not be able to get a single so we write a -1
				if(othervalue/critical_val <1){
					material_sigma_.at(row).push_back(-1);
				}
				else{
					//we only take values at 0 degree!! no need to index at other deg values of reference material
					double mt=(ref_material_1_[distindx][0]-ref_material_1_[distindx-1][0])/(distance_ref_1_[distindx]-distance_ref_1_[distindx-1]);
					double t=ref_material_1_[distindx][0]-mt*(distance_ref_1_[distindx]);
					double value=mt*y+t;
					material_sigma_.at(row).push_back(scalingf_grey*value);
					
					if(deg == 0 || deg == 12){
						//std::cout<<y<<" "<<scalingf_grey*value<<" "<<ref_material_1_[distindx][0]<<std::endl;
					}
				}
				break;
			}  
		} 
	    row=row+1;
   	} 
   }
}

std::vector<std::vector<double>> MaterialInterpolator::mat_sigma(){
    return material_sigma_;
}

std::vector<std::vector<double>> MaterialInterpolator::sigma_with_deg_dis(){
	std::vector<double> degs = interpolated_deg();
	std::vector<std::vector<double>> toreturn;
	toreturn.push_back({});
	toreturn[0].insert(toreturn[0].end(),degs.begin(),degs.end());
	toreturn.insert(toreturn.end(),material_sigma_.begin(),material_sigma_.end());
	std::vector<double> distance = interpolated_distance();
	for(int i=0;i<distance.size();i++){
		toreturn[i+1].insert(toreturn[i+1].begin(),distance[i]);
	}
    return toreturn;
}

void MaterialInterpolator::InterpolateRefmat(){
	float critical_val=17.9/pow(2.158,2);
	critical_val=21.3/pow(9.92,2);
	int dimy=maxrange/step_width_interp_dist_;
	int dimx=90/step_width_interpolate_deg_;
	for(int rows=0;rows<dimy+1;rows++){
	    material_sigma_.push_back({});
	}
	int degindex=0; 
	int lastvalue=distance_ref_1_.size()-1;
	for(int deg=0;deg<dimx;deg++){
		int row=0;
		//find the degreeindex for interpolation
		for(degindex=1;degindex<deg_ref_1_.size();degindex++){
			if(step_width_interpolate_deg_*deg<=deg_ref_1_[degindex]){			
					break;
			}
		}
		if(degindex==deg_ref_1_.size()){
			degindex--;
		}
		for(double y=0;y<=maxrange+0.01;y=y+step_width_interp_dist_){
			bool foundlarger=false;
			if(withcrit_){
				float othervalue=material_intensity_inter_.at(deg)/pow(y,2);
				if(othervalue/critical_val<1){
					material_sigma_.at(row).push_back(-1);
					row=row+1;
					continue;
				}
			}
			for(int x=0;x<distance_ref_1_.size();x++){	
				if(distance_ref_1_[x]>y){
					int distindx=x;
			
					if(distindx == 0){distindx=1;}
					float value =-1;
					if(ref_material_1_[distindx-1][degindex-1] == -1 ||ref_material_1_[distindx][degindex-1] == -1 || ref_material_1_[distindx-1][degindex] == -1 || ref_material_1_[distindx][degindex] == -1){}
					else{					
						value = interpolatewithangle(ref_material_1_[distindx-1][degindex-1],ref_material_1_[distindx][degindex-1],ref_material_1_[distindx-1][degindex],ref_material_1_[distindx][degindex],distance_ref_1_[distindx-1],distance_ref_1_[distindx],deg_ref_1_[degindex-1],deg_ref_1_[degindex],step_width_interpolate_deg_*deg,y);
						if(value < 0){
							value=0;
						}
					}
					material_sigma_.at(row).push_back(value);
					foundlarger=true;
					break;
				}
			}
			if(foundlarger==false){
					float value =-1;
					if(ref_material_1_[lastvalue-1][degindex-1] == -1 ||ref_material_1_[lastvalue][degindex-1] == -1 || ref_material_1_[lastvalue-1][degindex] == -1 || ref_material_1_[lastvalue][degindex] == -1){}
					else{
					value = interpolatewithangle(ref_material_1_[lastvalue-1][degindex-1],ref_material_1_[lastvalue][degindex-1],ref_material_1_[lastvalue-1][degindex],ref_material_1_[lastvalue][degindex],distance_ref_1_[lastvalue-1],distance_ref_1_[lastvalue],deg_ref_1_[degindex-1],deg_ref_1_[degindex],step_width_interpolate_deg_*deg,y);}
					material_sigma_.at(row).push_back(value);
					if(deg == 0){std::cout<<y<<" "<<value<<" "<<distance_ref_1_[lastvalue]<<" "<<ref_material_1_[lastvalue-1][degindex-1]<<" "<<ref_material_1_[lastvalue][degindex-1]<<" "<<ref_material_1_[lastvalue-1][degindex]<<" "<<std::endl;
					}
			}
	    		row=row+1;}
	}
}

void MaterialInterpolator::InterpolateIntensities(std::vector<double>& material_intensity_data,std::vector<double> deg)
{
    material_intensity_inter_.clear();
    int size=deg.size();

    for(double degree=0;degree<90;degree=degree+step_width_interpolate_deg_){ 
        bool found = false;
        for(int m=0;m<size;m++){     
            if(degree <= deg.at(m) && m == 0){
                material_intensity_inter_.push_back(interpolate(material_intensity_data.at(m), deg.at(m), material_intensity_data.at(m+1), deg.at(m+1), degree));
                found=true;
                break;
            }        
            if(degree <= deg.at(m) && m > 0){
                material_intensity_inter_.push_back(interpolate(material_intensity_data.at(m), deg.at(m), material_intensity_data.at(m-1), deg.at(m-1), degree));
                found=true;
                break;
            }
        }
        if(found == false){
                material_intensity_inter_.push_back(interpolate(material_intensity_data.at(size-1), deg.at(size-1), material_intensity_data.at(size-2), deg.at(size-2), degree));
        }
    }

}

void MaterialInterpolator::PrintInterpolateIntensities(){
    std::cout<<"Interpolated Intensities"<<std::endl;
    for(int i=0;i<material_intensity_.size();i++){
        std::cout<<material_intensity_.at(i)<<" ";
    }
    std::cout << std::endl;
}

std::vector<double> MaterialInterpolator::interpolated_deg(){
	std::vector<double> toreturn;
	for(int degree=0;degree<maxdeg;degree=degree+step_width_interpolate_deg_){ 
		toreturn.push_back(degree);
	}
	return toreturn;
}
std::vector<double> MaterialInterpolator::interpolated_distance(){
	std::vector<double> toreturn;
	for(double y=0;y<maxrange+0.01;y=y+step_width_interp_dist_){ 
		toreturn.push_back(y);
	}	
	return toreturn;
}
//only usable with 3rd Constructor!
void MaterialInterpolator::Printrefmat1(){
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
void MaterialInterpolator::PrintpredictedMat(){
	std::cout<<"Predicted material values"<<std::endl;
	//print angles
	for(double i=0;i<maxdeg;i=i+step_width_interpolate_deg_){
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