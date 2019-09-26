#include "separate_map.h"

namespace gm{
	void get_map_block_id_from_gps(unsigned int& block_id, Eigen::Vector3d gps_latlon){

		block_id=floor(gps_latlon(0)*100)*360*100+floor(gps_latlon(1)*100);
	}

    void get_gps_from_block_id(Eigen::Vector3d& gps_latlon, unsigned int block_id){
        gps_latlon(0)=block_id/(360*100);
        gps_latlon(0)=gps_latlon(0)/(double)100;
        gps_latlon(1)=(block_id-gps_latlon(0)*(360*100*100))/(double)100;
    }

	std::vector<unsigned int> getNearBlock(std::vector<unsigned int>& input_ids){
		double min_lon=9999;
		double max_lon=-9999;
		double min_lat=9999;
		double max_lat=-9999;
		for(int i=0; i<input_ids.size(); i++){
			Eigen::Vector3d gps_latlon;
			get_gps_from_block_id(gps_latlon, input_ids[i]);
			//std::cout<<std::setprecision(15)<<gps_latlon(0)<<" : "<<gps_latlon(1)<<std::endl;
			if(gps_latlon(0)>max_lon){
				max_lon=gps_latlon(0);
			}
			if(gps_latlon(0)<min_lon){
				min_lon=gps_latlon(0);
			}
			if(gps_latlon(1)>max_lat){
				max_lat=gps_latlon(1);
			}
			if(gps_latlon(1)<min_lat){
				min_lat=gps_latlon(1);
			}
		}
		//std::cout<<std::setprecision(15)<<min_lon<<" : "<<max_lon<<std::endl;
		//std::cout<<std::setprecision(15)<<min_lat<<" : "<<max_lat<<std::endl;

		min_lon=floor(min_lon*100)/100;
		max_lon=floor(max_lon*100)/100;
		min_lat=floor(min_lat*100)/100;
		max_lat=floor(max_lat*100)/100;
		std::vector<unsigned int> out_ids;
		for(double lon=min_lon-0.01; lon<=max_lon+0.01; lon=lon+0.01){
			for(double lat=min_lat-0.01; lat<=max_lat+0.01; lat=lat+0.01){
				unsigned int block_id;
				Eigen::Vector3d gps_latlon;
				gps_latlon(0)=lon;
				gps_latlon(1)=lat;
				get_map_block_id_from_gps(block_id, gps_latlon);
				out_ids.push_back(block_id);
			}
		}
		return out_ids;
	}

}
