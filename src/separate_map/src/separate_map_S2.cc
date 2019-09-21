#include "separate_map.h"
#include "s2cell_id.h"
#include"s1angle.h"
#include"s2latlng.h"

namespace gm{
	void get_map_block_id_from_gps(unsigned int& block_id, Eigen::Vector3d gps_latlon){

		double lat_degrees,lng_degrees;
		lat_degrees = gps_latlon(0);
		lng_degrees = gps_latlon(1);

		S2CellId id(S2LatLng::FromDegrees(lat_degrees, lng_degrees));
		block_id = id.parent(12).id() >> 32;
	}

	void get_gps_from_block_id(Eigen::Vector3d& gps_latlon, const unsigned int block_id){

		S2::uint64 id = block_id;
		id = id << 32;
		S2CellId cellId(id);

		R2Point point;
		S2LatLng latlng;
		latlng = cellId.ToLatLng();

		gps_latlon(0) = latlng.lat().degrees();
		gps_latlon(1) = latlng.lng().degrees();
		gps_latlon(2) = 0;
	}

	std::vector<unsigned int> getNearBlock(std::vector<unsigned int>& block_ids){

		std::vector<unsigned int> set_id;
		int nbr_level = 12;
		std::set<unsigned int> set_ids;
		std::vector<S2CellId> output;

		for(int i=0; i<block_ids.size(); i++){
			S2::uint64 id = block_ids[i];
			id = id << 32;
			S2CellId cellId(id);
			cellId.AppendAllNeighbors(nbr_level, &output);

			for(int j=0; j < output.size(); j++){
                unsigned int id_temp = output[j].id()>>32;
				set_ids.insert( id_temp );
			}
		}

		for(int i=0; i<block_ids.size(); i++){
			set_ids.erase( block_ids[i] );
		}

		for(std::set<unsigned int>::iterator it=set_ids.begin(); it!=set_ids.end(); ++it){
			 set_id.push_back(*it);
		}

		return set_id;
	}
}
