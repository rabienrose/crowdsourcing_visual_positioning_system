#include"global_map/global_map.h"
#include"s1angle.h"
#include"s2latlng.h"
#include"s2cell_id.h"
#include<iomanip>

void get_map_block_id_from_gps(unsigned int& block_id, Eigen::Vector3d gps_latlon){

    double lat_degrees,lng_degrees;
    lat_degrees = gps_latlon(0);
    lng_degrees = gps_latlon(1);
    std::cout << "lat: " << lat_degrees << std::endl;
    std::cout << "lng: " << lng_degrees << std::endl;

    S2CellId id(S2LatLng::FromDegrees(lat_degrees, lng_degrees));
    block_id = id.parent(12).id() >> 32;
    std::cout << "$id: " << id.parent(12).id() << std::endl;
}

void get_gps_from_block_id(Eigen::Vector3d& gps_latlon, unsigned int block_id){

    std::cout << "block_id: " << block_id <<std::endl;
    S2::uint64 id =3869277694130651136; //block_id;
    //id = id << 32;
    std::cout << "$block_id: " << id <<std::endl;
    S2CellId cellId(id);
    std::cout << "level: " << cellId.level() <<std::endl;
    std::cout << "after face: " << cellId.face() << std::endl;
    std::cout << "id64: " << cellId.id() <<std::endl;

    R2Point point;
    S2LatLng latlng;
    latlng = cellId.ToLatLng();
    std::cout << "Lat value: " << latlng.coords().x() << std::endl;
    std::cout << "Lng value: " << latlng.coords().y() << std::endl;

    //lng=gps_latlon(0)   lat=gps_latlon(1)
    gps_latlon(0) = latlng.lat().degrees();
    gps_latlon(1) = latlng.lng().degrees();

    gps_latlon(2) = 0;
    std::cout << "GPS latlon: \n" << gps_latlon << std::endl;
}

int main(){

	Eigen::Vector3d gps_latlon = {31.232135, 121.41321700000003, 0};
    //uint64 id=3869277694130651136;
    unsigned int id = 3869277694130651136;

	//get_map_block_id_from_gps(id, gps_latlon);

	get_gps_from_block_id(gps_latlon, id);

	return 0;
}
