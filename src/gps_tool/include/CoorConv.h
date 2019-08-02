#ifndef __COORCONV_H__
#define __COORCONV_H__

#include <cmath>
#include <Eigen/Core>


typedef struct tagUTMCorr 
{
	double x;
	double y;
}UTMCoor;

typedef struct tagWGS84Corr
{
	double lat;
	double log;
}WGS84Corr;

void convert_to_coor(Eigen::Vector3d ori_gps, Eigen::Vector3d& coor_gps, Eigen::Vector3d anchor_gps);

class GpsConverter
{
public:
    GpsConverter();

    GpsConverter(double origin_lat, double origin_lon, int zone, bool southhemi);

	GpsConverter(double origin_lat, double origin_lon, bool southhemi);

    void MapLatLonToXY (double lat, double lon, UTMCoor &xy);

    void MapXYToLatLon (double x, double y, WGS84Corr &latlon);

	void LatLonToUTMXY (double lat, double lon, int zone, UTMCoor &xy);

	void LatLonToUTMXY (double lat, double lon, UTMCoor &xy);

	void UTMXYToLatLon (double x, double y, int zone, bool southhemi, WGS84Corr &latlon);

private:
    double DegToRad (double deg);

    double RadToDeg (double rad);

    double ArcLengthOfMeridian (double phi);

    double UTMCentralMeridian (int zone);

    double FootpointLatitude (double y);

    void MapLatLonToXY (double phi, double lambda, double lambda0, UTMCoor &xy);

    void MapXYToLatLon (double x, double y, double lambda0, WGS84Corr &philambda);

    int LatLonToZone(double lat, double lon);



private:
    tagWGS84Corr origin_wgs84_;
    tagUTMCorr origin_utm_;
    int default_zone_;
    bool is_southhemi_;
    bool global_shift_;

};


#endif //__COORCONV_H__