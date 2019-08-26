#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include "CoorConv.h"
#include <Eigen/Core>
#include <gflags/gflags.h>
#include "global_map/global_map.h"

DEFINE_string(raw_rtk_addr, "", "rtk's address");
DEFINE_double(gps_anchor_x, -1, "");
DEFINE_double(gps_anchor_y, -1, "");
DEFINE_double(gps_anchor_z, -1, "");
std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}



double pi = 3.1415926535897932384626;
double a = 6378245.0;
double ee = 0.00669342162296594323;


bool outOfChina(double lat, double lon) {
    if (lon < 72.004 || lon > 137.8347) return true;
    if (lat < 0.8293 || lat > 55.8271) return true;
    return false;
}

double transformLat(double x, double y) {
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
    + 0.2 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}
double transformLon(double x, double y) {
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1* sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}

void gps84_To_Gcj02(double& lat, double& lon) {
    if (outOfChina(lat, lon)) {
        return;
    }
    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * pi;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
    lat = lat + dLat;
    lon = lon + dLon;
}


void getDecDegree(std::string raw, double& degree){
    std::vector<std::string> splited = split(raw, ":");
    std::string sec=splited[2];
    sec.erase(sec.end()-1, sec.end());
    //std::cout<<sec<<std::endl;
    degree = atof(splited[0].c_str())+atof(splited[1].c_str())/60+atof(sec.c_str())/3600;
}


int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    Eigen::Vector3d anchorGps;
    std::ofstream outfile_gps_orth;
    outfile_gps_orth.open (FLAGS_raw_rtk_addr+".oth.txt");
    std::ofstream outfile_gps_raw;
    outfile_gps_raw.open (FLAGS_raw_rtk_addr+".raw.txt");
    unsigned int block_id=112260160;
    Eigen::Vector3d gps_latlon;
    gm::get_gps_from_block_id(gps_latlon, block_id);
    //anchorGps<<FLAGS_gps_anchor_x, FLAGS_gps_anchor_y, FLAGS_gps_anchor_z; 
    anchorGps=gps_latlon;
    std::ifstream infile(FLAGS_raw_rtk_addr.c_str());
    std::string line;
    while (true)
    {
        std::getline(infile, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        std::string lat=splited[1];
        std::string lon=splited[2];
        double lat_d;
        double lon_d;
        getDecDegree(lat, lat_d);
        getDecDegree(lon, lon_d);
        gps84_To_Gcj02(lat_d, lon_d);
        Eigen::Vector3d ori_gps;
        ori_gps(0)=lat_d;
        ori_gps(1)=lon_d;
        ori_gps(2)=atof(splited[3].c_str());
        Eigen::Vector3d coor_gps;
        convert_to_coor(ori_gps, coor_gps, anchorGps);
        outfile_gps_orth<<coor_gps(0)<<","<<coor_gps(1)<<","<<coor_gps(2)<<",0.01"<<std::endl;
        outfile_gps_raw<<ori_gps(0)<<","<<ori_gps(1)<<","<<ori_gps(2)<<std::endl;
    }
    outfile_gps_orth.close();
    outfile_gps_raw.close();

    return 0;
}
