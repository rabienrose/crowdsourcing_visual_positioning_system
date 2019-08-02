#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include "bag_tool/extract_bag.h"
#include <gflags/gflags.h>

DEFINE_bool(isExtractImage, false, "Whether extract images");
DEFINE_string(bag_addr, "", "bag's address");
DEFINE_string(img_topic, "img", "rostopic of image");
DEFINE_string(imu_topic, "imu", "rostopic of imu");
DEFINE_string(gps_topic, "gps", "rostopic of gps");
DEFINE_string(out_dir, "", "address of output");


int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);

    if( !FLAGS_bag_addr.empty() && !FLAGS_out_dir.empty() ){
        extract_bag(FLAGS_out_dir, FLAGS_bag_addr, FLAGS_img_topic, FLAGS_imu_topic, FLAGS_gps_topic, FLAGS_isExtractImage);

    }
    else{
        std::cout << "Please enter bag and output address in comand line." << std::endl;
    }

    return 0;
}
