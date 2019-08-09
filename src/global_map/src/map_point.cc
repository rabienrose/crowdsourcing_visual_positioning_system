#include "global_map/map_point.h"
#include <glog/logging.h>
#include "global_map/frame.h"
namespace gm {

int DescriptorDistance(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& a, Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& b){
        const int *pa = (int*)a.data();
        const int *pb = (int*)b.data();

        int dist=0;
        int count=a.rows()/4;
        for(int i=0; i<count; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

void TrackItem::getUV(float& x, float& y, int& octave)
{
    if ((size_t)kp_ind >= frame->kps.size()) {
        std::cout << "[TrackItem::getUV][error]kp_ind>=frame->kps.size()" << std::endl;
        exit(0);
    }
    x = frame->kps[kp_ind].pt.x;
    y = frame->kps[kp_ind].pt.y;
    octave = frame->kps[kp_ind].octave;
}
    
    void MapPoint::getALLDesc(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descs){
        if(track.size()==0){
            return;
        }
        int desc_width=track[0].frame->descriptors.rows();
        descs.resize(desc_width, track.size());
        for(int i=0; i<track.size(); i++){
            std::shared_ptr< Frame> frame=track[i].frame;
            int kp_ind=track[i].kp_ind;
            descs.block(0,i, desc_width,1)=frame->descriptors.block(0,kp_ind, desc_width,1);
        }
    }
    
    int MapPoint::calDescDiff(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& query_desc){
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> target_descs;
        int min_diff=100000;
        int desc_width=query_desc.rows();
        getALLDesc(target_descs);
        for(int i=0; i<target_descs.cols(); i++){
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> tar_desc = target_descs.block(0,i,desc_width, 1);
            int diff = DescriptorDistance(tar_desc, query_desc);
            if(diff<min_diff){
                min_diff=diff;
            }
        }
        return min_diff;
    }
    
    MapPoint::MapPoint(){
        isfix=false;
        isbad=false;
    }

}  // namespace gm
