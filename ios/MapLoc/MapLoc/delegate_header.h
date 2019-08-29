#include <Eigen/Dense>
@protocol FrameInfoDelegate <NSObject>
@required
- (void) showFrame: (cv::Mat) img;
- (void) showCurInfo: (float) repro_err match_count: (int) match_count expo_time: (float )expo_time iso:(float )iso offset:(float )offset gps_accu:(int) gps_accu;
@end

@protocol SceneInfoDelegate <NSObject>
@required
- (void) set_cur_posi: (Eigen::Vector3d) cur_posis matches:(std::vector<Eigen::Vector3d>) cur_matches update_center:(bool)update_center;
- (void) showPC: (std::vector<Eigen::Vector3d>&) mp_posis kf:(std::vector<Eigen::Vector3d>&) kf_posis;
- (void) setBackground: (cv::Mat) img rot:(double)rot trans_x:(double)trans_x trans_y:(double)trans_y scale:(double)scale file:(std::string)file_addr;

@end

