#include <Eigen/Dense>
@protocol FrameInfoDelegate <NSObject>
@required
- (void) showFrame: (cv::Mat) img;
- (void) showCurInfo: (float) repro_err match_count: (int) match_count mp_count: (int )mp_count kf_count:(int )kf_count;
@end

@protocol SceneInfoDelegate <NSObject>
@required
- (void) showTraj: (std::vector<Eigen::Vector3d>) posis;
@end

