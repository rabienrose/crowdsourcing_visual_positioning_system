#include<opencv2/core/core.hpp>
#import "DetailViewController.h"
#import <AVFoundation/AVFoundation.h>
#import <UIKit/UIKit.h>
#include "Eigen/Core"

@interface ViewController : UIViewController<SceneInfoDelegate>{
    dispatch_queue_t sessionQueue;
    Eigen::Vector2f cam_center;
    float pix_per_meter;
    Eigen::Vector3d center_posi;
    std::vector<Eigen::Vector3d> mps;
     std::vector<Eigen::Vector3d> kfs;
    std::vector<Eigen::Vector3d> traj;
    std::vector<Eigen::Vector3d> matches;
    cv::Mat backgound_img;
    cv::Mat scene_img;
    double rot_;
    double trans_x_;
    double trans_y_;
    double scale_;
    std::string trans_file_addr;
    int scene_w;
    int scene_h;
    Eigen::Vector3d temp_pan_posi;
    float temp_pin_scale;
    float temp_rot;
}
@property (weak, nonatomic) IBOutlet UISwitch *mp_s;
@property (weak, nonatomic) IBOutlet UISwitch *kf_s;
@property (weak, nonatomic) IBOutlet UISwitch *traj_s;
@property (weak, nonatomic) IBOutlet UISwitch *match_s;
@property (weak, nonatomic) IBOutlet UISwitch *algin_s;
@property (weak, nonatomic) IBOutlet UISwitch *flip_s;


@property (weak, nonatomic) IBOutlet UISlider *z_min_slider;
@property (weak, nonatomic) IBOutlet UISlider *z_max_slider;
@property (weak, nonatomic) IBOutlet UIImageView *image_view;
@end

