#include<opencv2/core/core.hpp>
#import "DetailViewController.h"
#import <AVFoundation/AVFoundation.h>
#import <UIKit/UIKit.h>
#include "Eigen/Core"

@interface ViewController : UIViewController<SceneInfoDelegate>{
    dispatch_queue_t sessionQueue;
    DetailViewController *detail_view;
    Eigen::Vector2f cam_center;
    float pix_per_meter;
    Eigen::Vector3d center_posi;
    std::vector<Eigen::Vector3d> mps;
    std::vector<Eigen::Vector3d> traj;
    int scene_w;
    int scene_h;
    Eigen::Vector3d temp_pan_posi;
    float temp_pin_scale;
}
@property (weak, nonatomic) IBOutlet UISlider *z_min_slider;
@property (weak, nonatomic) IBOutlet UISlider *z_max_slider;
@property (weak, nonatomic) IBOutlet UIImageView *image_view;
@end

