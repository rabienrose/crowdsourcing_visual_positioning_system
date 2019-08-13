#include<opencv2/core/core.hpp>
#import "DetailViewController.h"
#import <AVFoundation/AVFoundation.h>
#import <SceneKit/SceneKit.h>
#import <UIKit/UIKit.h>

@interface ViewController : UIViewController<SCNSceneRendererDelegate, SceneInfoDelegate>{
    SCNNode *meNode;
    SCNNode *cameraNode;
    SCNNode *worldNode;
    SCNNode *pcNode;
    
    dispatch_queue_t sessionQueue;
    
    float cam_distence;
    float cam_pitch;
    float cam_yaw;
    float cam_roll;
    float temp_cam_pitch;
    float temp_cam_yaw;
    float temp_cam_roll;
    float temp_cam_tar_x;
    float temp_cam_tar_y;
    float cam_tar_x;
    float cam_tar_y;
    float temp_scale;
    float cam_scale;
    DetailViewController *detail_view;
}


@end

