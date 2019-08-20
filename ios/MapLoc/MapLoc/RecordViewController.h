#include "common_header.h"
#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <memory>
#include <vector>
#import <ros/ros.h>
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include "delegate_header.h"
#import "UIListDelegate.h"
#include <global_map_api/global_map_api.h>
@protocol RecordViewControllerDelegate <NSObject>
@required
- (cv::Mat)getNewFrame;
@end

@interface RecordViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate>{
    CMMotionManager *motionManager;
    dispatch_queue_t sessionQueue;
    dispatch_queue_t updateQueue;
    NSOperationQueue *quene;
    AVCaptureSession *session;
    AVCaptureDeviceInput *videoDeviceInput;
    AVCaptureDeviceDiscoverySession *videoDeviceDiscoverySession;
    AVCaptureDevice *videoDevice;
    AVCaptureVideoDataOutput *video_output;
    std::vector<std::vector<double> > gyros;
    std::vector<std::vector<double> > acces;
    bool is_recording_bag;
    std::shared_ptr<rosbag::Bag> bag_ptr;
    int img_count;
    cv::Mat img_display;
    bool hasNewFrame;
    int last_kf_count;
    BagListDelegate *dele_map;
    gm::GlobalMapApi api;
    std::string status_mapping;
}
@property (weak, nonatomic) IBOutlet UILabel *status_label;
@property (weak, nonatomic) IBOutlet UILabel *mem_label;
@property (weak, nonatomic) IBOutlet UILabel *proc_label;
@property (weak, nonatomic) IBOutlet UILabel *bag_label;
@property (weak, nonatomic) IBOutlet UILabel *mp_label;
@property (weak, nonatomic) IBOutlet UILabel *kf_label;




@property (nonatomic, weak) id<FrameInfoDelegate> frameDelegate;
@property (nonatomic, weak) id<SceneInfoDelegate> sceneDelegate;
@property (weak, nonatomic) IBOutlet UIPickerView *map_list_ui;
@end
