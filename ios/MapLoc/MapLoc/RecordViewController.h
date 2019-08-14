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
@protocol RecordViewControllerDelegate <NSObject>
@required
- (cv::Mat)getNewFrame;
@end

@interface RecordViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate>{
    CMMotionManager *motionManager;
    dispatch_queue_t sessionQueue;
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
}
@property (nonatomic, weak) id<FrameInfoDelegate> frameDelegate;
@property (nonatomic, weak) id<SceneInfoDelegate> sceneDelegate;
@property (weak, nonatomic) IBOutlet UIPickerView *map_list_ui;
@end
