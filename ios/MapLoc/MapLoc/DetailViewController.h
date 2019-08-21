#include "common_header.h"
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <memory>
#include <vector>
#include "delegate_header.h"

@interface DetailViewController : UIViewController<FrameInfoDelegate>{
    dispatch_queue_t sessionQueue;
}
@property (weak, nonatomic) IBOutlet UIImageView *image_view;
@property (nonatomic, weak) IBOutlet UILabel *reproj_err_label;
@property (nonatomic, weak) IBOutlet UILabel *match_count_label;
@property (nonatomic, weak) IBOutlet UILabel *expo_time_label;
@property (nonatomic, weak) IBOutlet UILabel *iso_label;
@property (nonatomic, weak) IBOutlet UILabel *offset_label;
@property (weak, nonatomic) IBOutlet UILabel *gps_accu_l;

@end
