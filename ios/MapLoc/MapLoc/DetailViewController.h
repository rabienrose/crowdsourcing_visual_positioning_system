#include "common_header.h"
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <memory>
#include <vector>
#include "RecordViewController.h"
#include "delegate_header.h"

@interface DetailViewController : UIViewController<FrameInfoDelegate>{
    dispatch_queue_t sessionQueue;
    RecordViewController *record_view;
}
@property (nonatomic, weak) id<SceneInfoDelegate> sceneDelegate;
@property (weak, nonatomic) IBOutlet UIImageView *image_view;
@property (nonatomic, weak) IBOutlet UILabel *reproj_err_label;
@property (nonatomic, weak) IBOutlet UILabel *match_count_label;
@property (nonatomic, weak) IBOutlet UILabel *total_mp_label;
@property (nonatomic, weak) IBOutlet UILabel *total_kf_label;
@end
