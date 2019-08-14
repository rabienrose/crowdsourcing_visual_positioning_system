#include<opencv2/core/core.hpp>
#import "DetailViewController.h"
#import <AVFoundation/AVFoundation.h>
#import <UIKit/UIKit.h>

@interface ViewController : UIViewController{
    dispatch_queue_t sessionQueue;
    DetailViewController *detail_view;
}


@end

