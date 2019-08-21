#import "DetailViewController.h"

@implementation DetailViewController

- (void)viewDidLoad
{
	[super viewDidLoad];
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    
}

- (IBAction)exit_btn:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}

- (void)showFrame: (cv::Mat) img{
    UIImage *ui_image;
    ui_image = [mm_Try UIImageFromCVMat:img];
    dispatch_async( dispatch_get_main_queue(), ^{
        self.image_view.image=ui_image;
    });
}

- (void) showCurInfo: (float) repro_err match_count: (int) match_count expo_time: (float )expo_time iso:(float )iso offset:(float )offset gps_accu:(int) gps_accu{
    dispatch_async( dispatch_get_main_queue(), ^{
        if(repro_err>0){
            self.reproj_err_label.text = [NSString stringWithFormat: @"err: %f", repro_err];
        }
        if(match_count>0){
            self.match_count_label.text = [NSString stringWithFormat: @"match: %d", match_count];
        }
        if(expo_time>0){
            self.expo_time_label.text = [NSString stringWithFormat: @"expo: %f", expo_time];
        }
        if(iso>0){
            self.iso_label.text = [NSString stringWithFormat: @"iso: %f", iso];
            self.offset_label.text = [NSString stringWithFormat: @"offset: %f", offset];
        }
        if(gps_accu>0){
            self.gps_accu_l.text = [NSString stringWithFormat: @"gps: %d", gps_accu];
        }
    });
}


@end
