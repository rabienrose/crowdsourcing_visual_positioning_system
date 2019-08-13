#import "DetailViewController.h"

@implementation DetailViewController

- (void)viewDidLoad
{
	[super viewDidLoad];
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    UIStoryboard* storyboard = [UIStoryboard storyboardWithName:@"Main" bundle:nil];
    record_view = [storyboard instantiateViewControllerWithIdentifier:@"RecordViewController"];
    record_view.frameDelegate = self;
    record_view.sceneDelegate=self.sceneDelegate;
}

- (IBAction)exit_btn:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}

- (IBAction)record_btn:(id)sender {
   [self presentViewController:record_view animated:NO completion:nil];
}

- (void)showFrame: (cv::Mat) img{
    UIImage *ui_image;
    ui_image = [mm_Try UIImageFromCVMat:img];
    dispatch_async( dispatch_get_main_queue(), ^{
        self.image_view.image=ui_image;
    });
}

- (void) showCurInfo: (float) repro_err match_count: (int) match_count mp_count: (int)mp_count kf_count:(int)kf_count{
    dispatch_async( dispatch_get_main_queue(), ^{
        self.reproj_err_label.text = [NSString stringWithFormat: @"err: %f", repro_err];
        self.match_count_label.text = [NSString stringWithFormat: @"match: %d", match_count];
        self.total_mp_label.text = [NSString stringWithFormat: @"mp: %d", mp_count];
        self.total_kf_label.text = [NSString stringWithFormat: @"kf: %d", kf_count];
    });
}


@end
