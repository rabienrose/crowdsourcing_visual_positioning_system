//
//  ViewController.m
//  MapLoc
//
//  Created by zili wang on 2019/4/5.
//  Copyright © 2019 zili wang. All rights reserved.
//

#import "ViewController.h"

@interface ViewController ()

@end
#define PI 3.1415
@implementation ViewController
- (IBAction)slider_z_min_action:(id)sender{
    std::cout<<_z_min_slider.value<<std::endl;
    
}
- (IBAction)slider_z_max_action:(id)sender{
    
}
- (void) handleTap:(UIGestureRecognizer*)gestureRecognize{
    //std::cout<<"tap"<<std::endl;
}

- (void) handlePan:(UIGestureRecognizer*)gestureRecognize{
    std::cout<<"pan"<<std::endl;
    UIPanGestureRecognizer* reco = (UIPanGestureRecognizer*)gestureRecognize;
    CGPoint p = [reco translationInView:_image_view];
    if(reco.numberOfTouches==2){
    }else if(reco.numberOfTouches==1){
        if(reco.state==1){
            temp_pan_posi=center_posi;
        }else if(reco.state==2){
            center_posi=temp_pan_posi+Eigen::Vector3d(-p.x/pix_per_meter*1.5, p.y/pix_per_meter*1.5, 0);
        }
    }
    [self update_scene];

}

- (void) handleRot:(UIGestureRecognizer*)gestureRecognize{
    //std::cout<<"rot"<<std::endl;
}

- (void) handlePin:(UIGestureRecognizer*)gestureRecognize{
    UIPinchGestureRecognizer* reco = (UIPinchGestureRecognizer*)gestureRecognize;
    if(reco.state==1){
        temp_pin_scale=pix_per_meter;
    }else if(reco.state==2){
        pix_per_meter=temp_pin_scale*reco.scale;
    }
    [self update_scene];
}

- (IBAction)from_scene_to_view_btn:(id)sender {
    [self presentViewController:detail_view animated:NO completion:nil];
}

- (void)viewDidLoad {
    [super viewDidLoad];
    
    UIPanGestureRecognizer *panGesture = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handlePan:)];
    UIRotationGestureRecognizer *rotGesture = [[UIRotationGestureRecognizer alloc] initWithTarget:self action:@selector(handleRot:)];
    UIPinchGestureRecognizer *pinGesture = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(handlePin:)];
    UITapGestureRecognizer *tapGesture = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleTap:)];
    NSMutableArray *gestureRecognizers = [NSMutableArray array];
    [gestureRecognizers addObject:panGesture];
    [gestureRecognizers addObject:rotGesture];
    [gestureRecognizers addObject:pinGesture];
    [gestureRecognizers addObject:tapGesture];
    _image_view.gestureRecognizers = gestureRecognizers;
    _image_view.userInteractionEnabled = YES;
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    UIStoryboard* storyboard = [UIStoryboard storyboardWithName:@"Main" bundle:nil];
    detail_view = [storyboard instantiateViewControllerWithIdentifier:@"DetailViewController"];
    detail_view.sceneDelegate=self;
    pix_per_meter=1;
    scene_w=900;
    scene_h=600;
}
- (void) set_cur_posi: (Eigen::Vector3d) posis{
    center_posi =posis;
}
    
- (void) showPC: (std::vector<Eigen::Vector3d>&) posis{
    mps=posis;
    [self update_scene];
}

- (void) update_scene{
    UIImage *ui_image;
    cv::Mat img=cv::Mat(scene_h,scene_w,CV_8UC4,cv::Scalar(0,0,0,255));
    for(int i=0; i<mps.size(); i++){
        Eigen::Vector3d pix_posi;
        pix_posi = (mps[i]-center_posi)*pix_per_meter;
        if(fabs(pix_posi(1))<scene_h/2 && fabs(pix_posi(0))<scene_w/2){
            img.at<cv::Scalar_<unsigned char>>(-pix_posi(1)+scene_h/2, pix_posi(0)+scene_w/2)=cv::Scalar(255,255,255,255);
            //cv::circle(img, cv::Point2f(pix_posi(1)+scene_w/2, pix_posi(0)+scene_h/2), 1, cv::Scalar(255,255,255,255));
        }
    }
    ui_image = [mm_Try UIImageFromCVMat:img];
    dispatch_async( dispatch_get_main_queue(), ^{
        self.image_view.image=ui_image;
    });
}

@end
