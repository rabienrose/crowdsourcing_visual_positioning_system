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
- (IBAction)slider_z_action:(id)sender{
    [self update_scene];
    
}

- (void) handleTap:(UIGestureRecognizer*)gestureRecognize{
    //std::cout<<"tap"<<std::endl;
}

- (void) handlePan:(UIGestureRecognizer*)gestureRecognize{
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
- (IBAction)exit_btn:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}

- (id)initWithCoder:(NSCoder *)decoder
{
    self = [super initWithCoder:decoder];
    pix_per_meter=1;
    scene_w=900;
    scene_h=600;
    return self;
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
    [self update_scene];
}
- (void) set_cur_posi: (Eigen::Vector3d) cur_posis matches:(std::vector<Eigen::Vector3d>) cur_matches update_center:(bool)update_center{
    traj.push_back(cur_posis);
    if(update_center){
        center_posi =cur_posis;
    }
    matches=cur_matches;
    [self update_scene];
}
    
- (void) showPC: (std::vector<Eigen::Vector3d>&) mp_posis kf:(std::vector<Eigen::Vector3d>&) kf_posis{
    mps=mp_posis;
    kfs = kf_posis;
    float min=99999;
    float max=-99999;
    for(int i=0; i<kf_posis.size(); i++){
        if(kf_posis[i](0)<min){
            min=kf_posis[i](0);
        }
        if(kf_posis[i](0)>max){
            max=kf_posis[i](0);
        }
    }
    pix_per_meter=scene_w/((max-min)*1.5);
    [self update_scene];
}
- (bool) checkPtVisible:(Eigen::Vector3d)pt pix_pt:(Eigen::Vector3d&)pix_posi{
    pix_posi = (pt-center_posi)*pix_per_meter;
    if(fabs(pix_posi(1))<scene_h/2 && fabs(pix_posi(0))<scene_w/2){
        return true;
    }else{
        return false;
    }
}

-(void) show_pcs:(std::vector<Eigen::Vector3d>&)pcs img:(cv::Mat&)img color:(cv::Scalar)color {
    for(int i=0; i<pcs.size(); i++){
        Eigen::Vector3d pix_posi;
        if([self checkPtVisible:pcs[i] pix_pt:pix_posi]){
            for(int n=-1; n<=1; n++){
                for(int m=-1; m<=1; m++){
                    int row=-pix_posi(1)+scene_h/2+n;
                    int col=pix_posi(0)+scene_w/2+m;
                    if(col<0 || col>=img.cols || row<0 || row>=img.rows){
                        continue;
                    }
                    img.at<cv::Scalar_<unsigned char>>(row, col )=color;
                }
            }
        }
    }
}

- (void) update_scene{
    UIImage *ui_image;
    cv::Mat img=cv::Mat(scene_h,scene_w,CV_8UC4,cv::Scalar(255,255,255,255));
    if(self.mp_s.on){
        for(int i=0; i<mps.size(); i++){
            if(mps[i](2)>self.z_min_slider.value && mps[i](2)<self.z_max_slider.value){
                Eigen::Vector3d pix_posi;
                if([self checkPtVisible:mps[i] pix_pt:pix_posi]){
                    img.at<cv::Scalar_<unsigned char>>(-pix_posi(1)+scene_h/2, pix_posi(0)+scene_w/2)=cv::Scalar(0,0,0,255);
                }
            }
        }
    }
    if(self.kf_s.on){
        [self show_pcs:kfs img:img color:cv::Scalar(255,0,0,255)];
    }
    if(self.traj_s.on){
        [self show_pcs:traj img:img color:cv::Scalar(0,0,255,255)];
    }
    if(self.match_s.on){
        Eigen::Vector3d pix_posi1;
        if([self checkPtVisible:traj.back() pix_pt:pix_posi1]){
            for(int i=0; i<matches.size(); i++){
                Eigen::Vector3d pix_posi2;
                if([self checkPtVisible:matches[i] pix_pt:pix_posi2]){
                    cv::Point2f pt1( pix_posi1(0)+scene_w/2,-pix_posi1(1)+scene_h/2);
                    cv::Point2f pt2( pix_posi2(0)+scene_w/2,-pix_posi2(1)+scene_h/2);
                    cv::line(img, pt1, pt2,  cv::Scalar(0,0,255,255), 1);
                }
            }
        }
        
    }
    
    ui_image = [mm_Try UIImageFromCVMat:img];
    dispatch_async( dispatch_get_main_queue(), ^{
        self.image_view.image=ui_image;
    });
}

- (IBAction)clear_all:(id)sender {
    traj.clear();
    matches.clear();
    [self update_scene];
}
- (IBAction)mp_switch:(id)sender {
    [self update_scene];
}
- (IBAction)kf_switch:(id)sender {
    [self update_scene];
}
- (IBAction)traj_switch:(id)sender {
    [self update_scene];
}
- (IBAction)match_switch:(id)sender {
    [self update_scene];
}


@end
