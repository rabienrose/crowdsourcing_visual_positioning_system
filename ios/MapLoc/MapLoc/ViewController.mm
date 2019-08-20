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
- (void) set_cur_posi: (Eigen::Vector3d) cur_posis matches:(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>) cur_matches{
    center_posi =cur_posis;
    matches=cur_matches;
}
    
- (void) showPC: (std::vector<Eigen::Vector3d>&) mp_posis kf:(std::vector<Eigen::Vector3d>&) kf_posis{
    mps=mp_posis;
    kfs = kf_posis;
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
        for(int i=0; i<kfs.size(); i++){
            Eigen::Vector3d pix_posi;
            if([self checkPtVisible:kfs[i] pix_pt:pix_posi]){
                for(int n=-1; n<=1; n++){
                    for(int m=-1; m<=1; m++){
                        int row=-pix_posi(1)+scene_h/2+n;
                        int col=pix_posi(0)+scene_w/2+m;
                        if(col<0 || col>=img.cols || row<0 || row>=img.rows){
                            continue;
                        }
                        
                        img.at<cv::Scalar_<unsigned char>>(row, col )=cv::Scalar(0,255,0,255);
                    }
                }
                
                //cv::circle(img, cv::Point2f(pix_posi(0)+scene_w/2, -pix_posi(1)+scene_h/2), 2,  cv::Scalar(0,255,0,255), 3);
            }
        }
    }
    if(self.traj_s.on){
        for(int i=0; i<traj.size(); i++){
            Eigen::Vector3d pix_posi;
            if([self checkPtVisible:traj[i] pix_pt:pix_posi]){
                cv::circle(img, cv::Point2f(pix_posi(0)+scene_w/2, -pix_posi(1)+scene_h/2), 2,  cv::Scalar(0,255,0,255), 3);
            }
        }
    }
    if(self.match_s.on){
        for(int i=0; i<matches.size(); i++){
            Eigen::Vector3d pix_posi1;
            if([self checkPtVisible:matches[i].first pix_pt:pix_posi1]){
                Eigen::Vector3d pix_posi2;
                if([self checkPtVisible:matches[i].second pix_pt:pix_posi2]){
                    cv::line(img, cv::Point2f(pix_posi1(0), pix_posi1(1)), cv::Point2f(pix_posi2(0), pix_posi2(1)),  cv::Scalar(0,0,255,255), 1);
                }
            }
        }
    }
    
    ui_image = [mm_Try UIImageFromCVMat:img];
    dispatch_async( dispatch_get_main_queue(), ^{
        self.image_view.image=ui_image;
    });
}

- (IBAction)go_map_list:(id)sender {
}
- (IBAction)clear_all:(id)sender {
    
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
