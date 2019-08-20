#import "RecordViewController.h"
#include "common_header.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <chamo_common/common.h>
#import <mach/mach.h>

@implementation RecordViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    updateQueue = dispatch_queue_create( "update queue", DISPATCH_QUEUE_SERIAL );
    quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    //motionManager = [[CMMotionManager alloc] init];

    session = [[AVCaptureSession alloc] init];
    NSError *error = nil;
    [session beginConfiguration];
    session.sessionPreset =AVCaptureSessionPreset1280x720;
    videoDevice = [AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInWideAngleCamera mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified];
    AVCaptureDeviceInput *videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
    if ( ! videoDeviceInput ) {
        NSLog( @"Could not create video device input: %@", error );
        [session commitConfiguration];
        return;
    }
    if ( [session canAddInput:videoDeviceInput] ) {
        [session addInput:videoDeviceInput];
        videoDeviceInput = videoDeviceInput;
    }
    else {
        NSLog( @"Could not add video device input to the session" );
        [session commitConfiguration];
        return;
    }
    [videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 10)];
    video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    video_output.videoSettings = newSettings;
    [video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([session canAddOutput:video_output]) {
        [session addOutput:video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }

    [video_output setSampleBufferDelegate:self queue:sessionQueue];

    [session commitConfiguration];
    img_count=0;
    dele_map= [[BagListDelegate alloc] init];
    self.map_list_ui.delegate = dele_map;
    self.map_list_ui.dataSource = dele_map;
    status_mapping="done";
    dispatch_async( updateQueue, ^{
        while(true){
            struct task_basic_info info;
            mach_msg_type_number_t size = TASK_BASIC_INFO_COUNT;
            kern_return_t kerr = task_info(mach_task_self(),
                                           TASK_BASIC_INFO,
                                           (task_info_t)&info,
                                           &size);
            dispatch_async( dispatch_get_main_queue(), ^{
                self.status_label.text=[[NSString alloc] initWithUTF8String:status_mapping.c_str()];
                if( kerr == KERN_SUCCESS ) {
                    self.mem_label.text=[NSString stringWithFormat:@"%.2f MB", ((CGFloat)info.resident_size / 1048576)];
                } else {
                    self.mem_label.text=@"Err";
                }
            });
            [NSThread sleepForTimeInterval:1.0f];
        }
    });
}

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

- (void) processIMU_gyro{
    int last_acc_id=-1;
    int last_gyro_id=-1;
    int g_size=gyros.size();
    int a_size=acces.size();
    for(int i=0;i<gyros.size();i++){
        for(int j=0;j<a_size-1;j++){
            if(gyros[i][3]>acces[j][3] && gyros[i][3]<=acces[j+1][3]){
                double x,y,z;
                interDouble(acces[j][0], acces[j+1][0], acces[j][3], acces[j+1][3], x, gyros[i][3]);
                interDouble(acces[j][1], acces[j+1][1], acces[j][3], acces[j+1][3], y, gyros[i][3]);
                interDouble(acces[j][2], acces[j+1][2], acces[j][3], acces[j+1][3], z, gyros[i][3]);
                last_acc_id=j;
                last_gyro_id=i;
                sensor_msgs::Imu msg;
                msg.linear_acceleration.x=x;
                msg.linear_acceleration.y=y;
                msg.linear_acceleration.z=z;
                msg.angular_velocity.x=gyros[i][0];
                msg.angular_velocity.y=gyros[i][1];
                msg.angular_velocity.z=gyros[i][2];
                static int imu_data_seq=0;
                msg.header.seq=imu_data_seq;
                msg.header.stamp= ros::Time(gyros[i][3]);
                msg.header.frame_id="map";
                dispatch_async(sessionQueue, ^{
                    if (is_recording_bag){
                        if(bag_ptr->isOpen()){
                            NSDate * t1 = [NSDate date];
                            NSTimeInterval now = [t1 timeIntervalSince1970];
                            bag_ptr->write("imu", ros::Time(now), msg);
                        }
                    }
                });
                
                break;
            }
        }
    }
    if(last_acc_id>0){
        if(last_acc_id-1<acces.size()){
            acces.erase(acces.begin(), acces.begin()+last_acc_id);
        }else{
            NSLog(@"test overflow");
        }
    }
    if(last_gyro_id>=0){
        if(last_gyro_id<gyros.size()){
            gyros.erase(gyros.begin(), gyros.begin()+last_gyro_id+1);
        }else{
            NSLog(@"test overflow");
        }
    }
}

- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    CGImageRelease(quartzImage);
    return image;
}

- (void)update_maplist{
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    bool has_file=false;
    
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        if (count ==0){
            dele_map.sel_filename=[directoryContent objectAtIndex:count];
            has_file=true;
            break;
        }
        NSLog(@"File %d: %@", (count + 1), [directoryContent objectAtIndex:count]);
    }
    dele_map.file_list =directoryContent;
    [self.map_list_ui reloadAllComponents];
    if ((int)[directoryContent count]>0){
        [self.map_list_ui selectRow:0 inComponent:0 animated:NO];
    }
}

- (IBAction)new_btn:(id)sender {
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSDate *date = [NSDate date];
    NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
    [formatter setDateFormat:@"MM-dd-HH-mm-ss"];
    NSString *timeString = [formatter stringFromDate:date];
    NSString *string1 = [NSString stringWithFormat:@"%@",timeString];
    NSString *full_new_map_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:string1];
    BOOL isDir;
    NSFileManager *fileManager= [NSFileManager defaultManager];
    if(![fileManager fileExistsAtPath:full_new_map_addr isDirectory:&isDir])
        if(![fileManager createDirectoryAtPath:full_new_map_addr withIntermediateDirectories:YES attributes:nil error:NULL])
            NSLog(@"Error: Create folder failed %@", full_new_map_addr);
    [self update_maplist];
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
    double sync_sensor_time = (double)timestamp.value/(double)timestamp.timescale;
    double time_sec = (double)timestamp.value/(double)timestamp.timescale;
    UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
    cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
    [self.frameDelegate showFrame: img_cv];
    sensor_msgs::CompressedImage img_ros_img;
    std::vector<unsigned char> binaryBuffer_;
    cv::imencode(".jpg", img_cv, binaryBuffer_);
    img_ros_img.data=binaryBuffer_;
    img_ros_img.header.seq=img_count;
    img_ros_img.header.stamp= ros::Time(sync_sensor_time);
    img_ros_img.format="jpeg";
    
//    sensor_msgs::ImagePtr img_ros_img;
//    cv_bridge::CvImage img_cvbridge;
//    img_cvbridge.image=img_cv;
//    img_ros_img=img_cvbridge.toImageMsg();
//    img_ros_img->encoding="bgra8";
//    img_ros_img->header.seq=img_count;
//    img_ros_img->header.stamp= ros::Time(sync_sensor_time);
    dispatch_async( sessionQueue, ^{
        if(is_recording_bag){
            if(bag_ptr->isOpen()){
                NSDate * t1 = [NSDate date];
                NSTimeInterval now = [t1 timeIntervalSince1970];
                bag_ptr->write("img", ros::Time(now), img_ros_img);
            }
        }
    });
    img_count++;
}

- (IBAction)exit_btn:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}

- (void)viewWillDisappear:(BOOL)animated{
}
- (IBAction)start_record:(id)sender {
    if(!is_recording_bag){
        [self start_sensor];
        dispatch_async( sessionQueue, ^{
            NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSString *full_map_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
            full_map_addr = [full_map_addr stringByAppendingString:@"/bag"];
            BOOL isDir;
            NSFileManager *fileManager= [NSFileManager defaultManager];
            if(![fileManager fileExistsAtPath:full_map_addr isDirectory:&isDir])
                if(![fileManager createDirectoryAtPath:full_map_addr withIntermediateDirectories:YES attributes:nil error:NULL])
                    NSLog(@"Error: Create folder failed %@", full_map_addr);
            NSDate *date = [NSDate date];
            NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
            [formatter setDateFormat:@"MM-dd-HH-mm-ss"];
            NSString *timeString = [formatter stringFromDate:date];
            NSString *string1 = [NSString stringWithFormat:@"%@.bag",timeString];
            NSString *full_addr = [full_map_addr stringByAppendingPathComponent:string1];
            char *docsPath;
            docsPath = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
            std::string full_file_name(docsPath);
            std::cout<<full_file_name<<std::endl;
            bag_ptr.reset(new rosbag::Bag());
            bag_ptr->open(full_file_name.c_str(), rosbag::bagmode::Write);
            is_recording_bag=true;
        });
        //[self update_baglist];
        [sender setTitle:@"Stop" forState:UIControlStateNormal];
    }else{
        is_recording_bag=false;
        dispatch_async( sessionQueue, ^{
            bag_ptr->close();
            NSLog(@"close the bag");
            [self start_sensor];
        });
        [sender setTitle:@"Record" forState:UIControlStateNormal];
        [self update_maplist];
    }
    
}

-(void) start_sensor {
    if(session.running){
        [session stopRunning];
    }else{
        [session startRunning];
    }
    
    if(motionManager.accelerometerActive){
        [motionManager stopAccelerometerUpdates];
    }else{
        if (motionManager.accelerometerAvailable){
            motionManager.accelerometerUpdateInterval =0.01;
            [motionManager
             startAccelerometerUpdatesToQueue:quene
             withHandler:
             ^(CMAccelerometerData *data, NSError *error){
                 std::vector<double> imu;
                 imu.resize(5);
                 imu[0]=-data.acceleration.x*9.8;
                 imu[1]=-data.acceleration.y*9.8;
                 imu[2]=-data.acceleration.z*9.8;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 acces.push_back(imu);
             }];
        }
    }
    
    if(motionManager.gyroActive){
        [motionManager stopGyroUpdates];
    }else{
        if (motionManager.gyroAvailable){
            motionManager.gyroUpdateInterval =0.01;
            [motionManager
             startGyroUpdatesToQueue:quene
             withHandler:
             ^(CMGyroData *data, NSError *error){
                 std::vector<double> imu;
                 imu.resize(5);
                 imu[0]=data.rotationRate.x;
                 imu[1]=data.rotationRate.y;
                 imu[2]=data.rotationRate.z;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 gyros.push_back(imu);
                 [self processIMU_gyro];
             }];
        }
    }
}
- (IBAction)update_info_btn:(id)sender {
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *full_proj_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
    NSString *full_bag_addr = [full_proj_addr stringByAppendingString:@"/bag"];
    std::string full_bag_std=std::string([full_bag_addr UTF8String]);
    NSURL *url = [NSURL URLWithString:full_bag_addr];
     NSArray * dirContents =
    [[NSFileManager defaultManager] contentsOfDirectoryAtURL: url
                                  includingPropertiesForKeys:@[]
                                                     options:NSDirectoryEnumerationSkipsHiddenFiles
                                                       error:nil];
    NSPredicate *predicate = [NSPredicate predicateWithFormat:@"pathExtension='bag'"];
    NSArray *bagFiles = [dirContents filteredArrayUsingPredicate:predicate];
     int bag_count=[bagFiles count];
    self.bag_label.text=[NSString stringWithFormat:@"Bag: %d", bag_count];
    
    NSString *full_map_addr = [full_proj_addr stringByAppendingString:@"/global"];
    std::string map_addr_std=std::string([full_map_addr UTF8String]);
    url = [NSURL URLWithString:full_map_addr];
    dirContents =
    [[NSFileManager defaultManager] contentsOfDirectoryAtURL: url
                                  includingPropertiesForKeys:@[]
                                                     options:NSDirectoryEnumerationSkipsHiddenFiles
                                                       error:nil];
    predicate = [NSPredicate predicateWithFormat:@"pathExtension='map'"];
    NSArray *mapFiles = [dirContents filteredArrayUsingPredicate:predicate];
    std::vector<unsigned int> ids;
    for (NSURL* mapfile in dirContents) {
        NSString *mapfile_ns = [mapfile absoluteString];
        std::string filename_std=std::string([mapfile_ns UTF8String]);
        std::vector<std::string> splited_addr = chamo::split(filename_std,"/");
        std::vector<std::string> splited_filename = chamo::split(splited_addr.back(),".");
        filename_std=splited_filename[0];
        unsigned int temp_id=stoul(filename_std);
        ids.push_back(temp_id);
    }
    if(ids.size()==0){
        self.mp_label.text=[NSString stringWithFormat:@"MP: %d", 0];
        self.kf_label.text=[NSString stringWithFormat:@"KF: %d", 0];
    }else{
        gm::GlobalMapApi temp_api;
        api=temp_api;
        api.init("", map_addr_std);
        int mp_count;
        int kf_count;
        api.get_mpkf_count(mp_count, kf_count, ids);
        self.mp_label.text=[NSString stringWithFormat:@"MP: %d", mp_count];
        self.kf_label.text=[NSString stringWithFormat:@"KF: %d", kf_count];
    }
   
    
    
    
}

- (IBAction)start_mapping:(id)sender {
    dispatch_async( sessionQueue, ^{
        dispatch_async( dispatch_get_main_queue(), ^{
            self.proc_label.text=dele_map.sel_filename;
        });
        
        NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *full_proj_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
        NSString *full_map_addr = [full_proj_addr stringByAppendingString:@"/global"];
        NSString *full_bag_addr = [full_proj_addr stringByAppendingString:@"/bag"];
        NSBundle* myBundle = [NSBundle mainBundle];
        NSString*  mycam_str = [myBundle pathForResource:@"camera_config" ofType:@"txt"];
        NSString *config_addr = [mycam_str stringByDeletingLastPathComponent];
        std::string config_addr_std = std::string([config_addr UTF8String]);
        std::string map_addr_std=std::string([full_map_addr UTF8String]);
        std::string full_bag_std=std::string([full_bag_addr UTF8String]);
        
        BOOL isDir;
        NSFileManager *fileManager= [NSFileManager defaultManager];
        if(![fileManager fileExistsAtPath:full_map_addr isDirectory:&isDir])
            if(![fileManager createDirectoryAtPath:full_map_addr withIntermediateDirectories:YES attributes:nil error:NULL])
                NSLog(@"Error: Create folder failed %@", full_map_addr);
        gm::GlobalMapApi temp_api;
        temp_api.init(config_addr_std, map_addr_std);
        NSURL *url = [NSURL URLWithString:full_bag_addr];
        NSArray * dirContents =
        [[NSFileManager defaultManager] contentsOfDirectoryAtURL: url
                                      includingPropertiesForKeys:@[]
                                                         options:NSDirectoryEnumerationSkipsHiddenFiles
                                                           error:nil];
        NSPredicate *predicate = [NSPredicate predicateWithFormat:@"pathExtension='bag'"];
        NSArray *bagFiles = [dirContents filteredArrayUsingPredicate:predicate];
        std::vector<unsigned int> ids;
        for (NSURL* bagfile in bagFiles) {
            NSString *bagfile_ns = [bagfile absoluteString];
            std::string bagfile_std=std::string([bagfile_ns UTF8String]);
            std::vector<std::string> splited_addr = chamo::split(bagfile_std,"/");
            std::string full_bag_name=full_bag_std+"/"+splited_addr.back();
            NSString *full_cache_addr = [full_proj_addr stringByAppendingString:@"/cache"];
            if([fileManager fileExistsAtPath:full_cache_addr isDirectory:&isDir]){
                [fileManager removeItemAtPath:full_cache_addr error:nil];
            }
            [fileManager createDirectoryAtPath:full_cache_addr withIntermediateDirectories:YES attributes:nil error:NULL];
            NSString *full_local_addr = [full_proj_addr stringByAppendingString:@"/local"];
            if([fileManager fileExistsAtPath:full_local_addr isDirectory:&isDir]){
                [fileManager removeItemAtPath:full_local_addr error:nil];
            }
            [fileManager createDirectoryAtPath:full_local_addr withIntermediateDirectories:YES attributes:nil error:NULL];
            std::string cache_addr_std=std::string([full_cache_addr UTF8String]);
            std::string local_addr_std=std::string([full_local_addr UTF8String]);
            temp_api.process_bag(full_bag_name, cache_addr_std, local_addr_std, status_mapping);
        }
        dispatch_async( dispatch_get_main_queue(), ^{
            self.proc_label.text=@"None";
        });
    } );
}
- (IBAction)clear_map_btn:(id)sender {
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *full_map_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
    NSString *full_global_addr = [full_map_addr stringByAppendingString:@"/global"];
    BOOL isDir;
    if([[NSFileManager defaultManager] fileExistsAtPath:full_global_addr isDirectory:&isDir]){
        [[NSFileManager defaultManager] removeItemAtPath:full_global_addr error:nil];
    }
}

- (IBAction)load_map:(id)sender {
    if((int)[dele_map.file_list count]>0){
        NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *full_map_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
        full_map_addr = [full_map_addr stringByAppendingString:@"/global"];
        NSBundle* myBundle = [NSBundle mainBundle];
        NSString*  mycam_str = [myBundle pathForResource:@"camera_config" ofType:@"txt"];
        NSString *config_addr = [mycam_str stringByDeletingLastPathComponent];
        std::string config_addr_std = std::string([config_addr UTF8String]);
        std::string map_addr_std=std::string([full_map_addr UTF8String]);

        NSURL *url = [NSURL URLWithString:full_map_addr];
        NSArray * dirContents =
            [[NSFileManager defaultManager] contentsOfDirectoryAtURL: url
              includingPropertiesForKeys:@[]
                                 options:NSDirectoryEnumerationSkipsHiddenFiles
                                   error:nil];
        NSPredicate *predicate = [NSPredicate predicateWithFormat:@"pathExtension='map'"];
        NSArray *mapFiles = [dirContents filteredArrayUsingPredicate:predicate];
        std::vector<unsigned int> ids;
        for (NSURL* mapfile in dirContents) {
            NSString *mapfile_ns = [mapfile absoluteString];
            std::string filename_std=std::string([mapfile_ns UTF8String]);
            std::vector<std::string> splited_addr = chamo::split(filename_std,"/");
            std::vector<std::string> splited_filename = chamo::split(splited_addr.back(),".");
            filename_std=splited_filename[0];
            unsigned int temp_id=stoul(filename_std);
            ids.push_back(temp_id);
        }
        gm::GlobalMapApi temp_api;
        api=temp_api;
        api.init(config_addr_std, map_addr_std);
        std::vector<Eigen::Vector3d> out_pointcloud;
        std::vector<Eigen::Vector3d> kf_posis;
        std::vector<Eigen::Quaterniond> kf_rot;
        
        api.get_pointcloud(out_pointcloud, kf_posis, kf_rot, ids);
        Eigen::Vector3d av_posi;
        int mp_count=out_pointcloud.size();
        for(int i=0; i<out_pointcloud.size(); i++){
            av_posi=av_posi+out_pointcloud[i]/(double)mp_count;
        }
        for(int i=0; i<out_pointcloud.size(); i++){
            out_pointcloud[i](2)=out_pointcloud[i](2)-av_posi(2);
        }
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches;
        [self.sceneDelegate set_cur_posi: av_posi matches:matches];
        [self.sceneDelegate showPC: out_pointcloud kf:kf_posis];
    }
}
- (IBAction)locate:(id)sender {
}

- (void)viewDidAppear:(BOOL)animated{
    [self update_maplist];
}

- (IBAction)go_to_frame_page:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}


@end
