#import "RecordViewController.h"
#include "common_header.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include <chamo_common/common.h>
#import <mach/mach.h>
#include <math.h>
#include "chamo_common/common.h"
#include <sensor_msgs/NavSatFix.h>

@implementation RecordViewController
static void *ExposureTargetOffsetContext = &ExposureTargetOffsetContext;
- (void)viewDidLoad
{
    [super viewDidLoad];
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    updateQueue = dispatch_queue_create( "update queue", DISPATCH_QUEUE_SERIAL );
    recordingQueue = dispatch_queue_create( "recording queue", DISPATCH_QUEUE_SERIAL );
    sensorQueue = dispatch_queue_create( "sensor queue", DISPATCH_QUEUE_SERIAL );
    quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    locate_count=0;
    UIStoryboard* storyboard = [UIStoryboard storyboardWithName:@"Main" bundle:nil];
    frame_view = [storyboard instantiateViewControllerWithIdentifier:@"DetailViewController"];
    map_view = [storyboard instantiateViewControllerWithIdentifier:@"ViewController"];
    _sceneDelegate=map_view;
    _frameDelegate=frame_view;
    frame_view_update_count=0;
    sync_sensor_time =-1;
    _locationManager = [[CLLocationManager alloc] init];
    self.locationManager.delegate = self;
    self.locationManager.desiredAccuracy = kCLLocationAccuracyBest;
    self.locationManager.distanceFilter = kCLDistanceFilterNone;
    if ([_locationManager respondsToSelector:@selector(requestWhenInUseAuthorization)]) {
        [_locationManager requestWhenInUseAuthorization];
    }
    motionManager = [[CMMotionManager alloc] init];
    is_locating=false;
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
    [videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 30)];
    video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    video_output.videoSettings = newSettings;
    [video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([session canAddOutput:video_output]) {
        [session addOutput:video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }

    [video_output setSampleBufferDelegate:self queue:sensorQueue];
    [session commitConfiguration];
    if ( [videoDevice lockForConfiguration:&error] ) {
        if ( [videoDevice isFocusModeSupported:AVCaptureFocusModeLocked] ) {
            videoDevice.focusMode = AVCaptureFocusModeContinuousAutoFocus;
        }
        if([videoDevice isExposureModeSupported:AVCaptureExposureModeCustom]){
            videoDevice.exposureMode=AVCaptureExposureModeCustom;
        }
        //[videoDevice setFocusModeLockedWithLensPosition:0.8 completionHandler:nil];
        [videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( 0.01, 1000*1000*1000 ) ISO:AVCaptureISOCurrent completionHandler:nil];
        [videoDevice unlockForConfiguration];
    }else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }

    img_count=0;
    dele_map= [[BagListDelegate alloc] init];
    self.map_list_ui.delegate = dele_map;
    self.map_list_ui.dataSource = dele_map;
    status_mapping="done";
    
    dispatch_async( updateQueue, ^{
        while(true){
            if(api_proc.map_is_change){
                api_proc.map_is_change=false;
                [self.sceneDelegate showPC: api_proc.debug_mp_posi kf:api_proc.debug_kf_posi];
            }
            if(api_proc.match_is_change){
                api_proc.match_is_change=false;
                [self.sceneDelegate set_lines: api_proc.debug_matches];
            }
            if(api_proc.img_is_change){
                api_proc.img_is_change=false;
                [self.frameDelegate showFrame: api_proc.debug_img];
            }
            cv::Mat debug_img;
            struct task_basic_info info;
            mach_msg_type_number_t size = TASK_BASIC_INFO_COUNT;
            kern_return_t kerr = task_info(mach_task_self(),
                                           TASK_BASIC_INFO,
                                           (task_info_t)&info,
                                           &size);
            dispatch_async( dispatch_get_main_queue(), ^{
                self.status_label.text=[[NSString alloc] initWithUTF8String:status_mapping.c_str()];
                if( kerr == KERN_SUCCESS ) {
                    self.mem_label.text=[NSString stringWithFormat:@"%.3f MB", ((CGFloat)info.resident_size / 1048576)];
                } else {
                    self.mem_label.text=@"Err";
                }
            });
            [NSThread sleepForTimeInterval:0.2f];
        }
    });
}

- (void)observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context{
    
    if (context == ExposureTargetOffsetContext){
        frame_view_update_count++;
        float newExposureTargetOffset = [change[NSKeyValueChangeNewKey] floatValue];
        //NSLog(@"Offset is : %f, ISO: %f, shutter: %f",newExposureTargetOffset, videoDevice.ISO, CMTimeGetSeconds(videoDevice.exposureDuration));
        if(frame_view_update_count%10==0){
            [self.frameDelegate showCurInfo:-1 match_count:-1 expo_time:CMTimeGetSeconds(videoDevice.exposureDuration) iso:videoDevice.ISO offset:newExposureTargetOffset gps_accu: -1];
        }
       
        
        if(!videoDevice) return;
        CGFloat currentISO = videoDevice.ISO;
        CGFloat biasISO = 0;
        float isoChangeStep=0;
        float limit = 0.05;
        if (fabs(newExposureTargetOffset) > 1) {
            isoChangeStep = 50;
        } else if (fabs(newExposureTargetOffset) > 0.5) {
            isoChangeStep = 30;
        } else if (fabs(newExposureTargetOffset) > 0.2) {
            isoChangeStep = 10;
        } else if (fabs(newExposureTargetOffset) > 0.1) {
            isoChangeStep = 3;
        } else {
            isoChangeStep = 1;
        }
        if (newExposureTargetOffset > limit) {
            biasISO -= isoChangeStep;
        } else if (newExposureTargetOffset < -limit) {
            biasISO += isoChangeStep;
        } else {
            return;
        }
        
        if(biasISO){
            //Normalize ISO level for the current device
            CGFloat newISO = currentISO+biasISO;
            newISO = newISO > videoDevice.activeFormat.maxISO? videoDevice.activeFormat.maxISO : newISO;
            newISO = newISO < videoDevice.activeFormat.minISO? videoDevice.activeFormat.minISO : newISO;
            double newDur=0.01;
            AVCaptureDeviceFormat *activeFormat = videoDevice.activeFormat;
            if(newISO<=activeFormat.minISO){
                newDur=CMTimeGetSeconds(videoDevice.exposureDuration)-0.001;
                if(newDur<0.001){
                    newDur=0.001;
                }
            }else if(newISO>activeFormat.minISO){
                if(CMTimeGetSeconds(videoDevice.exposureDuration)<0.01){
                    newDur=CMTimeGetSeconds(videoDevice.exposureDuration)+0.001;
                }
            }
            NSError *error = nil;
            if ([videoDevice lockForConfiguration:&error]) {
                [videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( newDur, 1000*1000*1000 ) ISO:newISO completionHandler:^(CMTime syncTime) {}];
                [videoDevice unlockForConfiguration];
            }
        }
    }
}
- (IBAction)del:(id)sender {
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *full_map_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
    bool isDir;
    NSFileManager *fileManager= [NSFileManager defaultManager];
    if([fileManager fileExistsAtPath:full_map_addr isDirectory:&isDir]){
        [fileManager removeItemAtPath:full_map_addr error:nil];
    }
    [self update_maplist];
}

- (IBAction)goto_frame_btn:(id)sender {
    [self presentViewController:frame_view animated:NO completion:nil];
}
- (IBAction)goto_map_btn:(id)sender {
    [self presentViewController:map_view animated:NO completion:nil];
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
                dispatch_async(recordingQueue, ^{
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
    sync_sensor_time = (double)timestamp.value/(double)timestamp.timescale;
    sync_sys_time = [NSDate date];
    double time_sec = (double)timestamp.value/(double)timestamp.timescale;
    UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
    cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
    if(is_locating){
        locate_count++;
        NSDate *methodStart = [NSDate date];
        if(locate_count%3==0){
            dispatch_async( recordingQueue, ^{
                NSDate *methodFinish = [NSDate date];
                NSTimeInterval executionTime = [methodFinish timeIntervalSinceDate:methodStart];
                if(executionTime<1.0){
                    Eigen::Matrix4d pose;
                    std::vector<cv::Point2f> inliers_kp;
                    std::vector<Eigen::Vector3d> inliers_mp;
                    cv::Mat img_gray;
                    cv::Mat debug_gray;
                    cv::cvtColor(img_cv, img_gray, CV_BGRA2GRAY);
                    NSDate *loc_before = [NSDate date];
                    float match_time;
                    api.locate_img(img_gray, debug_gray, pose, Eigen::Vector3d(-1,-1,-1), inliers_kp, inliers_mp, match_time);
                    NSDate *loc_after= [NSDate date];
                    NSTimeInterval locTime = [loc_after timeIntervalSinceDate:loc_before];
                    [self.frameDelegate showTime: locTime match_time:match_time ];
                    if(inliers_kp.size()>20){
                        cv::cvtColor(debug_gray, debug_gray, CV_GRAY2BGRA);
                        for(int i=0; i<inliers_kp.size(); i++){
                            for(int n=-3; n<=3; n++){
                                for(int m=-3; m<=3; m++){
                                    int row=inliers_kp[i].y+n;
                                    int col=inliers_kp[i].x+m;
                                    if(col<0 || col>=debug_gray.cols || row<0 || row>=debug_gray.rows){
                                        continue;
                                    }
                                    debug_gray.at<cv::Scalar_<unsigned char>>(row, col )=cv::Scalar(0,0,255,255);
                                }
                            }
                        }
                        [self.frameDelegate showFrame: debug_gray];
                        [self.sceneDelegate set_cur_posi: pose.block(0,3,3,1) matches:inliers_mp update_center:false];
                    }
                }
            });
        }
    }else{
        [self.frameDelegate showFrame: img_cv];
        sensor_msgs::CompressedImage img_ros_img;
        std::vector<unsigned char> binaryBuffer_;
        cv::imencode(".jpg", img_cv, binaryBuffer_);
        img_ros_img.data=binaryBuffer_;
        img_ros_img.header.seq=img_count;
        img_ros_img.header.stamp= ros::Time(sync_sensor_time);
        img_ros_img.format="jpeg";
        dispatch_async( recordingQueue, ^{
            if(is_recording_bag){
                if(bag_ptr->isOpen()){
                    NSDate * t1 = [NSDate date];
                    NSTimeInterval now = [t1 timeIntervalSince1970];
                    bag_ptr->write("img", ros::Time(now), img_ros_img);
                    if(bag_ptr->getSize()>3500000000){
                        [self switch_recording: false];
                        [self switch_recording: true];
                    }
                }
            }
        });
    }
    img_count++;
}

- (void)viewWillDisappear:(BOOL)animated{
}

- (void) switch_recording: (bool) doRecord{
    if(doRecord){
        dispatch_async( recordingQueue, ^{
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
    }else{
        is_recording_bag=false;
        dispatch_async( recordingQueue, ^{
            bag_ptr->close();
            NSLog(@"close the bag");
        });
    }
}
- (IBAction)start_record:(id)sender {
    if(!is_recording_bag){
        [self switch_recording: true];
        [sender setTitle:@"Stop Re" forState:UIControlStateNormal];
    }else{
        [self switch_recording: false];
        [sender setTitle:@"Record" forState:UIControlStateNormal];
    }
}
- (void)locationManager:(CLLocationManager *)manager didUpdateToLocation:(CLLocation *)newLocation fromLocation:(CLLocation *)oldLocation {
    
    if (newLocation.horizontalAccuracy < 0) {
        return;
    }
    NSTimeInterval locationAge = -[newLocation.timestamp timeIntervalSinceNow];
    if (locationAge > 5.0) {
        return;
    }
    [self recordGPS: newLocation];
}

bool outOfChina(double lat, double lon) {
    if (lon < 72.004 || lon > 137.8347) return true;
    if (lat < 0.8293 || lat > 55.8271) return true;
    return false;
}
double transformLat(double x, double y) {
    double pi = 3.1415926535897932384626;
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
    + 0.2 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}
double transformLon(double x, double y) {
    double pi = 3.1415926535897932384626;
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1* sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}

void gps84_To_Gcj02(double& lat, double& lon) {
    double pi = 3.1415926535897932384626;
    double ee = 0.00669342162296594323;
    double a = 6378245.0;
    if (outOfChina(lat, lon)) {
        return;
    }
    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * pi;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
    lat = lat + dLat;
    lon = lon + dLon;
}

- (void)recordGPS:(CLLocation *)newLocation{
    if(sync_sensor_time<0){
        return;
    }
    
    [self.frameDelegate showCurInfo:-1 match_count:-1 expo_time:-1 iso:-1 offset:-1  gps_accu: newLocation.horizontalAccuracy];
    sensor_msgs::NavSatFix msg;
    msg.latitude=newLocation.coordinate.latitude;
    msg.longitude=newLocation.coordinate.longitude;
    gps84_To_Gcj02(msg.latitude, msg.longitude);
    msg.altitude=newLocation.altitude;
    msg.position_covariance[0]=newLocation.horizontalAccuracy;
    msg.position_covariance[3]=newLocation.horizontalAccuracy;
    msg.position_covariance[6]=newLocation.verticalAccuracy;
    static int gps_data_seq=0;
    msg.header.seq=gps_data_seq;
    gps_data_seq++;
    NSDate* eventDate = newLocation.timestamp;
    
    double time_change = [eventDate timeIntervalSinceDate:sync_sys_time];
    double howRecent = time_change+sync_sensor_time;
    msg.header.stamp = ros::Time(howRecent);
    msg.header.frame_id="map";
    dispatch_async( recordingQueue, ^{
        if (is_recording_bag){
            if(bag_ptr->isOpen()){
                NSDate * t1 = [NSDate date];
                NSTimeInterval now = [t1 timeIntervalSince1970];
                bag_ptr->write("gps", ros::Time(now), msg);
            }
        }
    });
}

- (IBAction)start_sensor:(id)sender {
    if(session.running){
        [sender setTitle:@"Sensor" forState:UIControlStateNormal];
        [self removeObserver:self forKeyPath:@"videoDevice.exposureTargetOffset" context:ExposureTargetOffsetContext];
        [session stopRunning];
        [self.locationManager stopUpdatingLocation];
    }else{
        [sender setTitle:@"Stop Sen" forState:UIControlStateNormal];
        [self addObserver:self forKeyPath:@"videoDevice.exposureTargetOffset" options:NSKeyValueObservingOptionNew context:ExposureTargetOffsetContext];
        [session startRunning];
        [self.locationManager startUpdatingLocation];
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
    
    NSString *full_map_addr = [full_proj_addr stringByAppendingString:@"/release"];
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
        std::cout<<filename_std<<std::endl;
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

-(void) copy_folder:(NSString*) sour dest:(NSString*) dest{
    NSFileManager *fileManager = [NSFileManager defaultManager];
    NSArray *sourceFiles = [fileManager contentsOfDirectoryAtPath:sour error:NULL];
    NSError *copyError = nil;
    for (NSString *currentFile in sourceFiles)
    {
        BOOL isDirectory = NO;
        NSString* fullFilePath = [NSString stringWithFormat:@"%@/%@",sour,currentFile];
        if ([fileManager fileExistsAtPath:fullFilePath isDirectory:&isDirectory] && !isDirectory)
        {
            if (![fileManager copyItemAtPath:[sour stringByAppendingPathComponent:currentFile] toPath:[dest stringByAppendingPathComponent:currentFile] error:&copyError])
            {
                NSLog(@"Error Copying: %@", [copyError description]);
            }
        }
    }
}

- (unsigned long long) fastFolderSizeAtFSRef:(NSString *)theFilePath
{
    unsigned long long totalSize = 0;
    NSFileManager *fileManager = [NSFileManager defaultManager];
    BOOL  isdirectory;
    NSError *error;
    if ([fileManager fileExistsAtPath:theFilePath])
    {
        NSMutableArray * directoryContents = [[fileManager contentsOfDirectoryAtPath:theFilePath error:&error] mutableCopy];
        for (NSString *fileName in directoryContents)
        {
            if (([fileName rangeOfString:@".DS_Store"].location != NSNotFound) )
                continue;
            NSString *path = [theFilePath stringByAppendingPathComponent:fileName];
            if([fileManager fileExistsAtPath:path isDirectory:&isdirectory] && isdirectory  )
            {
                totalSize =  totalSize + [self fastFolderSizeAtFSRef:path];
            }else{
                unsigned long long fileSize = [[fileManager attributesOfItemAtPath:path error:&error] fileSize];
                totalSize = totalSize + fileSize;
            }
        }
    }
    return totalSize;
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
        NSString *full_release_addr = [full_proj_addr stringByAppendingString:@"/release"];
        std::string map_release_std=std::string([full_release_addr UTF8String]);
        BOOL isDir;
        NSFileManager *fileManager= [NSFileManager defaultManager];
        if(![fileManager fileExistsAtPath:full_map_addr isDirectory:&isDir])
            if(![fileManager createDirectoryAtPath:full_map_addr withIntermediateDirectories:YES attributes:nil error:NULL])
                NSLog(@"Error: Create folder failed %@", full_map_addr);
        if(![fileManager fileExistsAtPath:full_release_addr isDirectory:&isDir])
            if(![fileManager createDirectoryAtPath:full_release_addr withIntermediateDirectories:YES attributes:nil error:NULL])
                NSLog(@"Error: Create folder failed %@", full_release_addr);
        gm::GlobalMapApi temp_api;
        api_proc=temp_api;
        api_proc.init(config_addr_std, map_addr_std);
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
            if([fileManager fileExistsAtPath:full_map_addr isDirectory:&isDir]){
                [fileManager removeItemAtPath:full_map_addr error:nil];
            }
            [fileManager createDirectoryAtPath:full_map_addr withIntermediateDirectories:YES attributes:nil error:NULL];
            [self copy_folder:  full_release_addr dest:full_map_addr];
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
            unsigned long long size_before =[self fastFolderSizeAtFSRef:full_map_addr];
            bool update_re = api_proc.process_bag(full_bag_name, cache_addr_std, local_addr_std, status_mapping);
            unsigned long long size_after =[self fastFolderSizeAtFSRef:full_map_addr];
            if(update_re==true && size_after>size_before){
                if([fileManager fileExistsAtPath:full_release_addr isDirectory:&isDir]){
                    [fileManager removeItemAtPath:full_release_addr error:nil];
                }
                [fileManager createDirectoryAtPath:full_release_addr withIntermediateDirectories:YES attributes:nil error:NULL];
                [self copy_folder:full_map_addr dest:full_release_addr];
                [fileManager removeItemAtPath:[[NSString alloc] initWithUTF8String:full_bag_name.c_str()] error:nil];
            }
        }
        dispatch_async( dispatch_get_main_queue(), ^{
            self.proc_label.text=@"None";
        });
    } );
}
- (IBAction)clear_map_btn:(id)sender {
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *full_map_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
    NSString *full_global_addr = [full_map_addr stringByAppendingString:@"/release"];
    BOOL isDir;
    if([[NSFileManager defaultManager] fileExistsAtPath:full_global_addr isDirectory:&isDir]){
        [[NSFileManager defaultManager] removeItemAtPath:full_global_addr error:nil];
    }
}

- (IBAction)load_map:(id)sender {
    if((int)[dele_map.file_list count]>0){
        NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *full_map_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
        NSString *background_map_addr = [full_map_addr stringByAppendingString:@"/background"];
        full_map_addr = [full_map_addr stringByAppendingString:@"/release"];
        
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
        BOOL isDir;
        cv::Mat background_img;
        double rot=0;
        double trans_x=0;
        double trans_y=0;
        double scale=1;
        bool hasTrans=false;
        std::string trans_map_addr_std;
        if([[NSFileManager defaultManager] fileExistsAtPath:background_map_addr isDirectory:&isDir]){
            NSString *background_img_map_addr = [background_map_addr stringByAppendingString:@"/back_img.jpg"];
            if([[NSFileManager defaultManager] fileExistsAtPath:background_img_map_addr isDirectory:&isDir]){
                std::string background_img_std=std::string([background_img_map_addr UTF8String]);
                background_img= cv::imread(background_img_std, cv::IMREAD_COLOR );
            }
            NSString *trans_map_addr = [background_map_addr stringByAppendingString:@"/trans.txt"];
            trans_map_addr_std=std::string([trans_map_addr UTF8String]);
            if([[NSFileManager defaultManager] fileExistsAtPath:trans_map_addr isDirectory:&isDir]){
                hasTrans=true;
                std::string line;
                std::ifstream infile(trans_map_addr_std.c_str());
                std::getline(infile, line);
                std::vector<std::string> splited = chamo::split(line, ",");
                rot=atof(splited[0].c_str());
                trans_x=atof(splited[1].c_str());
                trans_y=atof(splited[2].c_str());
                scale=atof(splited[3].c_str());
            }
        }
       
        
        api.Release();
        gm::GlobalMapApi temp_api;
        api=temp_api;
        api.init(config_addr_std, map_addr_std);
        api.load_map(ids);
        std::vector<Eigen::Vector3d> out_pointcloud;
        std::vector<Eigen::Vector3d> kf_posis;
        std::vector<Eigen::Quaterniond> kf_rot;
        api.get_pointcloud(out_pointcloud, kf_posis, kf_rot, ids);
        [self.sceneDelegate showPC: out_pointcloud kf:kf_posis];
        if(!background_img.empty()){
            if(hasTrans==false){
                scale=-1;
            }
            [self.sceneDelegate setBackground: background_img rot:rot trans_x:trans_x trans_y:trans_y scale:scale file:trans_map_addr_std];
        }
        
    }
}
- (IBAction)locate:(id)sender {
    if(!is_locating){
        is_locating=true;
        [sender setTitle:@"Stop Loc" forState:UIControlStateNormal];
    }else{
        is_locating=false;
        [sender setTitle:@"Locate" forState:UIControlStateNormal];
    }
    
}

- (void)viewDidAppear:(BOOL)animated{
    [self update_maplist];
}

@end
