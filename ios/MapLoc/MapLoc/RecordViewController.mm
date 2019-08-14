#import "RecordViewController.h"
#include "common_header.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>

@implementation RecordViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    motionManager = [[CMMotionManager alloc] init];
    
    session = [[AVCaptureSession alloc] init];
    NSError *error = nil;
    [session beginConfiguration];
    session.sessionPreset =AVCaptureSessionPreset640x480;
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
    if ( [videoDevice lockForConfiguration:&error] ) {
        videoDevice.exposureMode=AVCaptureExposureModeLocked;
        [videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( 0.001, 1000*1000*1000 ) ISO:900 completionHandler:nil];
    }
    [videoDevice unlockForConfiguration];
    
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



- (void) start_slam: (NSString *) bag_name
{
    
}

- (IBAction)new_btn:(id)sender {
    
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
        dispatch_async( sessionQueue, ^{
            NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSDate *date = [NSDate date];
            NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
            [formatter setDateFormat:@"MM-dd-HH-mm-ss"];
            NSString *timeString = [formatter stringFromDate:date];
            NSString *string1 = [NSString stringWithFormat:@"%@.bag",timeString];
            NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:string1];
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
        });
        [sender setTitle:@"Record" forState:UIControlStateNormal];
        [self update_maplist];
    }
    
}

- (IBAction)start_sensor:(id)sender {
    if(session.running){
        [session stopRunning];
        [sender setTitle:@"Start Sensor" forState:UIControlStateNormal];
    }else{
        [session startRunning];
        [sender setTitle:@"Stop" forState:UIControlStateNormal];
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
- (IBAction)start_mapping:(id)sender {
    
}

- (IBAction)load_map:(id)sender {
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
