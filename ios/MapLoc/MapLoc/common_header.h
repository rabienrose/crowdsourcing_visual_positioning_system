//
//  common_header.h
//  opencv_ios_try
//
//  Created by test on 10/1/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef common_header_h
#define common_header_h
#include <opencv2/opencv.hpp>
#import<Foundation/Foundation.h>
#import <UIKit/UIKit.h>


@interface mm_Try : NSObject
+(cv::Mat)cvMatFromUIImage:(UIImage *)image;
+(UIImage *) convertImag: (UIImage *)image;
+(cv::Mat)cvMatGrayFromUIImage:(UIImage *)image;
+(UIImage *)UIImageFromCVMat:(cv::Mat)cvMat;
@end
#endif /* common_header_h */
