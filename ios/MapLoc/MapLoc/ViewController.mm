//
//  ViewController.m
//  MapLoc
//
//  Created by zili wang on 2019/4/5.
//  Copyright © 2019 zili wang. All rights reserved.
//

#import "ViewController.h"
#include "Eigen/Core"
@interface ViewController ()

@end
#define PI 3.1415
@implementation ViewController

- (void) handleTap:(UIGestureRecognizer*)gestureRecognize
{
}

- (void) handlePan:(UIGestureRecognizer*)gestureRecognize
{

}

- (void) handleRot:(UIGestureRecognizer*)gestureRecognize
{
}

- (void) handlePin:(UIGestureRecognizer*)gestureRecognize
{
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
    
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    UIStoryboard* storyboard = [UIStoryboard storyboardWithName:@"Main" bundle:nil];
    detail_view = [storyboard instantiateViewControllerWithIdentifier:@"DetailViewController"];
}

@end
