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

- (SCNGeometry*) buildGird{
    double step=5;
    int count=20;
    float height=2.0;
    Eigen::Matrix3Xf vertice;
    for (int i=-count; i<count; i++){
        Eigen::Vector3f p1(i*step, -step*count, -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p1;
        Eigen::Vector3f p2(i*step, step*count, -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p2;
        
        Eigen::Vector3f p3(-step*count, i*step, -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p3;
        Eigen::Vector3f p4(step*count, i*step , -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p4;
    }
    int vertex_count=vertice.cols();
    int size_byte=vertice.cols()*vertice.rows()*4;
    NSData *data = [NSData dataWithBytes:vertice.data() length:size_byte];
    SCNGeometrySource *vertexSource;
    vertexSource = [SCNGeometrySource geometrySourceWithData:data
                                                    semantic:SCNGeometrySourceSemanticVertex
                                                 vectorCount:vertex_count
                                             floatComponents:YES
                                         componentsPerVector:3
                                           bytesPerComponent:4
                                                  dataOffset:0
                                                  dataStride:12];
    
    Eigen::Matrix3Xf colors;
    colors.conservativeResize(colors.rows(), vertice.cols());
    for (int i=0; i<vertice.cols(); i++){
        Eigen::Vector3f p1(0.1, 0.1, 0.1);
        colors.col(i) = p1;
    }
    NSData *data_color = [NSData dataWithBytes:colors.data() length:size_byte];
    SCNGeometrySource *colorSource;
    colorSource = [SCNGeometrySource geometrySourceWithData:data_color
                                                   semantic:SCNGeometrySourceSemanticColor
                                                vectorCount:vertex_count
                                            floatComponents:YES
                                        componentsPerVector:3
                                          bytesPerComponent:4
                                                 dataOffset:0
                                                 dataStride:12];
    
    std::vector<int> indice;
    for(int i=0; i<vertex_count; i++){
        indice.push_back(i);
    }
    NSData *ind_data = [NSData dataWithBytes:indice.data() length:4*vertex_count];
    SCNGeometryElement* vertexInd = [SCNGeometryElement geometryElementWithData:ind_data primitiveType:SCNGeometryPrimitiveTypeLine primitiveCount:vertex_count bytesPerIndex:4];
    
    NSMutableArray *vsourArrar = [[NSMutableArray alloc] init];
    [vsourArrar addObject:vertexSource];
    [vsourArrar addObject:colorSource];
    NSMutableArray *isourArrar = [[NSMutableArray alloc] init];
    [isourArrar addObject:vertexInd];
    SCNGeometry* gird_geo =[SCNGeometry geometryWithSources:vsourArrar elements:isourArrar];
    return gird_geo;
}

- (void)renderer:(id<SCNSceneRenderer>)renderer updateAtTime:(NSTimeInterval)time{
    
    cameraNode.eulerAngles = SCNVector3Make(0,0,cam_roll/180*PI);
    cameraNode.camera.orthographicScale=cam_scale;
    
    cameraNode.camera.usesOrthographicProjection=YES;
    cameraNode.position=SCNVector3Make(cam_tar_x, cam_tar_y, cam_distence);
}

- (void) handleTap:(UIGestureRecognizer*)gestureRecognize
{
    [self presentViewController:detail_view animated:NO completion:nil];
}

- (void) handlePan:(UIGestureRecognizer*)gestureRecognize
{
    SCNView *scnView = (SCNView *)self.view;
    UIPanGestureRecognizer* reco = (UIPanGestureRecognizer*)gestureRecognize;
    CGPoint p = [reco translationInView:scnView];
    if(reco.numberOfTouches==2){
    }else if(reco.numberOfTouches==1){
        if(reco.state==1){
            temp_cam_tar_x=p.x;
            temp_cam_tar_y=p.y;
        }else if(reco.state==2){
            float rate=cam_scale/200;
            float rot_x=(p.x-temp_cam_tar_x)*cos(-cam_roll/180*PI)-(p.y-temp_cam_tar_y)*sin(-cam_roll/180*PI);
            float rot_y=(p.x-temp_cam_tar_x)*sin(-cam_roll/180*PI)+(p.y-temp_cam_tar_y)*cos(-cam_roll/180*PI);
            cam_tar_x=cam_tar_x-rot_x*rate;
            cam_tar_y=cam_tar_y+rot_y*rate;
            temp_cam_tar_x=p.x;
            temp_cam_tar_y=p.y;
        }
    }
}

- (void) handleRot:(UIGestureRecognizer*)gestureRecognize
{
    SCNView *scnView = (SCNView *)self.view;
    UIRotationGestureRecognizer* reco = (UIRotationGestureRecognizer*)gestureRecognize;
    if(reco.state==1){
        temp_cam_roll=reco.rotation;
    }else if(reco.state==2){
        cam_roll=cam_roll+(reco.rotation-temp_cam_roll)*180/PI;
        if(cam_yaw>360){
            cam_roll=cam_roll-360;
        }
        if(cam_yaw<0){
            cam_roll=cam_roll+360;
        }
        temp_cam_roll=reco.rotation;
    }
}

- (void) handlePin:(UIGestureRecognizer*)gestureRecognize
{
    SCNView *scnView = (SCNView *)self.view;
    UIPinchGestureRecognizer* reco = (UIPinchGestureRecognizer*)gestureRecognize;
    if(reco.state==1){
        temp_scale=reco.scale;
    }else if(reco.state==2){
        cam_scale=cam_scale*(temp_scale/reco.scale);
        temp_scale=reco.scale;
    }
}

- (SCNGeometry*) fillPC: (const std::vector<Eigen::Vector3d>&)posis color:(Eigen::Vector3f) color{
    
    Eigen::Matrix3Xf G_observer_positions;
    G_observer_positions.resize(3, posis.size());

    for(int i=0; i<posis.size(); i++){
        G_observer_positions.col(i)=posis[i].cast<float>();
    }
    int size_byte=  G_observer_positions.cols()*G_observer_positions.rows()*4;
    int vertex_count=G_observer_positions.cols();
    NSData *data = [NSData dataWithBytes:G_observer_positions.data() length:size_byte];
    SCNGeometrySource *vertexSource;
    
    vertexSource = [SCNGeometrySource geometrySourceWithData:data
                                                    semantic:SCNGeometrySourceSemanticVertex
                                                 vectorCount:vertex_count
                                             floatComponents:YES
                                         componentsPerVector:3
                                           bytesPerComponent:4
                                                  dataOffset:0
                                                  dataStride:12];
    
    Eigen::Matrix3Xf colors;
    colors.conservativeResize(colors.rows(), vertex_count);
    for (int i=0; i<colors.cols(); i++){
        colors.col(i) = color;
    }
    NSData *data_color = [NSData dataWithBytes:colors.data() length:size_byte];
    SCNGeometrySource *colorSource;
    colorSource = [SCNGeometrySource geometrySourceWithData:data_color
                                                   semantic:SCNGeometrySourceSemanticColor
                                                vectorCount:vertex_count
                                            floatComponents:YES
                                        componentsPerVector:3
                                          bytesPerComponent:4
                                                 dataOffset:0
                                                 dataStride:12];
    
    
    std::vector<int> indice;
    for(int i=0; i<vertex_count; i++){
        indice.push_back(i);
    }
    NSData *ind_data = [NSData dataWithBytes:indice.data() length:4*vertex_count];
    SCNGeometryElement* vertexInd = [SCNGeometryElement geometryElementWithData:ind_data primitiveType:SCNGeometryPrimitiveTypePoint primitiveCount:vertex_count bytesPerIndex:4];
    vertexInd.minimumPointScreenSpaceRadius = 5;
    vertexInd.maximumPointScreenSpaceRadius=5;
    NSMutableArray *vsourArrar = [[NSMutableArray alloc] init];
    [vsourArrar addObject:vertexSource];
    [vsourArrar addObject:colorSource];
    NSMutableArray *isourArrar = [[NSMutableArray alloc] init];
    [isourArrar addObject:vertexInd];
    SCNGeometry* pc_geo =[SCNGeometry geometryWithSources:vsourArrar elements:isourArrar];
    return pc_geo;
}


- (void)viewDidLoad {
    [super viewDidLoad];
    
    SCNScene *scene = [SCNScene scene];
    worldNode=scene.rootNode;

    cameraNode = [SCNNode node];
    cameraNode.camera = [SCNCamera camera];
    cameraNode.camera.usesOrthographicProjection=YES;
    cameraNode.camera.orthographicScale=20;
    cameraNode.camera.automaticallyAdjustsZRange=YES;
    [worldNode addChildNode:cameraNode];
    cam_distence=500;
    cameraNode.position = SCNVector3Make(0, 0, cam_distence);

//    meNode = [SCNNode node];
//    meNode.geometry = [SCNPyramid pyramidWithWidth:1 height:2 length:1];
//    [worldNode addChildNode:meNode];

    SCNNode* girdNode = [SCNNode node];
    girdNode.geometry = [self buildGird];
    [worldNode addChildNode:girdNode];

    SCNView *scnView = (SCNView *)self.view;
    UIPanGestureRecognizer *panGesture = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handlePan:)];
    UIRotationGestureRecognizer *rotGesture = [[UIRotationGestureRecognizer alloc] initWithTarget:self action:@selector(handleRot:)];
    UIPinchGestureRecognizer *pinGesture = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(handlePin:)];
    UITapGestureRecognizer *tapGesture = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleTap:)];
    NSMutableArray *gestureRecognizers = [NSMutableArray array];
    [gestureRecognizers addObject:panGesture];
    [gestureRecognizers addObject:rotGesture];
    [gestureRecognizers addObject:pinGesture];
    [gestureRecognizers addObject:tapGesture];
    scnView.gestureRecognizers = gestureRecognizers;

    scnView.scene = scene;
    scnView.allowsCameraControl = YES;
    scnView.showsStatistics = YES;
    scnView.backgroundColor = [UIColor blackColor];
    scnView.delegate = self;
    scnView.playing=YES;
    scnView.preferredFramesPerSecond=15;

    cam_pitch=90;
    cam_yaw=0;
    cam_roll=0;
    cam_tar_x=0;
    cam_tar_y=0;
    cam_scale=cameraNode.camera.orthographicScale;
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    if(pcNode==nullptr){
        pcNode = [SCNNode node];
        [worldNode addChildNode:pcNode];
    }
    
    UIStoryboard* storyboard = [UIStoryboard storyboardWithName:@"Main" bundle:nil];
    detail_view = [storyboard instantiateViewControllerWithIdentifier:@"DetailViewController"];
    detail_view.sceneDelegate=self;
}

- (void) showTraj: (std::vector<Eigen::Vector3d>) posis{
    dispatch_async( dispatch_get_main_queue(), ^{
        if(pcNode==nullptr){
            pcNode = [SCNNode node];
            [worldNode addChildNode:pcNode];
        }
        std::cout<<posis.size()<<std::endl;
        pcNode.geometry=[self fillPC: posis color:Eigen::Vector3f(1.0, 0.0, 0.0)];
    } );
}

@end
