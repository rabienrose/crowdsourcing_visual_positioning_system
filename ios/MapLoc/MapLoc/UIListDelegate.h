#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
@interface BagListDelegate : NSObject<UIPickerViewDataSource, UIPickerViewDelegate>{

}

@property (nonatomic) NSMutableArray *file_list;
@property (nonatomic) NSString *sel_filename;
@end

