//
//  ViewController.m
//  MapLoc
//
//  Created by zili wang on 2019/4/5.
//  Copyright © 2019 zili wang. All rights reserved.
//

#import "UIListDelegate.h"
@interface BagListDelegate ()

@end
@implementation BagListDelegate

- (NSInteger)numberOfComponentsInPickerView:(UIPickerView *)pickerView{
    return 1;
}
- (NSInteger)pickerView:(UIPickerView *)pickerView numberOfRowsInComponent:(NSInteger)component{
    return _file_list.count;
}
- (NSString *)pickerView:(UIPickerView *)pickerView titleForRow:(NSInteger)row forComponent:(NSInteger)component{
    return _file_list[row];
}
- (void)pickerView:(UIPickerView *)pickerView didSelectRow:(NSInteger)row inComponent:(NSInteger)component{
    if(row<_file_list.count){
        _sel_filename = _file_list[row];
    }
}

@end
