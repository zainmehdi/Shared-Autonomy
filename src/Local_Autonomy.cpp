//
// Created by zain on 17. 11. 30.
//

#include "opencv_test/feature_extractor.h"
int main(int argc, char* argv[]){

    ros::init(argc, argv, "local_autonomy");
    feature_extractor ex;
    ros::spin();
    return 0;
}