//
// Created by kari on 18. 1. 31.
//

#include "opencv_test/Shared_Autonomy.h"
int main(int argc, char* argv[]) {

    ros::init(argc, argv, "Shared_Autonomy");
    Shared_Autonomy_Vine SA;
    ros::spin();
    return 0;
}
