#include <ros/ros.h>
#include <geometry_msgs/Pose.h>



int main(int argc, char* argv[]){
    
    int dim_x = 10;
    int dim_y = 10;
    int dim_z = 10;
    int voxel_side_length = 10;

	
    printf("\nCreating CUDA VoxelMap(s) of size:\n");
    printf(" - dim x             : %u\n", dim_x);
    printf(" - dim y             : %u\n", dim_y);
    printf(" - dim z             : %u\n", dim_z);
    printf(" - voxel_side_length : %f\n", voxel_side_length);
    printf("--------------------------------\n\n");

}
