#include <ros/ros.h>
#include <grad_recover.h>

using namespace air_bumper;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grad_recover_node");
    ros::NodeHandle nh;
    ros::Rate rate(50.0);

    GradMap grad_map;

    grad_map.initMap(nh);

    ros::Time time_now = ros::Time::now();
    ros::Time time_lst = ros::Time::now();

    // Main Loop
    while (ros::ok())
    {
        // CallBack for Updating Sensor State
        ros::spinOnce();

        time_now = ros::Time::now();

        if((time_now - time_lst).toSec() > 0.2)
        {
            Eigen::Vector3d grad;
            Eigen::Vector3i coord;

            double dist = grad_map.getDistWithGrad(grad_map._map_center, grad);

            coord = (grad_map._local_size.cast<float>() * 0.5).cast<int>();

            int idx = grad_map.coord2idx_edt(coord);

            cout << "Map Center:" << coord(2) << " " << coord(1) << " " << coord(0) << endl;

            cout << "idx: " << idx << " EDT Center Dist: " << grad_map._val_map[idx].d * grad_map._voxel_width
                 << " s: " << grad_map._val_map[0].s
                 << " o: " << grad_map._val_map[0].o << endl;

            cout << "Pos: x: " << grad_map.pos_info(0) << " y: " << grad_map.pos_info(1) << " z: " << grad_map.pos_info(2) << endl;

            cout << "Grad:"
                 << " x: " << grad_map.grad_info(0)
                 << " y: " << grad_map.grad_info(1)
                 << " z: " << grad_map.grad_info(2)
                 << endl;
            cout<< "****************************" << endl;
            time_lst = time_now;
        }
        
        rate.sleep();
    }

    return 0;
}