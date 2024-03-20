#include <pc_localizer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localize_pc");
  ros::NodeHandle nh("~");

  pc_localizer::PCLocalizer node(nh);
  static tf2_ros::TransformBroadcaster tf_broadcast;
  geometry_msgs::TransformStamped location;
  geometry_msgs::Transform loc;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    if(node.got_transform)
    {

      location.header.stamp = ros::Time::now();
      location.header.frame_id = node.base_frame_name.c_str();
      location.child_frame_id = node.object_frame_name.c_str();    
      tf2::Matrix3x3 R ( node.final_transform(0,0), node.final_transform(0,1), node.final_transform(0,2), 
              node.final_transform(1,0), node.final_transform(1,1), node.final_transform(1,2), 
              node.final_transform(2,0), node.final_transform(2,1), node.final_transform(2,2) );

      tf2::Vector3 p (node.final_transform(0,3), node.final_transform(1,3), node.final_transform(2,3) );

      tf2::Transform T(R, p);
      tf2::convert(T, loc);

      location.transform = loc;
      tf_broadcast.sendTransform(location);

    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}  // end main()