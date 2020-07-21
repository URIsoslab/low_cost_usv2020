#include <guidance/guidance.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "guidance_node");

  Guidance guidance_node;

  ros::spin();

  return (0);
}
