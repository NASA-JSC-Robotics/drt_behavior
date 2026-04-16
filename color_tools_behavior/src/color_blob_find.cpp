#include "color_tools_behavior/color_blob_find.hpp"

namespace color_tools_behavior
{

bool ColorBlobFind::setRequest(Request::SharedPtr& request)
{
  getInput("desired_blob", request->desired_blob);
  getInput("min_blob_size", request->min_blob_size);
  getInput("color", request->color);

  return true;
}

BT::NodeStatus ColorBlobFind::onResponseReceived(const Response::SharedPtr& response)
{
  // I dont know how to properly check if the response is success full, so right now im just assuming that the
  // timestamp would 0 if its not found.
  geometry_msgs::msg::PoseStamped res = response.get()->centroid_pose;
  if (res.header.stamp.sec == 0 && res.header.stamp.nanosec == 0)
  {
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    setOutput("centroid_pose", res);
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus ColorBlobFind::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace color_tools_behavior
