#include <color_tools_behavior/detect_color_blobs.hpp>

namespace
{
inline constexpr auto kDescriptionDetectColorBlobs = R"(
                <p>
                    Given a synchronized RGB/Depth/Camera info data, returns a list of color blob centroids.
                </p>
            )";
constexpr auto kPortRgbImage = "rgb_image";
constexpr auto kPortDepthImage = "depth_image";
constexpr auto kPortCameraInfo = "camera_info";
constexpr auto kPortDesiredBlob = "desired_blob";
constexpr auto kPortMinBlobSize = "min_blob_size";
constexpr auto kPortBlobColor = "blob_color";
constexpr auto kPortBlobPose = "blob_pose";
constexpr auto kPortMaskedColorImage = "mask_image";

}  // namespace

namespace color_tools_behavior
{
DetectColorBlobs::DetectColorBlobs(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ThreadedAction(name, config)

{
}

BT::PortsList DetectColorBlobs::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::Image>(kPortRgbImage, "Input RGB image"),
    BT::InputPort<sensor_msgs::msg::Image>(kPortDepthImage,
                                           "Input depth image (will be converted to meters if needed)"),
    BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortCameraInfo, "Camera intrinsic parameters"),
    BT::InputPort<u_int8_t>(kPortDesiredBlob, 0, "Desired Blob index. Default is 0"),
    BT::InputPort<double>(kPortMinBlobSize, 10.0, "Minimum size of width or height of object in pixels"),
    BT::InputPort<std::string>(kPortBlobColor, "red", "Color of the mask to use"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortBlobPose, "Detected blob pose"),
    BT::OutputPort<sensor_msgs::msg::Image>(kPortMaskedColorImage,
                                            "Computed masks of the color blobs overlaid on the image"),
  };
}

BT::NodeStatus DetectColorBlobs::tick()
{
  if (!getInput<sensor_msgs::msg::Image>(kPortRgbImage, blob_request.color_img))
  {
    throw BT::RuntimeError("Could not access blackboard input [base_frame]");
  }
  if (!getInput<sensor_msgs::msg::Image>(kPortDepthImage, blob_request.depth_img))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }
  if (!getInput<sensor_msgs::msg::CameraInfo>(kPortCameraInfo, blob_request.camera_info))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }
  if (!getInput<u_int8_t>(kPortDesiredBlob, blob_request.desired_blob))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }
  if (!getInput<double>(kPortMinBlobSize, blob_request.min_blob_size))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }
  if (!getInput<std::string>(kPortBlobColor, blob_request.blob_color))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }

  const auto result = color_blob_centroid::processBlobs(blob_request);
  if (!result.success)
  {
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortBlobPose, result.centroid_pose);
  setOutput(kPortMaskedColorImage, result.color_img);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace color_tools_behavior
