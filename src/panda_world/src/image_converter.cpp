#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class DepthConverterNode : public rclcpp::Node
{
public:
    DepthConverterNode() : Node("depth_converter_node")
    {
        // --- Parameters ---
        // Factor to convert float (meters) to integer (millimeters).
        this->declare_parameter<double>("scale_factor", 1000.0);
        scale_factor_ = this->get_parameter("scale_factor").as_double();

        // --- QoS Profiles ---
        // Use a reliable profile for this critical data link.
        // Queue depth of 1 ensures we always process the latest image.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

        // --- Publisher ---
        // Publishes the converted 16UC1 image.
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image_converted", qos);
        RCLCPP_INFO(this->get_logger(), "Publishing converted depth on topic 'depth/image_converted'");

        // --- Subscriber ---
        // Subscribes to the original 32FC1 image from the simulator.
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "depth/image_raw", // <-- Topic from Gazebo/Simulator
            qos,
            std::bind(&DepthConverterNode::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to raw depth on topic 'depth/image_raw'");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Ensure the input encoding is what we expect (32-bit float).
        if (msg->encoding != "32FC1")
        {
            RCLCPP_WARN_ONCE(
                this->get_logger(),
                "Received image with encoding '%s', but expected '32FC1'. Skipping.",
                msg->encoding.c_str()
            );
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // Convert ROS message to OpenCV Mat. No copy is made if possible.
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // --- The Conversion Logic ---
        cv::Mat float_image = cv_ptr->image;
        cv::Mat uint16_image(float_image.size(), CV_16UC1);

        // 1. Sanitize for NaN and infinity, which are common in simulator depth images.
        //    Replace them with 0 (representing invalid depth).
        cv::patchNaNs(float_image, 0.0);
        
        // 2. Perform the conversion: (meters * scale_factor) -> millimeters
        //    The conversion handles clipping values outside the 16-bit range.
        float_image.convertTo(uint16_image, CV_16UC1, scale_factor_);
        
        // --- Republish the Converted Image ---
        // Create a new CvImage containing our converted Mat.
        // The header is copied from the original message to preserve timestamp and frame_id.
        cv_bridge::CvImage out_msg(msg->header, "16UC1", uint16_image);
        
        // Convert back to a ROS message and publish.
        pub_->publish(*out_msg.toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    double scale_factor_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthConverterNode>());
    rclcpp::shutdown();
    return 0;
}
