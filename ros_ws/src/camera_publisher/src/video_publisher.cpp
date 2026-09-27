#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image/compressed", 10);
        
        // 初始化 SDK
        unitree::robot::ChannelFactory::Instance()->Init(0);
        video_client_.SetTimeout(1.0f);
        video_client_.Init();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&VideoPublisher::publish_frame, this));
    }

private:
    void publish_frame() {
        std::vector<uint8_t> image_data;
        if (video_client_.GetImageSample(image_data) == 0) {
            auto msg = sensor_msgs::msg::CompressedImage();
            msg.header.stamp = this->now();
            msg.format = "jpeg"; // 告诉 ROS 这是 JPEG 格式
            msg.data = image_data; // 直接把二进制塞进去

            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    unitree::robot::go2::VideoClient video_client_;
};

int main(int argc, char *argv[]) {
    unitree::robot::ChannelFactory::Instance()->Init(0, "wlan0");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
