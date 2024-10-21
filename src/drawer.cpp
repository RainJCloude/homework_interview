#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>


#include <chrono>

class FrameDrawer_on_image : public rclcpp::Node {
public:
    explicit FrameDrawer_on_image(): 
        Node("drawer"), node_handle_(std::shared_ptr<FrameDrawer_on_image>(this)),
            it_(node_handle_)
    {

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); //connect tf2_listner to the /tf topic

        // Subscriber to the camera image published by the python node
        /*image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_input", 10,
            std::bind(&FrameDrawer_on_image::imageSubscriber, this, std::placeholders::_1)
        );*/
        
        sub_ = it_.subscribeCamera("camera/Image", 10,
            std::bind(&FrameDrawer_on_image::cameraSubscriber, this, std::placeholders::_1, std::placeholders::_2));

    }

    void cameraSubscriber(const sensor_msgs::msg::Image::ConstSharedPtr& raw_image,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info){
                            
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;
        input_bridge = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8); //from ros_msg to cv
        image = input_bridge->image;
        cam_model_.fromCameraInfo(cam_info);

        //Tranfor the coordinates
        geometry_msgs::msg::TransformStamped t;

        auto acquisition_time = cam_info->header.stamp;
        rclcpp::Duration timeout(std::chrono::milliseconds(33)); // Roughly 30 Hz
            
        // Wait for the transform
        RCLCPP_INFO(this->get_logger(), "Published camera info: %s", cam_info->header.frame_id.c_str());
         
        tf_buffer_->canTransform("rotating_frame", cam_info->header.frame_id, acquisition_time, timeout);
        t = tf_buffer_->lookupTransform("rotating_frame", cam_info->header.frame_id, acquisition_time);

        //t = tf_buffer_->lookupTransform("rotating_frame", "camera_optical_frame", tf2::TimePointZero);
        tf2::Vector3 pt(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        tf2::Matrix3x3 m(q);

        //CLCPP_INFO(rclcpp::get_logger("matrix_logger"));
        //Set the point in the image
        cv::Point3d pt_cv(pt.x(), pt.y(), pt.z()); // Create OpenCV Point3d
        cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

        static const int RADIUS = 3;
        cv::circle(image, uv, RADIUS, cv::Scalar(255, 0, 0), cv::FILLED);
        /*CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                                 uv.y - RADIUS - baseline - 3);
        cv:putText(image, "camera_optical_frame", uv, cv::FONT_HERSHEY_SIMPLEX, 12, CV_RGB(255,0,0));*/

        cv::imshow("Projected Image", image);
        cv::waitKey(1);
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    /*tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;*/
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    image_geometry::PinholeCameraModel cam_model_;
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::CameraSubscriber sub_; //be aware with image_transport::Subscriber
    image_transport::Subscriber sub_img_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameDrawer_on_image>());
    rclcpp::shutdown();
    return 0;
}
