#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fstream>

class PathSaver : public rclcpp::Node {

  // Constructor for the PathSaver node: initializes the ROS2 node named "path_saver",
  // sets up the TF2 buffer and listener to receive transform data,
  // creates a periodic timer that calls on_timer() every 100 milliseconds to process and save the VC's pose,
  // and opens a CSV file "vc_path.csv" with a header to log the vacuum cleaner's path over time (time, x, y, z).

public:
  PathSaver() : Node("path_saver"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&PathSaver::on_timer, this));
    csv_.open("vc_path.csv");
    csv_ << "time,x,y,z\n";
  }

private:
  void on_timer() {
    try {

      /* Those two lines get the positions of the charging station and the vacuum cleaner in the floor's frame,
         so you can compute the vacuum cleaner's path and distance relative to the CS, which is exactly what the exercise asks for.
      */

      // Use tf2::TimePointZero to get the latest available transform
      // This works better when playing a bag - it gets the most recent transform in the buffer
      // Transform from floor (tag36h11:0) to charging station (tag36h11:1) - It tells you where the CS is located in the floor's coordinate system.
      auto t0_1 = tf_buffer_.lookupTransform("tag36h11:0", "tag36h11:1", tf2::TimePointZero);

      // Print charging station location once
      if (!cs_location_printed_) {
        auto& trans = t0_1.transform.translation;
        auto& rot = t0_1.transform.rotation;
        RCLCPP_INFO(this->get_logger(), "\n=== Charging Station Location ===");
        RCLCPP_INFO(this->get_logger(), "Frame: tag36h11:1 (Charging Station)");
        RCLCPP_INFO(this->get_logger(), "Reference Frame: tag36h11:0 (Floor)");
        RCLCPP_INFO(this->get_logger(), "Position (x, y, z): %.4f, %.4f, %.4f", trans.x, trans.y, trans.z);
        RCLCPP_INFO(this->get_logger(), "Orientation (x, y, z, w): %.4f, %.4f, %.4f, %.4f", rot.x, rot.y, rot.z, rot.w);
        RCLCPP_INFO(this->get_logger(), "==================================\n");
        cs_location_printed_ = true;
      }

      // Transform from floor (tag36h11:0) to vacuum cleaner (tag36h11:2) - It tells you where the VC is located in the same floor reference.
      auto t0_2 = tf_buffer_.lookupTransform("tag36h11:0", "tag36h11:2", tf2::TimePointZero);

      // Convert to tf2
      /*
      Those lines convert the ROS messages into math objects,
      so you can compute where the vacuum cleaner is relative to the charging station.
      */
      tf2::Transform T0_1, T0_2;
      tf2::fromMsg(t0_1.transform, T0_1);
      tf2::fromMsg(t0_2.transform, T0_2);

      // Compute VC pose relative to CS: T_1_2 = inverse(T0_1) * T0_2
      tf2::Transform T1_2 = T0_1.inverseTimes(T0_2);
      tf2::Vector3 pos = T1_2.getOrigin();

      // Project on the plane of z axis (set z=0 for floor plane projection)
      // Use the timestamp from the transform for accurate time tracking
      // Convert timestamp to seconds since epoch
      double t = t0_2.header.stamp.sec + t0_2.header.stamp.nanosec * 1e-9;
      csv_ << t << "," << pos.x() << "," << pos.y() << ",0\n";
      csv_.flush();  // Ensure data is written immediately

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform error: %s", ex.what());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream csv_;
  bool cs_location_printed_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSaver>());
  rclcpp::shutdown();
  return 0;
}
