#include <memory>
#include <string>
#include <array>
#include <iostream>
#include <boost/algorithm/string/find.hpp>
#include <regex>
#include <unistd.h>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
using namespace std::chrono;
using namespace boost;

struct PoseData
{
  std::string name;
  int id;
  double positionX;
  double positionY;
  double positionZ;
  double orientationX;
  double orientationY;
  double orientationZ;
  double orientationW;
};

class RespawnRobot : public rclcpp::Node
{
  public:
    RespawnRobot()
    : Node("robot_respawner")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/respawn_robot", 10, std::bind(&RespawnRobot::topic_callback, this, _1));
      entity_name_ = "robot";
    }

  private:  
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {      
      if (!msg->data)
      {
        // Execute the command if the received boolean is false
        executeCommand();
      }
    }

    void executeCommand()
    {
      std::string command = "ign topic -t /world/simple_world/pose/info -e -n 1";
      std::array<char, 128> buffer;
      std::string output;
      std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
      
      if (pipe)
      {
        // Read the command's output into the result string
        while (!feof(pipe.get()))
        {
          if (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
          {
            output += buffer.data();
          }
        }
      }

      PoseData poseData = extractPoseData(output, entity_name_);


      if (poseData.id != -1)
      {
        // Calculate yaw angle
        double yaw_half = 0.5*calculateYawFromQuaternion(poseData.orientationW, poseData.orientationX, poseData.orientationY, poseData.orientationZ);

        // poseData.positionZ = 0.0;
        poseData.orientationX = 0.0;
        poseData.orientationY = 0.0;
        poseData.orientationZ = std::sin(yaw_half);
        poseData.orientationW = std::cos(yaw_half);
        
        command = "ign service -s /world/simple_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 300 --req 'name: \"" + entity_name_ +
                      "\", orientation: {x: " + std::to_string(poseData.orientationX) +
                      ", y: " + std::to_string(poseData.orientationY) +
                      ", z: " + std::to_string(poseData.orientationZ) +
                      ", w: " + std::to_string(poseData.orientationW) +
                      "}, position: {x: " + std::to_string(poseData.positionX) +
                      ", y: " + std::to_string(poseData.positionY) +
                      ", z: " + std::to_string(poseData.positionZ) + "}'";
        std::system(command.c_str());

        // RCLCPP_INFO(get_logger(), "\nName:%s, ID: %d\nPosition (x, y, z): (%f, %f, %f)\nOrientation (x, y, z, w): (%f, %f, %f, %f)",
        //   poseData.name.c_str(), poseData.id, poseData.positionX, poseData.positionY, poseData.positionZ,
        //   poseData.orientationX, poseData.orientationY, poseData.orientationZ, poseData.orientationW);
      }
    }

    PoseData extractPoseData(const std::string& output, const std::string& entityName)
    {
      PoseData poseData;

      poseData.id = -1;
      // Find the entity's name in the pose substring
      std::size_t nameStartPos = output.find("name: \"" + entityName + "\"");
      // Extract the substring containing the entity's pose
      std::string poseSubstring = output.substr(nameStartPos);
      iterator_range<std::string::iterator> endPos = find_nth(poseSubstring, "}", 2);
      poseSubstring = poseSubstring.substr(0, std::distance(poseSubstring.begin(), endPos.begin()));
      // RCLCPP_INFO(get_logger(), "\nSubstring:%s",poseSubstring.c_str());

      std::stringstream ss(poseSubstring);
      std::string line;

      while (std::getline(ss, line))
      {
        if (line.find("name:") != std::string::npos)
        {
          poseData.name = extractStringValue(line);
          // RCLCPP_INFO(get_logger(), "\nName:%s",poseData.name.c_str());
        }
        else if (line.find("id:") != std::string::npos)
        {
          poseData.id = std::stoi(line.substr(line.find(":") + 1));
          // RCLCPP_INFO(get_logger(), "\nID:%d",poseData.id);
        }
        else if (line.find("position {") != std::string::npos)
        {
          while (line.find("}") == std::string::npos)
          {
            std::getline(ss, line);
            if (line.find("x:") != std::string::npos)
            {
              poseData.positionX = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("y:") != std::string::npos)
            {
              poseData.positionY = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("z:") != std::string::npos)
            {
              poseData.positionZ = std::stod(line.substr(line.find(":") + 1));
            }
          }
        }
        else if (line.find("orientation {") != std::string::npos)
        {
          while (line.find("}") == std::string::npos)
          {
            std::getline(ss, line);
            if (line.find("x:") != std::string::npos)
            {
              poseData.orientationX = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("y:") != std::string::npos)
            {
              poseData.orientationY = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("z:") != std::string::npos)
            {
              poseData.orientationZ = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("w:") != std::string::npos)
            {
              poseData.orientationW = std::stod(line.substr(line.find(":") + 1));
            }
            sleep(1);
          }
          if (line.find("}") != std::string::npos)
          {
            break;
          }
        }
      }

      return poseData;
    }

    std::string extractStringValue(const std::string& line)
    {
      std::regex regex("\"([^\"]*)\"");
      std::smatch match;

      if (std::regex_search(line, match, regex) && match.size() > 1)
      {
        return match.str(1);
      }

      return "";
    }

    double calculateYawFromQuaternion(double w, double i, double j, double k) 
    {
      // Ensure quaternion is normalized
      double magnitude = std::sqrt(w * w + i * i + j * j + k * k);
      w /= magnitude;
      i /= magnitude;
      j /= magnitude;
      k /= magnitude;

      // Calculate yaw angle
      double yaw = std::atan2(2 * (w * k + i * j), 1 - 2 * (j * j + k * k));

      return yaw;
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    std::string entity_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RespawnRobot>());
  rclcpp::shutdown();
  return 0;
}
