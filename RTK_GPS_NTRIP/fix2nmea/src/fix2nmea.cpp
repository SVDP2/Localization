#include <chrono>
#include <cmath>
#include <string>

#include "nmea_msgs/msg/sentence.h"
#include "sensor_msgs/msg/nav_sat_fix.h"
#include "ublox_msgs/msg/nav_posllh.h"

#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_msgs/msg/nav_posllh.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

using std::placeholders::_1;

class Transformer : public rclcpp::Node
{
 public:
  Transformer() 
  : Node("fix2nmea")
  {
    this->declare_parameter<std::string>("fix_topic", "/follower/f9r/fix");
    this->declare_parameter<std::string>("nmea_topic", "/follower/ntrip_client/nmea");

    const auto fix_topic = this->get_parameter("fix_topic").as_string();
    const auto nmea_topic = this->get_parameter("nmea_topic").as_string();

	  nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>(nmea_topic, 10);  
    navsatfix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&Transformer::receiveNavSatFix, this, _1));

    RCLCPP_INFO(
      this->get_logger(),
      "fix2nmea started: fix_topic=%s, nmea_topic=%s",
      fix_topic.c_str(),
      nmea_topic.c_str());
  }
  
  private:
  
    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_sub_;
    //rclcpp::Subscription<ublox_msgs::msg::NavPOSLLH>::SharedPtr navhpposllh_sub_;

    void receiveNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg)
    {
  	//RCLCPP_INFO_ONCE("Received first NavSatFix message.");
  	// Conversion inspired from: https://answers.ros.org/question/377844/converting-sensors_msgsnavsatfix-to-nmea_msgssentence-messages/

  	char buf[255]; // Buffer for GPGGA sentence

    if (!std::isfinite(navsat_msg->latitude) ||
        !std::isfinite(navsat_msg->longitude) ||
        !std::isfinite(navsat_msg->altitude)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "dropping NavSatFix to NMEA conversion: non-finite LLA");
      return;
    }

  	// Time conversion
  	//auto time = navsat_msg->header.stamp.toBoost().time_of_day();	
  	boost::posix_time::ptime t = boost::posix_time::second_clock::universal_time();
  	auto time = t.time_of_day();
  	long int centiseconds = (navsat_msg->header.stamp.nanosec / 10000000) % 100;

  	// Latitude conversion
  	char lat_dir = navsat_msg->latitude < 0.0 ? 'S' : 'N';
  	double abs_lat = std::fabs(navsat_msg->latitude);
  	int lat_degs = static_cast<int>(abs_lat);
  	double lat_mins = (abs_lat - static_cast<double>(lat_degs)) * 60.0;

  	// Longitude conversion
  	char lon_dir = navsat_msg->longitude < 0.0 ? 'W' : 'E';
  	double abs_lon = std::fabs(navsat_msg->longitude);
  	int lon_degs = static_cast<int>(abs_lon);
  	double lon_mins = (abs_lon - static_cast<double>(lon_degs)) * 60.0;

  	// Status conversion
  	int status = navsat_msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX ? 1 : 0;

  	// Minimum number of satellites is service times 4.
  	int num_satellites = 0;
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_GPS) == sensor_msgs::msg::NavSatStatus::SERVICE_GPS) {
    		num_satellites += 4;
  	}
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS)
      	== sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS) {
    		num_satellites += 4;
  	}
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS)
      	== sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS) {
    		num_satellites += 4;
  	}
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO)
      	== sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO) {
    		num_satellites += 4;
  	}

	// Longitude conversion
  	double alt_msl = navsat_msg->altitude;
  	alt_msl = alt_msl - 1.2; 
  	
	//double alt_geoid = 1.2;
	
  	int len = snprintf(buf,
                    sizeof(buf),
                    "$GNGGA,%02ld%02ld%02ld.%02ld,%02d%010.7f,%c,%03d%010.7f,%c,%d,%d,0.63,%0.3f,M,1.2,M,",
                    time.hours(),
                    time.minutes(),
                    time.seconds(),
                    centiseconds,
                    lat_degs,
                    lat_mins,
                    lat_dir,
                    lon_degs,
                    lon_mins,
                    lon_dir,
                    status,
                    num_satellites,
                    alt_msl
                    //alt_geoid
                    );
    if (len <= 0 || static_cast<size_t>(len) >= sizeof(buf)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "dropping NavSatFix to NMEA conversion: sentence formatting failed");
      return;
    }

	// Calculate checksum of sentence and add it to the end of the sentence
	uint8_t checksum = 0;
  	for(int i = 1; i < len; i++)
  	{
    	checksum ^= buf[i];
  	}
  	snprintf(&buf[len], sizeof(buf) - static_cast<size_t>(len), "*%02X\r\n",checksum);
  
  	nmea_msgs::msg::Sentence nmea_msg;
  	nmea_msg.header = navsat_msg->header;
  	nmea_msg.sentence = buf;
  	
  	nmea_pub_->publish(nmea_msg);
    }
    
    
    

};



int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transformer>());
    rclcpp::shutdown();
    
    return 0;
  }
