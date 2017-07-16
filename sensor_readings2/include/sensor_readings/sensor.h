#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>

// TF
#include <tf/transform_listener.h>

class Sensor {
public:
    std::string topic_;
    
private:
	pcl::PointCloud<pcl::PointXYZ> sensor_readings_;
	
	ros::Time last_update_;
	
protected:
	tf::TransformListener tf_listener_;
	
	float min_distance_, max_distance_;
	
	int num_points_;


public:
	Sensor(std::string topic);

    Sensor(std::string topic, float min_distance, float max_distance, int num_points);
    
    // Copy constructor
    Sensor(const Sensor & other);
    
    // Copy constructor
    Sensor(Sensor & other);
        
    /**
     * @brief timeSinceLastUpdate
     * @return Time in seconds since last update
     */
    float timeSinceLastUpdate();
    
    // Maybe protected?
    void updateSensorReadings(pcl::PointCloud<pcl::PointXYZ> & updated_sensor_readings);
    
    /**
     * @brief getDistances
     * @return Distances
     */
    pcl::PointCloud<pcl::PointXYZ> getSensorReadings();
};


#endif // SENSOR_H
