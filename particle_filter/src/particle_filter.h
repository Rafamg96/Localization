/*
 * particle_filter.h
 *
 * 2D particle filter class.
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"


#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
//NUEVOS INCLUDES
#include <ros/ros.h>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <iostream>
#include <array>

#include <gazebo_msgs/SpawnModel.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/crop_box.h>
#include <pcl/range_image/range_image.h>

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
};



class ParticleFilter {
	
	// Number of particles to draw
	int num_particles; 

	// Flag, if filter is initialized
	bool is_initialized;


	// Vector of weights of all particles
	std::vector<double> weights;
	
public:
	
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param M Number of particles
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double diferenciax,double diferenciay,double diferenciatheta, double std_pos[]);
	
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	 *   standard deviation of bearing [rad]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr nubemapeo, pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubesensor);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();
	
	/*
	 * write Writes particle positions to a file.
	 * @param filename File to write particle positions to.
	 */
	void write(std::string filename, float x,float y,float theta);
	
	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	bool initialized();
};
void callbackObtenerNubeMapeo(const sensor_msgs::PointCloud2& input);
void callbackObtenerNubeSensor(const sensor_msgs::PointCloud2& input);
void callbackobtenerPosicionyVelocidad (const nav_msgs::Odometry input);
cv::Mat obtenerImagenSensor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubesensor);
cv::Mat obtenerImagenParticula(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr nubemapeo,float x, float y, float angle);
float registrarImagen(cv::Mat imagensensor,cv::Mat imagenparticula);
float histogramaImagen(cv::Mat imagensensor,cv::Mat imagenparticula);
float calcularpeso1(cv::Mat imagensensor,cv::Mat imagenparticula);
#endif /* PARTICLE_FILTER_H_ */
