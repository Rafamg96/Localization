/*
 * particle_filter.h
 *
 * 2D particle filter class.
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_



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
	
	// Número de particulas
	int num_particles; 

	// Boleano para controlar si el filtro de particulas ha sido inicializado
	bool is_initialized;


	// Vector sobre los pesos de las particulas
	std::vector<double> weights;
	
public:
	
	// Conjunto de particulas
	std::vector<Particle> particles;

	// Constructor
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	

	// Destructor
	~ParticleFilter() {}


	void init(double x, double y, double theta, double std[]);

	void prediction(double diferenciax,double diferenciay,double diferenciatheta, double std_pos[]);
	void updateWeights();
	void resample();
	void write(std::string filename, float x,float y,float theta);
	void writetiempo(std::string filename, float x);
	void writeposicion(std::string filename, float posrealx,float posrealy,float posrealtheta,  float posodometriax,float posodometriay, float posodometriatheta, float pospfx,float pospfy, float pospftheta);

	
	bool initialized();
};
void callbackPosicionReal (const nav_msgs::Odometry input);
void callbackObtenerNubeMapeo(const sensor_msgs::PointCloud2& input);
void callbackObtenerNubeSensor(const sensor_msgs::PointCloud2& input);
void callbackobtenererrorenodom (const nav_msgs::Odometry input);
void obtenerImagenSensor(cv::Mat &imagen);
void obtenerImagenParticula(float x, float y, float angle, cv::Mat &imagen);
float registrarImagen(cv::Mat imagensensor,cv::Mat imagenparticula);
float histogramaImagen(cv::Mat imagensensor,cv::Mat imagenparticula);
#endif /* PARTICLE_FILTER_H_ */
