/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "particle_filter.h"


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
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2/LinearMath/Quaternion.h>

#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;



//Definimos las variables generales del programa

int semilla=7;

//Set up parameters here
double delta_t = 0.004; // Tiempo entre cada medida

//Declaramos un booleano para sincronizar
bool odometryobtenida=false;

float posicionx=0;
float posiciony=0;
float posiciontheta=0;
float velocidadlineal=0;
float velocidadangular=0;
pcl::PointCloud<pcl::PointXYZRGB> nubesensor;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

image_transport::Publisher pubsensor;
image_transport::Publisher pubmapeo;
ros::Publisher pubpointcloud;
ros::Publisher pubpointcloud2;
ros::Publisher pubparticulas;
ros::Publisher pubmap;
int numeroparticulas=1;
using namespace std;
//Asignamos la semilla 7 para los aleatorios

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	//ROS_INFO("FASE INICIALIZACIÓN");
    num_particles = numeroparticulas; //set to number of files in observation directory

    weights.resize(num_particles);
    particles.resize(num_particles);

    // Creamos una distribucción normal para añadir obtener las nuevas particulas
    normal_distribution<double> dist_x(x, std[0]); 
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    //Creamos un generador gaussiano
    default_random_engine gen;
    gen.seed(7);
    

    for(int i=0; i<num_particles; i++){
        Particle p;
        p.id = i;
        p.x = dist_x(gen); 
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;



        particles[i] = p;
        weights[i] = p.weight;
	//ROS_INFO("Particula %d -> x=%f y=%f theta=%f",i,p.x,p.y,p.theta);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	//ROS_INFO("FASE PREDICION");

	default_random_engine gen;
	gen.seed(7);
    for(int i=0; i<num_particles; i++){
        Particle *p = &particles[i]; // get address of particle to update
	//ROS_INFO(" Antes x-> %f  y-> %f angulo-> %f",p->x, p->y, p->theta);
        // Predecimos la posición basandonos en la ecuación básica de e=v*t
        double new_x = p->x + velocity*delta_t * cos(p->theta);
        double new_y = p->y + velocity*delta_t * sin(p->theta);
        double new_theta = p->theta + (yaw_rate*delta_t);

	//Predicción encontrada en otros algoritmos que calculan la posición
	//float new_x = p->x + (velocity/yaw_rate) * (sin(p->theta + yaw_rate*delta_t) - sin(p->theta));
    //float new_y = p->y + (velocity/yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate*delta_t));
    //float new_theta = p->theta + (yaw_rate*delta_t);
	
	//Añadimos ruido gaussiano a la predición para que los resamples no utilicen siempre la misma particula
	
	normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);

	new_x=dist_x(gen);
	new_y=dist_y(gen);
	new_theta=dist_theta(gen);

	//ROS_INFO(" v->%f, yaw_rate->%f \n Antes x-> %f  y-> %f angulo-> %f \n Predicción x->%f y->%f theta->%f",velocity,yaw_rate,p->x, p->y, p->theta, new_x,new_y,new_theta);

	// update the particle attributes
        p->x = new_x;
        p->y = new_y;
        p->theta = new_theta;

    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for(auto pred : predicted){
      double dist_min = std::numeric_limits<double>::max();
      for(auto observation : observations){
        double distance = dist(observation.x, observation.y, pred.x, pred.y); // distance b/w obs and landmark
        if(distance < dist_min){
          observation.id = pred.id;
        }
        dist_min = distance;
      }
    }
}

void ParticleFilter::updateWeights(pcl::PointCloud<pcl::PointXYZRGB> nubemapeo, pcl::PointCloud<pcl::PointXYZRGB> nubesensor) {
	//ROS_INFO("FASE ACTUALIZACION DE PESOS");

	//Sumatorio de pesos para luego normalizar
    double weights_sum = 0;
    //Float para almacenar la similitud entre particula y lo que obtiene el sensor
    float coincidencia;
	//Obtenemos la imagen de nubesensor
	cv::Mat imagensensor=obtenerImagenSensor(nubesensor);

    for(int j=0; j<num_particles; j++){
		//Cogemos cada una de las particulas
        Particle *p = &particles[j];
		//Calculamos la imagen que se obtendría por la particula con la nubemapeo
		ROS_INFO("Antes de obtener imagen particula"); 
		cv::Mat imagenparticula=obtenerImagenParticula(nubemapeo,p->x,p->y,p->theta);
		ROS_INFO("Después de obtener imagen particula"); 
		//Calculamos cual es el peso correspondiente a la particula
		//HACER
		coincidencia=registrarImagen(imagensensor,imagenparticula);
		ROS_INFO("Después de registrar imagen"); 
		//coincidencia=registrarImagen(imagenparticula,imagenparticula);
		//Calculamos cual sería el mayor error entre lo que nos dicen los sensores y lo que podríamos tener en las particulas.
		
		weights_sum += coincidencia;
		p->weight = coincidencia;
		//ROS_INFO(" POS particula %d : %f",j,p->x);
    }
    
    // Normalizamos los pesos
    for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
	//ROS_INFO(" Peso particula %d : %f",i,p->weight);
    }
}
//Método resample wheel
void ParticleFilter::resample() {
    //ROS_INFO("FASE RESAMPLE WHEEL");
    //Declaramos un vector de particulas auxiliar donde iremos almacenando el conjunto de particulas que meteremos en nuestro conjunto de particulas
    std::vector<Particle> resampled_particles;
    //Elegimos un indice aleatoriamente
    int indice=rand()%num_particles;
    //ROS_INFO("Indice inicial: %d", indice);
    //Calculamos una beta que será la encargada de elegir que particulas irán a nuestro conjunto de particulas
    float beta=0;
    //Beta irá incrementando su valor un número aleatorio entre 0 y el doble del peso máximo de las particulas
    //Calculamos el máximo peso de las particulas
    float pesomaximo=0;
    for(int i=0;i<num_particles;i++){
    	if(pesomaximo<weights[i]){
		pesomaximo=weights[i];	
	}
    }
    //Debemos obtener un conjunto de particulas igual al número de particulas
    for (int i = 0; i < num_particles; i++){
	//Incrementamos beta
	beta=(float)((rand())/(float)RAND_MAX)*(pesomaximo*2);
	 //ROS_INFO("Beta: %f", beta);
	//Si beta es menor que el peso de la particula cuyo indice fue seleccionado
	//Si no es menor se tomara el peso de diferencia y se comparará con la siguiente partícula comprobándose de nuevo si el peso de diferencia es mayor que el peso de la partícula siguiente
	while(beta>weights[indice]){
		beta-=weights[indice];
		//Añadimos uno al indice y comprobamos que el indice no se pasa del numero de indices totales
		indice++;
		indice=indice%num_particles;
	}
	//Se coge dicha particula para el conjunto de particulas
	resampled_particles.push_back(particles[indice]);
	//ROS_INFO("Indice elegido: %d", indice);
    }
    //Se sustituye el conjunto de particulas por el resampled
    particles = resampled_particles;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}

void general(){
	
	pcl::PointCloud<pcl::PointXYZRGB> nubemapeo=*cloud;
	srand(semilla);
	//double sigma_pos [3] = {0.03, 0.03, 0.01}; // Posible error máximo del GPS [x [m], y [m], theta [rad]]
	//double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

	double sigma_pos [3] = {0, 0, 0}; // Posible error máximo del GPS [x [m], y [m], theta [rad]]
	double sigma_landmark [2] = {0, 0}; // Landmark measurement uncertainty [x [m], y [m]]


	//Para generar los diferentes posibles errores(ruido) del gps por ejemplo
	default_random_engine gen;
	//Asignamos una semilla para que sea reproducible
	gen.seed(7);
	//Distribuciones normales con media 0 y varianza la declarada anteriormente
	normal_distribution<double> N_x_init(0, sigma_pos[0]);
	normal_distribution<double> N_y_init(0, sigma_pos[1]);
	normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
	normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
	double n_x, n_y, n_theta, n_range, n_heading;

	//Declaramos el filtro de particulas
	ParticleFilter pf;
	double total_error[3] = {0,0,0};
	double cum_mean_error[3] = {0,0,0};
	//double total_error[3] = {0.3,0.3,0.3};
	//double cum_mean_error[3] = {0.3,0.3,0.3};

	//UN nuevo reloj para comprobar cuanto tarda para llegar a la proxima iteracción y calcular delta
	int start2 = clock();
	//cout << "Time step: " << i << endl;
	// Inicializamos el filtro de particulas, en el caso de que sea la primera iteración.
	if (!pf.initialized()) {
		//Generamos un aleatorio de la distribución N_x_init
		n_x = N_x_init(gen);
		//Generamos un aleatorio de la distribución N_y_init
		n_y = N_y_init(gen);
		//Generamos un aleatorio de la distribución N_theta_init
		n_theta = N_theta_init(gen);
		//Inicializamos el filtro de particulas con la posición global del robot  más un pequeño error generado aleatoriamente n_x,z,theta.
		pf.init(posicionx + n_x, posiciony + n_y, posiciontheta + n_theta, sigma_pos);
		ROS_INFO("Fin inicialización"); 
	}
	//En caso de que ya haya sido inicializado el filtro de particulas
	else {
		// Se predice la posición del vehiculo en el proximo instante de tiempo. Pasandole el tiempo que pasará hasta la proxima medida, los posibles errores, la velocidad lineal y angular que tenian en el anterior instante
		pf.prediction(delta_t, sigma_pos, velocidadlineal, velocidadangular);
		ROS_INFO("Fin predicción"); 
	}
	//Calculamos el valor de delta
	int stop2 = clock();
	double runtime2 = (stop2 - start2) / double(CLOCKS_PER_SEC);
	//cout << "Runtime (sec): " << runtime2 << endl;

	// Se actualizan los pesos y se realiza el resample añadiendo ruido
	//Aqui no se si nubesensor esta actualizada, porque ha pasado un tiempo ARREGLAR
	pf.updateWeights(nubemapeo, nubesensor);
	ROS_INFO("Fin updateweights"); 
	pf.resample();
	ROS_INFO("Fin resample"); 
	 //Publicamos las particulas
	geometry_msgs::PoseArray particulas;
	geometry_msgs::Pose particula;
	particulas.header.frame_id = "map";
    particulas.header.stamp = ros::Time::now();
	
	for (int i=0;i<pf.particles.size();i++){
	 	particula.position.x=pf.particles[i].x;
	 	particula.position.y=pf.particles[i].y;
	 	particula.position.z=0;
		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY( 0, 0, pf.particles[i].theta ); 
		particula.orientation.x=myQuaternion[0];
		particula.orientation.y=myQuaternion[1];
		particula.orientation.z=myQuaternion[2];
		particula.orientation.w=myQuaternion[3];
	 	particulas.poses.push_back(particula);

	 }
	 //Publicamos las particulas
     pubparticulas.publish(particulas); 
	ROS_INFO("Particulas publicadas"); 
	// Calculamos el error medio del filtro de particulas 
	vector<Particle> particles = pf.particles;
	int num_particles = particles.size();
	double highest_weight = 0.0;
	Particle best_particle;
	//Se calcula cual es la mejor particula
	for (int i = 0; i < num_particles; ++i) {
		if (particles[i].weight > highest_weight) {
			highest_weight = particles[i].weight;
			best_particle = particles[i];
		}
	}

}
//Main de el filtro de particulas
int main(int argc, char **argv){
	// Iniciamos un reloj para saber el tiempo que tarda en realizarse
	int start = clock();
  	ros::init(argc, argv, "particle_filter");

  	ros::NodeHandle n;
	//Declaramos los subscribers
	ros::Subscriber subscriberodom; //Para la posicion y velocidad
	ros::Subscriber subnubemapeo; //Para obtener la nube de puntos del mapeo
	ros::Subscriber subnubesensor;  //Para obtener la nube de puntos del sensor

	//Esto se puede borrar despues, es para ver si las imagenes salen bien
	image_transport::ImageTransport it(n);
	pubsensor = it.advertise("/imagensensor",1); 
	pubmapeo = it.advertise("/imagenparticula",1); 
	pubpointcloud= n.advertise<sensor_msgs::PointCloud2>("cloud_prueba", 1);
	pubpointcloud2= n.advertise<sensor_msgs::PointCloud2>("cloud_prueba2", 1);
    pubparticulas=n.advertise<geometry_msgs::PoseArray>("particulas",1);
	pubmap = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("map_cloud", 1, true);
	ros::Duration(0.5).sleep();

	//Obtenemos la nube de puntos del mapeo, podriamos en vez de suscribirmos obtenerla de un archivo y aceleramos calculos
	//subnubemapeo = n.subscribe("/cloud_pcd",1,callbackObtenerNubeMapeo);

	//Obtenemos la nube de puntos del mapeo
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/rafael/catkin_ws/pcd/prueba2.pcd", *cloud);
	//nubemapeo=*cloud;
	//nubemapeo.header.frame_id = "odom";
	//pcl_conversions::toPCL(ros::Time::now(), nubemapeo.header.stamp);
	//pubmap.publish(nubemapeo);
	
	//Obtenemos la imagen del sensor
	subnubesensor = n.subscribe("/camera_ir/depth/points",1,callbackObtenerNubeSensor);
	
	//Obtenemos la posicion y velocidad del robot 
	subscriberodom = n.subscribe("odom",1,callbackobtenerPosicionyVelocidad);
		



	ros::spin();	
	return 0;
}


void callbackObtenerNubeSensor(const sensor_msgs::PointCloud2& input){
	//Transformamos la nube de puntos del tipo sensor_msgs a pcl
	ROS_INFO("LLega nubesensor"); 
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(input,pcl_pc2);
	//Transformamos a pcl pointcloud xyzrgb
  	pcl::fromPCLPointCloud2(pcl_pc2, nubesensor);
	if(odometryobtenida){
		odometryobtenida=false;
		general();
	}
	
}

void callbackobtenerPosicionyVelocidad (const nav_msgs::Odometry input){
	posicionx=input.pose.pose.position.x;
	posiciony=input.pose.pose.position.y;
	posiciontheta=tf::getYaw(input.pose.pose.orientation);
	velocidadlineal=input.twist.twist.linear.x;
	velocidadangular=input.twist.twist.angular.z;
	odometryobtenida=true;
	ROS_INFO("LLega odometry"); 
}

cv::Mat obtenerImagenSensor(pcl::PointCloud<pcl::PointXYZRGB>in_nubesensor){
	//Primero debemos aplicar a la nube de puntos una matriz de traslación y rotación para dejar en el origen de coordenadas el punto más cercano y que este más a la izquierda
	//Declaramos la transformación a hacer poniendo una matriz identidad
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	//Añadimos la rotación para el eje z en la posición correspondiente
	float theta = 0;
	transform(0,0) = cos (theta);
  	transform(0,1) = -sin(theta);
  	transform(1,0) = sin (theta);
  	transform(1,1) = cos (theta);
	//Añadimos la traslación a la posición correspondiente (Los ejes en el sensor estan cambiados, siendo z el eje de la profundidad y el eje x el de la anchura)
	transform(0,3) = 3; //X
	transform(1,3) = 0; //Y
	transform(2,3) = -1; //Z

	//Aplicamos la transformación
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubesensor (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (in_nubesensor, *nubesensor, transform);

	
	//Definimos una estructura para almacenar la nube de puntos filtrada
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//Almacenamos la nube obtenida como puntero para poder posteriormente recortarla
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	//*cloud_in=nubesensor;
	//Recortamos la parte sobrante de la nube de puntos
	pcl::CropBox<pcl::PointXYZRGB> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(0, -1, 0, 1.0));
	boxFilter.setMax(Eigen::Vector4f(6,1, 4, 1.0));
	boxFilter.setInputCloud(nubesensor);
	boxFilter.filter(*bodyFiltered);
	//nubesensor=*bodyFiltered;

	//Publicamos si queremos para probar como se ve
	//pubpointcloud.publish(nubesensor);


	//Ahora pasaremos a proyectar esta nube de puntos en una imagen

	//Utilizamos CV_8UC3 Definiendo que el color utiliza un uint_8 y se usan 3 canales para pintar la imagen. Scalar indica el valor por defecto de toda la imágen

	cv::Mat imagen(100,150, CV_8UC3, cv::Scalar(0, 0,0));
	//cv::Mat imagen(60,90, CV_8UC3, cv::Scalar(0, 0,0));
	//Proyectamos la nube de puntos en la imagen recorriendo todos los puntos de la nube
	for(int i=0;i<bodyFiltered->size();i++){	
		cv::Vec3b color;
		//Obtenemos el color del pixel que tendría el punto en la nube de puntos, utilizamos el canal bgr porque la nube de puntos que obenemos viene en ese sistema
		//color[0]=nubesensor.points[i].b;color[1]=nubesensor.points[i].g;color[2]=nubesensor.points[i].r;
		color[0]=bodyFiltered->points[i].b;color[1]=bodyFiltered->points[i].g;color[2]=bodyFiltered->points[i].r;
	
		//Almacenamos el color en la matriz, multiplicamos *25 para cambiar la escala
		//imagen.at<cv::Vec3b>((int)(nubesensor.points[i].z*25),-(int)(nubesensor.points[i].x*25)) = color;
		imagen.at<cv::Vec3b>((int)(bodyFiltered->points[i].z*25),-(int)(bodyFiltered->points[i].x*25)) = color;
	
		//imagen.at<cv::Vec3b>((int)(nubesensor.points[i].z*15),-(int)(nubesensor.points[i].x*15)) = color;
	}
	
	//Creamos un cvImage donde almacenar la matriz y poder enviarlo utilizando ros
	cv_bridge::CvImage out_msg;
	out_msg.header.seq = 1; // user defined counter
	out_msg.header.stamp = ros::Time::now(); // añadimos el tiempo utilizado por ros 
	out_msg.encoding = sensor_msgs::image_encodings::RGB8; // AÑadimos el tipo de encoding
	out_msg.image    = imagen; // Añadimos la matriz
	pubsensor.publish(out_msg.toImageMsg());

	//Una vez que tenemos la matriz con la imagen la devolvemos.
	return imagen;
}
cv::Mat obtenerImagenParticula(pcl::PointCloud<pcl::PointXYZRGB>in_nubemapeo,float x, float y, float angle){
	//Primero debemos aplicar a la nube de puntos una matriz de traslación y rotación para dejar en el origen de coordenadas el punto más cercano y que este más a la izquierda
	//Declaramos la transformación a hacer poniendo una matriz identidad
	//Eigen::Matrix4f tras = Eigen::Matrix4f::Identity();
	//Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform= Eigen::Matrix4f::Identity() ;
	//Añadimos la rotación para el eje z en la posición correspondiente
	ROS_INFO("Angle: %f", angle * 180 / M_PI);
	float theta = -angle;
	transform(0,0) = cos (theta);
  	transform(0,1) = -sin(theta);
  	transform(1,0) = sin (theta);
  	transform(1,1) = cos (theta);
	//Añadimos la traslación a la posición correspondiente (Los ejes en el sensor estan cambiados, siendo z el eje de la profundidad y el eje x el de la anchura)
	//Hemos probado --, -+,+-,++
	transform(0,3) = -x; //X
	transform(1,3) = -y; //Y
	transform(2,3) = 0; //Z

	//Eigen::Matrix4f transform = rot*tras ;

	//Aplicamos la transformación
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubemapeo2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (in_nubemapeo, *nubemapeo2, transform);

	//Aplico la traslacción de cambio de eje
	theta=0;

	transform(0,0) = cos (theta);
  	transform(0,1) = -sin(theta);
  	transform(1,0) = sin (theta);
  	transform(1,1) = cos (theta);
	transform(0,3) = -1; //X
	transform(1,3) = -3; //Y
	transform(2,3) = 0; //Z
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubemapeo (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::transformPointCloud (*nubemapeo2, *nubemapeo, transform);

	//Definimos una estructura para almacenar la nube de puntos filtrada
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
	//Almacenamos la nube obtenida como puntero para poder posteriormente recortarla
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	//*cloud_in=nubemapeo;
	//Recortamos la parte sobrante de la nube de puntos
	pcl::CropBox<pcl::PointXYZRGB> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(0, -6, -1, 1.0));
	boxFilter.setMax(Eigen::Vector4f(4,0, 1, 1.0));
	boxFilter.setInputCloud(nubemapeo);
	boxFilter.filter(*bodyFiltered);
	//nubemapeo=*bodyFiltered;

	//Publicamos si queremos para probar como se ve
	pcl::PointCloud<pcl::PointXYZRGB>prueba2=*nubemapeo;
	prueba2.header.frame_id = "pioneer2dx/chassis";
	pcl_conversions::toPCL(ros::Time::now(), prueba2.header.stamp);
	pubpointcloud2.publish(prueba2);

	//Ahora pasaremos a proyectar esta nube de puntos en una imagen

	//Utilizamos CV_8UC3 Definiendo que el color utiliza un uint_8 y se usan 3 canales para pintar la imagen. Scalar indica el valor por defecto de toda la imágen

	cv::Mat imagen(100, 150, CV_8UC3, cv::Scalar(0,0,0));
	//cv::Mat imagen(60,90, CV_8UC3, cv::Scalar(0, 0,0));
	//Proyectamos la nube de puntos en la imagen recorriendo todos los puntos de la nube
	//for(int i=0;i<nubemapeo.size();i++){
	for(int i=0;i<bodyFiltered->size();i++){	
		cv::Vec3b color;
		//Obtenemos el color del pixel que tendría el punto en la nube de puntos
		color[0]=bodyFiltered->points[i].r;color[1]=bodyFiltered->points[i].g;color[2]=bodyFiltered->points[i].b;
		//Almacenamos el color en la matriz, multiplicamos *25 para cambiar la escala
		//imagen.at<cv::Vec3b>((int)(nubemapeo.points[i].x*15),-(int)(nubemapeo.points[i].y*15)) = color;
		imagen.at<cv::Vec3b>((int)(bodyFiltered->points[i].x*25),149+(int)(bodyFiltered->points[i].y*25)) = color;
	}

	// //Creamos un cvImage donde almacenar la matriz y poder enviarlo utilizando ros
	cv_bridge::CvImage out_msg;
	out_msg.header.seq = 1; // user defined counter
	out_msg.header.stamp = ros::Time::now(); // añadimos el tiempo utilizado por ros 
	out_msg.encoding = sensor_msgs::image_encodings::RGB8; // AÑadimos el tipo de encoding
	out_msg.image    = imagen; // Añadimos la matriz
	pubmapeo.publish(out_msg.toImageMsg());

	//Una vez que tenemos la matriz con la imagen la devolvemos.
	return imagen;

}

float registrarImagen(cv::Mat imagensensor,cv::Mat imagenparticula){
	// Convert images to gray scale;
	cv::Mat im1Gray, im2Gray;
	//cv::cvtColor(imagenparticula, im1Gray, CV_BGR2GRAY);
	//cv::cvtColor(imagenparticula, im2Gray, CV_BGR2GRAY);
	ROS_INFO("particula columnas %d filas %d", imagenparticula.cols, imagenparticula.rows); 
	ROS_INFO("sensor columnas %d filas %d", imagensensor.cols, imagensensor.rows); 
	cv::cvtColor(imagensensor, im1Gray, CV_RGB2GRAY);
	cv::cvtColor(imagenparticula, im2Gray, CV_RGB2GRAY);
	/* 	INtento 1
	// Define the motion model
	const int warp_mode = cv::MOTION_EUCLIDEAN;
	 
	// Set a 2x3 or 3x3 warp matrix depending on the motion model.
	cv::Mat warp_matrix;
	
	// Initialize the matrix to identity
	if ( warp_mode == cv::MOTION_HOMOGRAPHY )
		warp_matrix = cv::Mat::eye(3, 3, CV_32F);
	else
		warp_matrix = cv::Mat::eye(2, 3, CV_32F);
	
	// Specify the number of iterations.
	int number_of_iterations = 5000;
	
	// Specify the threshold of the increment
	// in the correlation coefficient between two iterations
	double termination_eps = 1e-10;
	
	// Define termination criteria
	cv::TermCriteria criteria (cv::TermCriteria::COUNT+cv::TermCriteria::EPS, number_of_iterations, termination_eps);
	
	// Run the ECC algorithm. The results are stored in warp_matrix.
	cv::findTransformECC(
					im1_gray,
					im2_gray,
					warp_matrix,
					warp_mode,
					criteria
				);
	ROS_INFO("Traslacción x %d ",warp_matrix.at<int>(0,2)); */
 
	//Intento 2
	const int MAX_FEATURES = 500;
	const float GOOD_MATCH_PERCENT = 0.20f;
	Mat im1Reg, h;
	// Variables to store keypoints and descriptors
  std::vector<KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;
   
  // Detect ORB features and compute descriptors.
  Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
  orb->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
  orb->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);

  //ROS_INFO("Keypoints1: %d, Keypoints2: %d", keypoints1.size(), keypoints2.size()); 
  if (!keypoints1.empty() && !keypoints2.empty()) {
  // Match features.
  std::vector<DMatch> matches;
  
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  ROS_INFO("INICIO MATCHER"); 
  matcher->match(descriptors1, descriptors2, matches, Mat());
  ROS_INFO("FIN MATCHER");  
  // Sort matches by score
  std::sort(matches.begin(), matches.end());
   
  // Remove not so good matches
  const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
  matches.erase(matches.begin()+numGoodMatches, matches.end());
   
   
  // Draw top matches
  Mat imMatches;
  drawMatches(imagensensor, keypoints1, imagenparticula, keypoints2, matches, imMatches);
  imwrite("/home/rafael/matches.jpg", imMatches);
	ROS_INFO("Imagen publicada"); 

	return 1;
  }
  /*
   // Extract location of good matches
   std::vector<Point2f> points1, points2;
 
  for( size_t i = 0; i < matches.size(); i++ )
  {
    points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
    points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
  }
   
  // Find homography
 // h = findHomography( points1, points2, RANSAC ); 
  
  // Use homography to warp image
 //warpPerspective(imagensensor, im1Reg, h, imagenparticula.size());
   
	ROS_INFO("Matriz obtenida [%f,%f,%f,%f,%f,%f,%f,%f,%f] ",h.at<double>(0,0),h.at<double>(0,1),h.at<double>(0,2),h.at<double>(1,0),h.at<double>(1,1),h.at<double>(1,2),h.at<double>(2,0),h.at<double>(2,1),h.at<double>(2,2)); 
	imwrite("/home/rafael/matches2.jpg", im1Reg );  */

	

	return 0;
}

//Comprimir todo Cntrl + K seguido de Cntrol+0.
//Descomprimir todo Cntrl + K seguido de Cntrol+J.
