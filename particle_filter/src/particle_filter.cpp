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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <omp.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;



//Definimos las variables generales del programa
//Asignamos la semilla 7 para los aleatorios
int semilla=8;

//Tiempo que tarda una iteración
float start2;
float stop2;

//Declaramos un booleano para sincronizar
bool nubesensorleida=false;

//Para almacenar lo obtenido por los callbacks
float posicionx=0;
float posiciony=0;
float posiciontheta=0;

float posicionanteriorx=0;
float posicionanteriory=0;
float posicionanteriortheta=0;

float velocidadlineal=0;
float velocidadangular=0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubesensor (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

//Para publicar en topics
image_transport::Publisher pubsensor;
image_transport::Publisher pubmapeo;
ros::Publisher pubpointcloud;
ros::Publisher pubpointcloud2;
ros::Publisher pubparticulas;
ros::Publisher pubmejorparticula;
ros::Publisher pubmap;

//Para definir el número de particulas 
int numeroparticulas=100;

//Para generar los diferentes posibles errores 
default_random_engine gen;


//Declaramos el filtro de particulas
ParticleFilter pf;

using namespace std;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = numeroparticulas; //set to number of files in observation directory

    weights.resize(num_particles);
    particles.resize(num_particles);

    // Creamos una distribucción normal para añadir ruido y obtener las nuevas particulas
    normal_distribution<double> dist_x(0, std[0]); 
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);

	//Añadimos al conjunto de particulas las particulas con el ruido
    for(int i=0; i<num_particles; i++){
		//Obtenemos un valor de la distribución normal
		double genx=dist_x(gen);
		double geny=dist_y(gen);
		double gentheta=dist_theta(gen);

		//1ROS_INFO("\n Particula %d \nValores ruidoparticula_x %f , ruidoparticula_y : %f , ruidoparticula_theta : %f\n",i,genx,geny,gentheta);

        Particle p;
        p.id = i;
        p.x = genx+x; 
        p.y = geny+y;
        p.theta = gentheta+theta; //+1 0.021061; Todo 0 0.071869;
        p.weight = 1;

        particles[i] = p;
        weights[i] = p.weight;
		//1ROS_INFO("\n Inicializacion del filtro de particulas \n Particula %d -> x=%f y=%f theta=%f \n",i,p.x,p.y,p.theta);
    }
	is_initialized = true;
}

void ParticleFilter::prediction(double diferenciax,double diferenciay,double diferenciatheta, double std_pos[]){
	    for(int i=0; i<num_particles; i++){

		// Predecimos la posición basandonos en la diferencia de odometrias
		double new_x = particles[i].x + diferenciax;
		double new_y = particles[i].y + diferenciay;
		double new_theta = particles[i].theta + diferenciatheta;
		
		//Calculamos el ruido gaussiano que añadiremos a la predición para que los resamples no utilicen siempre la misma particula
		
		normal_distribution<double> dist_x(new_x, std_pos[0]);
		normal_distribution<double> dist_y(new_y, std_pos[1]);
		normal_distribution<double> dist_theta(new_theta, std_pos[2]);

		//Obtenemos un valor de la distribución normal
		particles[i].x=dist_x(gen);
		particles[i].y=dist_y(gen);
		particles[i].theta=dist_theta(gen);

		//1ROS_INFO("\n Prediccion del filtro de particulas \n Particula %d -> x=%f y=%f theta=%f \n",i,p->x,p->y,p->theta);
   
    }
}

void ParticleFilter::updateWeights(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr nubemapeo, pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubesensor) {
	//Sumatorio de pesos para luego normalizar
    double weights_sum = 0;
    //Float para almacenar la similitud entre particula y lo que obtiene el sensor
    double coincidencia=0;
	//Obtenemos la imagen de nubesensor
	float startobtenerimagensensor= clock();
	cv::Mat imagensensor=obtenerImagenSensor(nubesensor);
	float stopobtenerimagensensor= clock();
	ROS_INFO("\nTiempo que se tarda en calcular la imagen del sensor: %f\n",(stopobtenerimagensensor-startobtenerimagensensor)/double (CLOCKS_PER_SEC));
		
	float startcoincidencia=clock();
	//Para cada particula
	for(int j=0; j<num_particles; j++){
		//Tiempo que tarda en obtener la imagen desde una particula
		float startobtenerimagenparticula= clock();
		//Obtenemos la imagen de la particula
		ROS_INFO("PETA AQUI?0");
		cv::Mat imagenparticula=obtenerImagenParticula(nubemapeo,particles[j].x,particles[j].y,particles[j].theta);
		float stopobtenerimagenparticula=clock();
		ROS_INFO("\nTiempo que se tarda en calcular la imagen de la particula: %f\n",(stopobtenerimagenparticula-startobtenerimagenparticula)/double (CLOCKS_PER_SEC));
		//Calculamos cual es el peso correspondiente a la particula
		//Tiempo que tarda en calcular el peso de la imagen
		//coincidencia=registrarImagen(imagensensor,imagenparticula);
		coincidencia=histogramaImagen(imagensensor,imagenparticula);
		//coincidencia=calcularpeso1(imagensensor,imagenparticula);
		
		//cambiar esto para cambiar el método de peso
		ROS_INFO("Coincidencia particula POSTFORMULA %d: %f",j,coincidencia); 
		//Calculamos cual sería el mayor error entre lo que nos dicen los sensores y lo que podríamos tener en las particulas.
		weights_sum += coincidencia;
		particles[j].weight = coincidencia;
    }
	float stopcoincidencia=clock();
	ROS_INFO("\nTiempo que se tarda en calcular el peso de todas las particulas: %f\n",(stopcoincidencia-startcoincidencia)/double (CLOCKS_PER_SEC));
	//Definimos el peso máximo y la mejor particula para luego publicarla
	double highest_weight = 0.0;
	Particle best_particle;
	int indicemejor=0;
    // Normalizamos los pesos y calculamos la mejor particula
    for (int i = 0; i < num_particles; i++) {
		//Dividimos el peso entre el sumatorio de todos los pesos
        particles[i].weight /= weights_sum;
		//Añadimos el peso de la particula al vector de pesos
        weights[i] = particles[i].weight;
		//Calculamos cual es la mejor particula
		if (particles[i].weight > highest_weight) {
			highest_weight = particles[i].weight;
			best_particle = particles[i];
			indicemejor=i;
		}
		//ROS_INFO("\nCoincidencia Particula normalizada \nPeso particula %d : %f\n",i,p->weight);
    }

	//Publicamos la particula con mayor peso
	geometry_msgs::PoseArray particulas;
	geometry_msgs::Pose particula;
	particulas.header.frame_id = "map";
    particulas.header.stamp = ros::Time::now();
	
	
	//Declaramos una variable para almacenar el error de la mejor particula
	float errormejorparticulax=0;
	float errormejorparticulay=0;
	float errormejorparticulatheta=0;

	//Almacenamos el error de la mejor particula
	errormejorparticulax=abs(posicionx-best_particle.x);
	errormejorparticulay=abs(posiciony-best_particle.y);
	errormejorparticulatheta=abs(posiciontheta-best_particle.theta);
	//Escribimos el error de la mejor particula en un fichero
	pf.write("/home/rafael/catkin_ws/Datos/errormejorparticula.csv", errormejorparticulax,errormejorparticulay,errormejorparticulatheta);

	particula.position.x=best_particle.x;
	particula.position.y=best_particle.y;
	particula.position.z=0;
	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY( 0, 0, best_particle.theta ); 
	particula.orientation.x=myQuaternion[0];
	particula.orientation.y=myQuaternion[1];
	particula.orientation.z=myQuaternion[2];
	particula.orientation.w=myQuaternion[3];
	particulas.poses.push_back(particula);	 
	 //Publicamos la mejor particula particula
    pubmejorparticula.publish(particulas);

	cv::Mat imagenmejorparticula=obtenerImagenParticula(nubemapeo,best_particle.x,best_particle.y,best_particle.theta);

	cv_bridge::CvImage out_msg;
	out_msg.header.seq = 1; // user defined counter
	out_msg.header.stamp = ros::Time::now(); // añadimos el tiempo utilizado por ros 
	out_msg.encoding = sensor_msgs::image_encodings::RGB8; // AÑadimos el tipo de encoding
	out_msg.image    = imagenmejorparticula; // Añadimos la matriz
	pubmapeo.publish(out_msg.toImageMsg());

	ROS_INFO("\nPosición mejor particula x: %f, y: %f\nPeso mejor particula : %f\n",best_particle.x,best_particle.y,best_particle.weight);

}
//Método resample wheel
void ParticleFilter::resample() {
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

void ParticleFilter::write(std::string filename, float x,float y,float theta) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	dataFile << x << ";" << y << ";" << theta << "\n";
	dataFile.close();
}

bool ParticleFilter::initialized() {
		return is_initialized;
}
void general(){
	//Obtenemos la nube de puntos del mapeo, lo suyo sería poner esto en el main, y no volver a cargarlo

	pcl::PointCloud<pcl::PointXYZRGB> nubemapeo;
	nubemapeo=*cloud;
	nubemapeo.header.frame_id = "odom";
	pcl_conversions::toPCL(ros::Time::now(), nubemapeo.header.stamp);
	pubmap.publish(nubemapeo);

	srand(semilla);

	double sigma_pos [3] = {3, 3, 0.25}; // Varianzas en la distribucción normal del GPS [x [m], y [m], theta [rad]]
	//Error de 9m en x, y y 42º
	double ruidoparticulas[3]={0.3,0.3,0.1}; //17 grados de diferencia, 1 metro de diferencia en x e y.
	//double sigma_pos [3] = {0, 0, 0}; 
	//double ruidoparticulas[3]={0,0,0}; 
	
	//Distribuciones normales con media 0 y varianza la declarada anteriormente
	normal_distribution<double> N_x_init(0, sigma_pos[0]);
	normal_distribution<double> N_y_init(0, sigma_pos[1]);
	normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	double n_x, n_y, n_theta;

	// Inicializamos el filtro de particulas, en el caso de que sea la primera iteración.
	if (!pf.initialized()) {
	//Para probar como te sigue correctamente la particula
	//if (1) {
		//Generamos un aleatorio de la distribución N_x_init
		n_x = N_x_init(gen);
		//Generamos un aleatorio de la distribución N_y_init
		n_y = N_y_init(gen);
		//Generamos un aleatorio de la distribución N_theta_init
		n_theta = N_theta_init(gen);
		ROS_INFO("\nAl iniciar se tiene un error de\n n_x: %f, n_y: %f, n_theta: %f\n",n_x,n_y,n_theta);
		//Inicializamos el filtro de particulas con la posición global del robot  más un pequeño error generado aleatoriamente n_x,z,theta.

		float startinit=clock();
		pf.init(posicionx + n_x, posiciony + n_y, posiciontheta + n_theta, sigma_pos);
		float stopinit=clock();
		ROS_INFO("\nTiempo que se tarda en realizar la inicializacion de particulas: %f\n",(stopinit-startinit)/double (CLOCKS_PER_SEC));
	}
	else{
		// Se predice la posición del vehiculo en el proximo instante de tiempo. Pasandole el tiempo que pasará hasta la proxima medida, los posibles errores, la velocidad lineal y angular que tenian en el anterior instante
		float startprediction=clock();
		pf.prediction(posicionx-posicionanteriorx,posiciony-posicionanteriory,posiciontheta-posicionanteriortheta, ruidoparticulas);
		float stopprediction=clock();
		ROS_INFO("\nTiempo que se tarda en realizar la prediccion de particulas: %f\n",(stopprediction-startprediction)/double (CLOCKS_PER_SEC)); 
	}
	//Declaramos una variable para almacenar el error de todas las particulas
	float errorconjuntox=0;
	float errorconjuntoy=0;
	float errorconjuntotheta=0;


	/* //Publicamos las particulas
	geometry_msgs::PoseArray particulas;
	geometry_msgs::Pose particula;
	particulas.header.frame_id = "map";
    particulas.header.stamp = ros::Time::now();
	
	for (int i=0;i<pf.particles.size();i++){
		//Almacenamos el error
		errorconjuntox+=abs(posicionx-pf.particles[i].x);
		errorconjuntoy+=abs(posiciony-pf.particles[i].y);
		errorconjuntotheta+=abs(posiciontheta-pf.particles[i].theta);
		
		//Almacenamos en el topic las particulas
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

	 } */

	//Escribimos en un fichero el error del conjunto de particulas
	pf.write("/home/rafael/catkin_ws/Datos/errorconjunto.csv", errorconjuntox,errorconjuntoy,errorconjuntotheta);

	//Publicamos las particulas
    //pubparticulas.publish(particulas); 

	// Se actualizan los pesos y se realiza el resample añadiendo ruido
	//Aqui no se si nubesensor esta actualizada, porque ha pasado un tiempo ARREGLAR
	float startactualizacionpesos=clock();
	pf.updateWeights(cloud, nubesensor);
	float stopactualizacionpesos=clock();
	ROS_INFO("\nTiempo que se tarda en realizar la actualizacion de pesos: %f\n",(stopactualizacionpesos-startactualizacionpesos)/double (CLOCKS_PER_SEC));
		
	float startresample=clock();
	pf.resample();
	float stopresample=clock();
	ROS_INFO("\nTiempo que se tarda en realizar el resample: %f\n",(stopresample-startresample)/double (CLOCKS_PER_SEC));
	

	//Publicamos las particulas elegidas del resample
	geometry_msgs::PoseArray particulas;
	geometry_msgs::Pose particula;
	particulas.header.frame_id = "map";
    particulas.header.stamp = ros::Time::now();
	
	for (int i=0;i<pf.particles.size();i++){		
		//Almacenamos en el topic las particulas
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
	pubparticulas.publish(particulas); 

	 //Almacenamos las posiciones anteriores
	 posicionanteriorx=posicionx;
	 posicionanteriory=posiciony;
	 posicionanteriortheta=posiciontheta;
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
	pubmejorparticula=n.advertise<geometry_msgs::PoseArray>("mejorParticula",1);

	//Asignamos una semilla para que sea reproducible
	gen.seed(8);

	//Obtenemos la nube de puntos del mapeo, podriamos en vez de suscribirmos obtenerla de un archivo y aceleramos calculos
	//subnubemapeo = n.subscribe("/cloud_pcd",1,callbackObtenerNubeMapeo);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/rafael/catkin_ws/pcd/prueba2.pcd", *cloud);
	//Obtenemos la imagen del sensor
	subnubesensor = n.subscribe("/camera_ir/depth/points",1,callbackObtenerNubeSensor);
	
	//Obtenemos la posicion y velocidad del robot 
	subscriberodom = n.subscribe("odom",1,callbackobtenerPosicionyVelocidad);
		


	//Con esto no hay paralelismo
	ros::spin();
	//Con esto hay paralelismo
	//ros::MultiThreadedSpinner spinner(2);
	//spinner.spin();
	return 0;
}


void callbackObtenerNubeSensor(const sensor_msgs::PointCloud2& input){
	//Transformamos la nube de puntos del tipo sensor_msgs a pcl
	ROS_INFO("LLega nubesensor"); 
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(input,pcl_pc2);
	//Transformamos a pcl pointcloud xyzrgb
	pcl::PointCloud<pcl::PointXYZRGB> nube;
  	pcl::fromPCLPointCloud2(pcl_pc2, nube);
	//Almacenamos la nube de puntos en el puntero 
	*nubesensor=nube;  
	nubesensorleida=true;
	
}

void callbackobtenerPosicionyVelocidad (const nav_msgs::Odometry input){
	//UN nuevo reloj para comprobar cuanto tarda para llegar a la proxima iteracción y calcular delta
	start2 = clock();
	//Almacenamos la posición del robot, para utilizarla en la primera iteracción como la posición que nos daría el gps
	posicionx=input.pose.pose.position.x;
	posiciony=input.pose.pose.position.y;
	posiciontheta=tf::getYaw(input.pose.pose.orientation);
	//Almacenamos la velocidad lineal y angular
	velocidadlineal=input.twist.twist.linear.x;
	velocidadangular=input.twist.twist.angular.z;
	ROS_INFO("LLega odometry"); 
	
	//Solo realizamos el proceso si se ha leido la nube del sensor
	if(nubesensorleida){
		nubesensorleida=false;
		general();

	//Calculamos el valor de delta
	stop2 = clock();
	double runtime2 = (stop2 - start2) / double(CLOCKS_PER_SEC);
	cout << "Runtime delta (sec): " << runtime2 << endl;
	}

}

cv::Mat obtenerImagenSensor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_nubesensor){
	//Definimos una estructura para almacenar la nube de puntos filtrada
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);

	//Recortamos la parte sobrante de la nube de puntos
	pcl::CropBox<pcl::PointXYZRGB> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-3, -1, 1, 1.0));
	boxFilter.setMax(Eigen::Vector4f(3,1, 5, 1.0));
	boxFilter.setInputCloud(in_nubesensor);
	boxFilter.filter(*bodyFiltered);

	//Ahora pasaremos a proyectar esta nube de puntos en una imagen

	//Utilizamos CV_8UC3 Definiendo que el color utiliza un uint_8 y se usan 3 canales para pintar la imagen. Scalar indica el valor por defecto de toda la imágen

	//cv::Mat imagen(100,150, CV_8UC3, cv::Scalar(0, 0,0));
	cv::Mat imagen(70,100, CV_8UC3, cv::Scalar(0, 0,0));
	//Proyectamos la nube de puntos en la imagen recorriendo todos los puntos de la nube
	for(int i=0;i<bodyFiltered->size();i++){	
		cv::Vec3b color;
		//Obtenemos el color del pixel que tendría el punto en la nube de puntos, utilizamos el canal bgr porque la nube de puntos que obenemos viene en ese sistema
		color[0]=bodyFiltered->points[i].b;color[1]=bodyFiltered->points[i].g;color[2]=bodyFiltered->points[i].r;
		//Almacenamos el color en la matriz, añadiendo la traslacción -1 al eje de la profundidad y +3 en el de la anchura, además multiplicamos *15 para cambiar la escala
		imagen.at<cv::Vec3b>((int)((bodyFiltered->points[i].z-1)*15),-(int)((bodyFiltered->points[i].x+3)*15)) = color;
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

cv::Mat obtenerImagenParticula(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_nubemapeo,float x, float y, float angle){
	//try{
	//Almacenamos la nube de puntos de entrada en un nuevo puntero.
	//pcl::PointCloud<pcl::PointXYZRGB> nubemapeo (new pcl::PointCloud<pcl::PointXYZRGB>);
	//*nubemapeo=in_nubemapeo;

	//Para no realizar transformaciones a toda la nube de puntos nos quedamos con una zona que rode a la posición del robot 5 m por cada lado ya que es lo máximo que ve en profundidad
	ROS_INFO("1");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);

	//Recortamos la parte sobrante de la nube de puntos
	pcl::CropBox<pcl::PointXYZRGB> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-5, -5, -1, 1.0));
	boxFilter.setMax(Eigen::Vector4f(5,5, 1, 1.0));
	boxFilter.setInputCloud(in_nubemapeo); 
	boxFilter.filter(*bodyFiltered);
	ROS_INFO("POSTFILTRADO");

	//Primero debemos aplicar a la nube de puntos una matriz de traslación y rotación para dejar en el origen de coordenadas el punto más cercano y que este más a la izquierda
	//Declaramos la transformación a hacer poniendo una matriz identidad
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	//Añadimos la rotación para el eje z en la posición correspondiente
	float theta = -angle;
	transform(0,0) = cos (theta);
  	transform(0,1) = -sin(theta);
  	transform(1,0) = sin (theta);
  	transform(1,1) = cos (theta);
	//Añadimos la traslación a la posición correspondiente 
	transform(0,3) = cos(theta)*(-x)-sin(theta)*(-y)-1; //X
	transform(1,3) = cos(theta)*(-y)+sin(theta)*(-x)-3, //Y
	transform(2,3) = 0; //Z

	ROS_INFO("POSTTRASNFORMACION");

	//Aplicamos la transformación
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubemapeotrasladada (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*bodyFiltered, *nubemapeotrasladada, transform);

	//Volvemos a filtrar la imágen
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeparticula (new pcl::PointCloud<pcl::PointXYZRGB>);
	//Recortamos la parte sobrante de la nube de puntos
	boxFilter.setMin(Eigen::Vector4f(0, -6, -1, 1.0));
	boxFilter.setMax(Eigen::Vector4f(4,0, 1, 1.0));
	boxFilter.setInputCloud(nubemapeotrasladada); 
	boxFilter.filter(*nubeparticula);

	ROS_INFO("POSTFILTRADO2");
	//Ahora pasaremos a proyectar esta nube de puntos en una imagen

	//Utilizamos CV_8UC3 Definiendo que el color utiliza un uint_8 y se usan 3 canales para pintar la imagen. Scalar indica el valor por defecto de toda la imágen

	//cv::Mat imagen(100, 150, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat imagen(70,100, CV_8UC3, cv::Scalar(0, 0,0));
	//Proyectamos la nube de puntos en la imagen recorriendo todos los puntos de la nube
	//for(int i=0;i<nubemapeo.size();i++){
	for(int i=0;i<nubeparticula->size();i++){	
		cv::Vec3b color;
		//Obtenemos el color del pixel que tendría el punto en la nube de puntos
		color[0]=nubeparticula->points[i].r;color[1]=nubeparticula->points[i].g;color[2]=nubeparticula->points[i].b;
		//Almacenamos el color en la matriz, multiplicamos *25 para cambiar la escala
		imagen.at<cv::Vec3b>((int)(nubeparticula->points[i].x*15),89+(int)(nubeparticula->points[i].y*15)) = color;
		//ROS_INFO("imagen y: %d",89+(int)(nubeparticula->points[i].y*15));
		//imagen.at<cv::Vec3b>((int)(nubeparticula->points[i].x*15),-(int)(nubeparticula->points[i].y*15)) = color;
		//ARREGLAR
	}
	ROS_INFO("IMAGENOBTENIDA");
	// //Creamos un cvImage donde almacenar la matriz y poder enviarlo utilizando ros
	/*
	cv_bridge::CvImage out_msg;
	out_msg.header.seq = 1; // user defined counter
	out_msg.header.stamp = ros::Time::now(); // añadimos el tiempo utilizado por ros 
	out_msg.encoding = sensor_msgs::image_encodings::RGB8; // AÑadimos el tipo de encoding
	out_msg.image    = imagen; // Añadimos la matriz
	pubmapeo.publish(out_msg.toImageMsg());
	*/
	//Una vez que tenemos la matriz con la imagen la devolvemos.
	return imagen;
	//}
	//catch(exception& e){}
	//return cv::Mat (60,90, CV_8UC3, cv::Scalar(0, 0,0));

}

float registrarImagen(cv::Mat imagensensor,cv::Mat imagenparticula){
	// Convert images to gray scale;
	cv::Mat im1Gray, im2Gray;
	//cv::cvtColor(imagenparticula, im1Gray, CV_BGR2GRAY);
	//cv::cvtColor(imagenparticula, im2Gray, CV_BGR2GRAY);
	//ROS_INFO("particula columnas %d filas %d", imagenparticula.cols, imagenparticula.rows); 
	//ROS_INFO("sensor columnas %d filas %d", imagensensor.cols, imagensensor.rows); 
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
	//ROS_INFO("INICIO MATCHER"); 
	matcher->match(descriptors1, descriptors2, matches, Mat());
	//ROS_INFO("FIN MATCHER");  
	// Sort matches by score
	std::sort(matches.begin(), matches.end());
	
	// Remove not so good matches
	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
	matches.erase(matches.begin()+numGoodMatches, matches.end());
	
	
	// Draw top matches
	Mat imMatches;
	drawMatches(imagensensor, keypoints1, imagenparticula, keypoints2, matches, imMatches);
	imwrite("/home/rafael/matches.jpg", imMatches);
	//ROS_INFO("Imagen publicada"); 
	/*
	// Extract location of good matches
	std::vector<Point2f> points1, points2;
	
	for( size_t i = 0; i < matches.size(); i++ )
	{
		points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
		points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
	}
	
	// Find homography
	h = findHomography( points1, points2, RANSAC ); 
	
	// Use homography to warp image
	warpPerspective(imagensensor, im1Reg, h, imagenparticula.size());
	
	ROS_INFO("Matriz obtenida [%f,%f,%f,%f,%f,%f,%f,%f,%f] ",h.at<double>(0,0),h.at<double>(0,1),h.at<double>(0,2),h.at<double>(1,0),h.at<double>(1,1),h.at<double>(1,2),h.at<double>(2,0),h.at<double>(2,1),h.at<double>(2,2)); 
	imwrite("/home/rafael/matches2.jpg", im1Reg );  
	*/
	return 1;
  }
  
   

	return 0;
}

//Obtener el peso de las particulas basandonos en el histograma
float histogramaImagen(cv::Mat imagensensor,cv::Mat imagenparticula){
	//Definimos el número de subimagenes que vamos a realizar por fila o columna
	int numerosubimagenes=4; //Tendriamos numerosubimagenes*numerosubimagenes =100
	//Creamos un vector que definirá la distancia entre las subimagenes de imagen sensor e imagen particula
	float matrizcomparacion[numerosubimagenes*numerosubimagenes];
	//Definimos la altura y anchura de las submatrices

	int anchura=imagensensor.cols/numerosubimagenes; 
	int altura=imagensensor.rows/numerosubimagenes; 

	//Creamos variables para almacenar los valores RGB
	float valormedioR=0;
	float valormedioG=0;
	float valormedioB=0;

	//Recorremos las subimágenes
	for(int i=0;i<numerosubimagenes;i++){
		for(int j=0;j<numerosubimagenes;j++){
			//Declaramos una variable para almacenar el valor acumulado de cada uno de los canales, BGR para imagensensor
			float valor[3]={0,0,0};
			//Declaramos una variable para almacenar el valor acumulado de cada uno de los canales, BGR para imagenparticula
			float valorparticula[3]={0,0,0};
			//Calculamos el valor acumulado en cada canal de la imagensensor
			for(int h=altura*i;h<altura*(i+1);h++){
				for(int w=anchura*j;w<anchura*(j+1);w++){
					//Solamente se suma al valor acumulado si la imagen sensor no tiene ese pixel en negro
					if((int)imagensensor.at<Vec3b>(h,w)[0]!=0 && (int)imagensensor.at<Vec3b>(h,w)[1]!=0 && (int)imagensensor.at<Vec3b>(h,w)[2]!=0){
						valor[0]=valor[0]+(int)imagensensor.at<Vec3b>(h,w)[0]; //B
						valor[1]=valor[1]+(int)imagensensor.at<Vec3b>(h,w)[1]; //G
						valor[2]=valor[2]+(int)imagensensor.at<Vec3b>(h,w)[2]; //R

						valorparticula[0]=valorparticula[0]+(int)imagenparticula.at<Vec3b>(h,w)[0]; //B
						valorparticula[1]=valorparticula[1]+(int)imagenparticula.at<Vec3b>(h,w)[1]; //G
						valorparticula[2]=valorparticula[2]+(int)imagenparticula.at<Vec3b>(h,w)[2]; //R
					}
				}
			}
			//Hacemos la media de los valores de cada canal de la imagen sensor
			valor[0]=valor[0]/(anchura*altura);
			valor[1]=valor[1]/(anchura*altura);
			valor[2]=valor[2]/(anchura*altura);
			//Hacemos la media de los valores de cada canal de la imagen sensor
			valorparticula[0]=valorparticula[0]/(anchura*altura);
			valorparticula[1]=valorparticula[1]/(anchura*altura);
			valorparticula[2]=valorparticula[2]/(anchura*altura);

			//Realizamos la resta para calcular la distancia entre los dos valores medios
			matrizcomparacion[i*numerosubimagenes+j]=abs(valorparticula[0]-valor[0])+abs(valorparticula[1]-valor[1])+abs(valorparticula[2]-valor[2]);
		}
	}

	//ROS_INFO("matrizcomparación: \n Subimagen 0: %f \n Subimagen 1: %f \n Subimagen 2: %f \n Subimagen 3: %f", matrizcomparacion[0], matrizcomparacion[1], matrizcomparacion[2], matrizcomparacion[3]);
	
	// 
	float valor=0;
	for(int i=0;i<numerosubimagenes;i++){
		for(int j=0;j<numerosubimagenes;j++){
			valor=valor+matrizcomparacion[i*numerosubimagenes+j];
		}
	}
	
	ROS_INFO("Coincidencia PREFORMULA: %f", valor);
	
	return  pow(3, 3/(valor/100));//Formula para 2x2
	//return  pow(3, 3/(valor/1000));//Formula para 4x4

	
}
//Prueba 
float calcularpeso1(cv::Mat imagensensor,cv::Mat imagenparticula){
	return 1;
}


/* 
//Obtener el peso de las particulas basandonos en el histograma
float histogramaImagen(cv::Mat imagensensor,cv::Mat imagenparticula){
	//Definimos el número de subimagenes que vamos a realizar por fila o columna
	int numerosubimagenes=2; //Tendriamos numerosubimagenes*numerosubimagenes =100
	//Creamos un vector que definirá la distancia entre las subimagenes de imagen sensor e imagen particula
	float matrizcomparacion[numerosubimagenes*numerosubimagenes];
	//Definimos la altura y anchura de las submatrices

	int anchuraaprovechable=1*(imagensensor.cols/6);// 15
	int anchuranoaprovechable=2.5*(imagensensor.cols/6); // 37.5=37
	int anchura=anchuraaprovechable/numerosubimagenes; //7.5=7
	int altura=imagensensor.rows/numerosubimagenes; //30

	//Creamos variables para almacenar los valores RGB
	float valormedioR=0;
	float valormedioG=0;
	float valormedioB=0;

	//Recorremos las subimágenes
	for(int i=0;i<numerosubimagenes;i++){
		for(int j=0;j<numerosubimagenes;j++){
			//Declaramos una variable para almacenar el valor acumulado de cada uno de los canales, BGR para imagensensor
			float valor[3]={0,0,0};
			//Calculamos el valor acumulado en cada canal de la imagensensor
			for(int h=altura*i;h<altura*(i+1);h++){
				for(int w=anchuranoaprovechable+anchura*j;w<anchuranoaprovechable+anchura*(j+1);w++){
					valor[0]=valor[0]+(int)imagensensor.at<Vec3b>(h,w)[0]; //B
					valor[1]=valor[1]+(int)imagensensor.at<Vec3b>(h,w)[1]; //G
					valor[2]=valor[2]+(int)imagensensor.at<Vec3b>(h,w)[2]; //R
				}
			}
			//Hacemos la media de los valores de cada canal de la imagen sensor
			valor[0]=valor[0]/(anchura*altura);
			valor[1]=valor[1]/(anchura*altura);
			valor[2]=valor[2]/(anchura*altura);

			//Declaramos una variable para almacenar el valor acumulado de cada uno de los canales, BGR para imagenparticula
			float valorparticula[3]={0,0,0};
			//Calculamos el valor acumulado en cada canal de la imagenparticula
			for(int h=altura*i;h<altura*(i+1);h++){
				for(int w=anchuranoaprovechable+anchura*j;w<anchuranoaprovechable+anchura*(j+1);w++){
					valorparticula[0]=valorparticula[0]+(int)imagenparticula.at<Vec3b>(h,w)[0]; //B
					valorparticula[1]=valorparticula[1]+(int)imagenparticula.at<Vec3b>(h,w)[1]; //G
					valorparticula[2]=valorparticula[2]+(int)imagenparticula.at<Vec3b>(h,w)[2]; //R
				}
			}
			//Hacemos la media de los valores de cada canal de la imagen sensor
			valorparticula[0]=valorparticula[0]/(anchura*altura);
			valorparticula[1]=valorparticula[1]/(anchura*altura);
			valorparticula[2]=valorparticula[2]/(anchura*altura);

			//Realizamos la resta para calcular la distancia entre los dos valores medios
			matrizcomparacion[i*numerosubimagenes+j]=abs(valorparticula[0]-valor[0])+abs(valorparticula[1]-valor[1])+abs(valorparticula[2]-valor[2]);
		}
	}

	//ROS_INFO("matrizcomparación: \n Subimagen 0: %f \n Subimagen 1: %f \n Subimagen 2: %f \n Subimagen 3: %f", matrizcomparacion[0], matrizcomparacion[1], matrizcomparacion[2], matrizcomparacion[3]);
	
	// 
	float valor=0;
	for(int i=0;i<numerosubimagenes;i++){
		for(int j=0;j<numerosubimagenes;j++){
			valor=valor+matrizcomparacion[i*numerosubimagenes+j];
		}
	}
	
	ROS_INFO("Coincidencia PREFORMULA: %f", valor);
	
	return  pow(3, 1/(valor/100));//1/valor;//pow(2,1/valor);
	
}
 */


//Comprimir todo Cntrl + K seguido de Cntrol+0.
//Descomprimir todo Cntrl + K seguido de Cntrol+J.
