#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

//Definimos el publisher
ros::Publisher pubodometryfalsa;

//Definimos los errores 
float errortotalx=0;
float errortotaly=0;
float errortotaltheta=0;
//Definimos una variable para ir incrementando el error en el incremento definido
//float erroralmoverse=0.000155; //erroralmoverse*65=error por segundo=0.01 m/s de desviación
//float erroralrotar=0.000769; //Erroralrotar*65=erroranguloporsegundo=0.05 radianes/s=3º/s

float erroralmoverse=0.0012;//0.08 m/s
float erroralrotar=0.0014; //6º/s
nav_msgs::Odometry inputanterior;
nav_msgs::Odometry odometriafalsa;
bool primeravez=true;
void callbackOdom (const nav_msgs::Odometry input){
	//Objeto que almacena la odometria modificada
	nav_msgs::Odometry odometriafalsa; 
	//Copiamos la posición real del robot 
	if (primeravez){
		odometriafalsa=input;
		primeravez=false;
		inputanterior=input;
	}
	//Calculamos la diferencia entre la anterior posición y la actual
	float diferencialineal=abs(input.pose.pose.position.x-inputanterior.pose.pose.position.x)+abs(input.pose.pose.position.y-inputanterior.pose.pose.position.y);
	float diferenciaangular=tf::getYaw(input.pose.pose.orientation)-tf::getYaw(inputanterior.pose.pose.orientation);
	diferenciaangular=fmod(diferenciaangular,M_PI);


	//Definimos un quaternion donde le metemos como yaw la rotación de la posición real + el error
	tf::Quaternion myQuaternion;
	//tf::getYaw(odometriafalsa.pose.pose.orientation)+diferenciaangular
	myQuaternion =tf::createQuaternionFromRPY(0, 0,tf::getYaw(input.pose.pose.orientation));
	myQuaternion.setRPY( 0, 0, tf::getYaw(input.pose.pose.orientation)); 
	myQuaternion.normalize();
	odometriafalsa.pose.pose.orientation.x=myQuaternion[0];
	odometriafalsa.pose.pose.orientation.y=myQuaternion[1];
	odometriafalsa.pose.pose.orientation.z=myQuaternion[2];
	odometriafalsa.pose.pose.orientation.w=myQuaternion[3];


	odometriafalsa.pose.pose.position.x+=diferencialineal*cos(tf::getYaw(odometriafalsa.pose.pose.orientation))*1.1;
	odometriafalsa.pose.pose.position.y+=diferencialineal*sin(tf::getYaw(odometriafalsa.pose.pose.orientation))*1.1;


	//Publicamos la odometria con error
	pubodometryfalsa.publish(odometriafalsa);
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generarodometryconerror");
  ros::NodeHandle n;
  
  //Nos suscribimos a odom para modificarla y meterle error
  ros::Subscriber subodometry = n.subscribe("/odom",1,callbackOdom);
  pubodometryfalsa=n.advertise<nav_msgs::Odometry>("odometriafalsa", 1);
  ros::spin();

  return 0;
}
