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

float errorminimoalmoverse=0.001;//0.06 m/s
float errorminimoalrotar=0.000769; //3º/s
float erroralmoverse=0;
float erroralrotar=0;
float errormaximoalmoverse=0.002;// 0.13 m/s
float errormaximoalrotar=0.00268461538; //6º/s
//Objeto que almacena la odometria modificada
nav_msgs::Odometry odometriafalsa;
bool primeravez=true;
void callbackOdom (const nav_msgs::Odometry input){
	erroralrotar = (rand() % (int)(errormaximoalrotar*1000000-errorminimoalrotar*1000000)+errorminimoalrotar*1000000)/1000000;
 	erroralmoverse = (rand() % (int)(errormaximoalmoverse*1000-errorminimoalmoverse*1000)+errorminimoalmoverse*1000)/1000;
	errortotaltheta=0;
	errortotalx=0;
	errortotaly=0;
	//La primera vez que sea llamado obtendrá la posición real del robot
	if(primeravez){
		odometriafalsa=input; 
		primeravez=false;
	}
	
	//Aumentamos el error si el robot esta moviendose con velocidad lineal positiva en el eje x
	if(input.twist.twist.linear.x>0.02){
		//Obtenemos el ángulo de la posición real del robot para modificar x e y dependiendo de este
		float angulo=tf::getYaw(odometriafalsa.pose.pose.orientation);
		//angulo+=errortotaltheta;
		//Aumentamos el error acumulado de las dos dimensiones
		errortotalx+=(erroralmoverse*cos(angulo));
		errortotaly+=(erroralmoverse*sin(angulo));	
	}

	//Aumentamos el error si el robot esta moviendose con velocidad lineal negativa en el eje x
	if(input.twist.twist.linear.x<-0.02){
		//Obtenemos el ángulo de la posición real del robot para modificar x e y dependiendo de este
		float angulo=tf::getYaw(odometriafalsa.pose.pose.orientation);
		//angulo+=errortotaltheta;
		//Aumentamos el error acumulado de las dos dimensiones
		errortotalx-=(erroralmoverse*cos(angulo));
		errortotaly-=(erroralmoverse*sin(angulo));	
	}

	//Aumentamos el error en la rotación en caso de que este rote en sentido antihorario
	if(input.twist.twist.angular.z>0.02){
		errortotaltheta+=erroralrotar;
	}
	
	//Aumentamos el error en la rotación en caso de que este rote en sentido horario
	if(input.twist.twist.angular.z<-0.02){
		errortotaltheta-=erroralrotar;
	}

	//Añadimos el error acumulado a la posición del robot real
	odometriafalsa.pose.pose.position.x+=errortotalx; //Calculamos el valor de la odometria en el eje x
	odometriafalsa.pose.pose.position.y+=errortotaly; //Calculamos el valor de la odometria en el eje y
	
	//Definimos un quaternion donde le metemos como yaw la rotación de la posición real + el error
	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY( 0, 0, tf::getYaw(odometriafalsa.pose.pose.orientation)+errortotaltheta); 
	odometriafalsa.pose.pose.orientation.x=myQuaternion[0];	
	odometriafalsa.pose.pose.orientation.y=myQuaternion[1];
	odometriafalsa.pose.pose.orientation.z=myQuaternion[2];
	odometriafalsa.pose.pose.orientation.w=myQuaternion[3];

	
	//Para dejarla sin error
	//odometriafalsa=input;
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
