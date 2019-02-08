#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
//Definimos el publisher
ros::Publisher pubodometryfalsa;

//Definimos los errores 
double errortotalx=0;
double errortotaly=0;
double errortotaltheta=0;
//Definimos una variable para ir incrementando el error en el incremento definido
//float erroralmoverse=0.000155; //erroralmoverse*65=error por segundo=0.01 m/s de desviación
//float erroralrotar=0.000769; //Erroralrotar*65=erroranguloporsegundo=0.05 radianes/s=3º/s

double erroralmoverse=0.000155;//0.01 m/s
double erroralrotar=0.000769; //6º/s

//Objeto que almacena la odometria modificada
nav_msgs::Odometry odometriafalsa;
bool primeravez=true;
nav_msgs::Odometry inputanterior;

void callbackOdom (const nav_msgs::Odometry input){
 	errortotaltheta=0;
	errortotalx=0;
	errortotaly=0;
	double diferencialineal=0;
	double diferenciaangular=0;
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
		diferencialineal=abs(input.pose.pose.position.x-inputanterior.pose.pose.position.x)+abs(input.pose.pose.position.y-inputanterior.pose.pose.position.y);
		errortotalx+=(diferencialineal+erroralmoverse)*cos(angulo);
		errortotaly+=(diferencialineal+erroralmoverse)*sin(angulo);	
	}

	//Aumentamos el error si el robot esta moviendose con velocidad lineal negativa en el eje x
	if(input.twist.twist.linear.x<-0.02){
		//Obtenemos el ángulo de la posición real del robot para modificar x e y dependiendo de este
		float angulo=tf::getYaw(odometriafalsa.pose.pose.orientation);
		diferencialineal=abs(input.pose.pose.position.x-inputanterior.pose.pose.position.x)+abs(input.pose.pose.position.y-inputanterior.pose.pose.position.y);
		//angulo+=errortotaltheta;
		//Aumentamos el error acumulado de las dos dimensiones
		errortotalx-=((diferencialineal+erroralmoverse)*cos(angulo));
		errortotaly-=((diferencialineal+erroralmoverse)*sin(angulo));	
	}

	//Aumentamos el error en la rotación en caso de que este rote en sentido antihorario
	if(input.twist.twist.angular.z>0.02){
		diferenciaangular=tf::getYaw(input.pose.pose.orientation)-tf::getYaw(inputanterior.pose.pose.orientation);
		errortotaltheta+=erroralrotar;//+diferenciaangular;
	}
	
	//Aumentamos el error en la rotación en caso de que este rote en sentido horario
	if(input.twist.twist.angular.z<-0.02){
		diferenciaangular=abs(tf::getYaw(input.pose.pose.orientation)-tf::getYaw(inputanterior.pose.pose.orientation));
		errortotaltheta-=erroralrotar;//+diferenciaangular;
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
