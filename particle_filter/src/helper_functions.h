/*
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

/*
 * Struct representing one position/control measurement.
 */
struct control_s {
	
	double velocity;	// Velocity [m/s]
	double yawrate;		// Yaw rate [rad/s]
};

/*
 * Struct representing one ground truth position.
 */
struct ground_truth {
	
	double x;		// Global vehicle x position [m]
	double y;		// Global vehicle y position
	double theta;	// Global vehicle yaw [rad]
};

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
	
	int id;				// Id of matching landmark in the map.
	double x;			// Local (vehicle coordinates) x position of landmark observation [m]
	double y;			// Local (vehicle coordinates) y position of landmark observation [m]
};

struct Histogram {				
	double R1;			
	double G1;
	double B1;
	double R2;			
	double G2;
	double B2;
	double R3;			
	double G3;
	double B3;
	double R4;			
	double G4;
	double B4;
			
};

/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double * getError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {
	static double error[3];
	error[0] = fabs(pf_x - gt_x);
	error[1] = fabs(pf_y - gt_y);
	error[2] = fabs(pf_theta - gt_theta);
	error[2] = fmod(error[2], 2.0 * M_PI);
	if (error[2] > M_PI) {
		error[2] = 2.0 * M_PI - error[2];
	}
	return error;
}



/* Reads control data from a file.
 * @param filename Name of file containing control measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_control_data(std::string filename, std::vector<control_s>& position_meas) {

	// Get file of position measurements:
	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_pos) {
		return false;
	}

	// Declare single line of position measurement file:
	std::string line_pos;

	// Run over each single line:
	while(getline(in_file_pos, line_pos)){

		std::istringstream iss_pos(line_pos);

		// Declare position values:
		double velocity, yawrate;

		// Declare single control measurement:
		control_s meas;

		//read data from line to values:

		iss_pos >> velocity;
		iss_pos >> yawrate;

		
		// Set values
		meas.velocity = velocity;
		meas.yawrate = yawrate;

		// Add to list of control measurements:
		position_meas.push_back(meas);
	}
	return true;
}

/* Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool read_gt_data(std::string filename, std::vector<ground_truth>& gt) {

	// Get file of position measurements:
	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_pos) {
		return false;
	}

	// Declare single line of position measurement file:
	std::string line_pos;

	// Run over each single line:
	while(getline(in_file_pos, line_pos)){

		std::istringstream iss_pos(line_pos);

		// Declare position values:
		double x, y, azimuth;

		// Declare single ground truth:
		ground_truth single_gt; 

		//read data from line to values:
		iss_pos >> x;
		iss_pos >> y;
		iss_pos >> azimuth;

		// Set values
		single_gt.x = x;
		single_gt.y = y;
		single_gt.theta = azimuth;

		// Add to list of control measurements and ground truth:
		gt.push_back(single_gt);
	}
	return true;
}

/* Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_landmark_data(std::string filename, std::vector<LandmarkObs>& observations) {

	// Get file of landmark measurements:
	std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_obs) {
		return false;
	}

	// Declare single line of landmark measurement file:
	std::string line_obs;

	// Run over each single line:
	while(getline(in_file_obs, line_obs)){

		std::istringstream iss_obs(line_obs);

		// Declare position values:
		double local_x, local_y;

		//read data from line to values:
		iss_obs >> local_x;
		iss_obs >> local_y;

		// Declare single landmark measurement:
		LandmarkObs meas;

		// Set values
		meas.x = local_x;
		meas.y = local_y;

		// Add to list of control measurements:
		observations.push_back(meas);
	}
	return true;
}

inline bool read_histogram_data(std::string filename, std::vector<Histogram>& observations) {

	// Get file of landmark measurements:
	std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_obs) {
		return false;
	}

	// Declare single line of landmark measurement file:
	std::string line_obs;

	// Run over each single line:
	while(getline(in_file_obs, line_obs)){

		std::istringstream iss_obs(line_obs);

		// Declare position values:
		double R1,R2,R3,R4,G1,G2,G3,G4,B1,B2,B3,B4;

		//read data from line to values:
		iss_obs >> B1;
		iss_obs >> G1;
		iss_obs >> R1;
		iss_obs >> B2;
		iss_obs >> G2;
		iss_obs >> R2;
		iss_obs >> B3;
		iss_obs >> G3;
		iss_obs >> R3;
		iss_obs >> B4;
		iss_obs >> G4;
		iss_obs >> R4;

		// Declare single landmark measurement:
		Histogram h;

		// Set values
		h.B1 = B1;
		h.G1 = G1;
		h.R1 = R1;
		h.B2 = B2;
		h.G2 = G2;
		h.R2 = R2;
		h.B3 = B3;
		h.G3 = G3;
		h.R3 = R3;
		h.B4 = B4;
		h.G4 = G4;
		h.R4 = R4;

		// Add to list of control measurements:
		observations.push_back(h);
	}
	return true;

}

#endif /* HELPER_FUNCTIONS_H_ */
