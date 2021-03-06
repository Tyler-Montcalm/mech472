
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer5.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;


//void distance_point_line(int label1, double slope, double b);
double distance_point_point(double x1, double y1, double x2, double y2);
void laser_to_hunted(double x_laser, double y_laser, double x_hunted, double y_hunted, double angle_hunter);
void equation_line_without_label(double x1, double x2, double y1, double y2, double &slope_return, double &b_return);
double angle_point_point(double x1, double x2, double y1, double y2);
double carangle(int xfront, int yfront, int xback, int yback);
void bestxy(double x_hunter, double x_obstacle, double y_hunter, double y_obstacle, double r_obstacle, double &x_best, double &y_best);

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	
	double slope_return, b_return; // used to store the variables of the eqn of a line
	double fxy[50];

	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs  = 2;

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 135; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
	
		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
	
	cout << "\npress space key to begin program.";
	pause();


	activate_vision();

	
	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	


	mode = 1;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);

	
	image rgb;
	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	// allocate memory for the images
	allocate_image(rgb);

	wait_for_player();

	// measure initial clock time
	tc0 = high_resolution_time(); 
	double x_best, y_best;
	while(1) {
		
		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;
		bestxy(25.0, 150.0, 25.0, 150.0, 20.0, x_best, y_best);
		laser_to_hunted(30, 30, 0, 0, 0); // this function takes the x,y coord of you and your opponent and aims laser to him if possible else closest
		
		
		view_rgb_image(rgb);

		
		Sleep(1000); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}

/*void distance_point_line(int label1,double slope,double b)
{
	double inverse_slope = 1 / slope;
	double b_inverse = ((y_obs[label1]) - (inverse_slope*(x_obs[label1])));
	// equation of inverse line is y=inverse_slope*x+b_inverse
	// to find distance we need intersection between original line and new line
	// system of equations:
	double Dx = ((b - b_inverse) / (slope - inverse_slope));
	double Dy = (slope*Dx) + b;
	return distance_point_point(Dx, Dy, x_obs[label1], y_obs[label1]);

}
*/
void bestxy(double x_hunter, double x_obstacle, double y_hunter, double y_obstacle,double r_obstacle,double &x_best,double &y_best)
{
	double angle_between_hunter_obstacle; 
	double x_to_move, y_to_move;
	angle_between_hunter_obstacle = carangle(x_hunter, y_hunter, x_obstacle, y_obstacle);
	//cout << "\n" << "angle between hunter obstacle: " << angle_between_hunter_obstacle;
	x_to_move = (sin(angle_between_hunter_obstacle*.01745))*r_obstacle;
	//cout << "\n" << "x to move: " << x_to_move;
	if (x_hunter >= x_obstacle) // if hunter is more right you want to be more left of obstacle
	{
		//cout << "hunter>xobst";
		x_best = x_obstacle - x_to_move;
	}
	else // if hunter is more left you want to be more right of obstacle
	{
		x_best = x_obstacle + x_to_move;
		//cout << "hunter<xobst";
	}
	y_to_move = (cos(angle_between_hunter_obstacle*.01745))*r_obstacle;
	//cout << "\n" << "y to move: " << y_to_move;
	if (y_hunter >= y_obstacle) // if hunter is more north you want to be more south of obstacle
	{
		y_best = y_obstacle - y_to_move;
		//cout << "hunter>yobst";
	}
	else // if hunter is more south you want to be more north of obstacle
	{
		y_best = y_obstacle + y_to_move;
		//cout << "hunter<yobst";
	}
	cout << "\n" << "x: " << x_best << "y: " << y_best;
	
}


double angle_point_point(double x1, double x2, double y1, double y2)
{
	double angle = atan((x2 - x1) / (y2 - y1));
	return angle;
}

void laser_to_hunted(double x_laser,double y_laser,double x_hunted,double y_hunted, double angle_hunter) // this function takes the x,y coord of you and your opponent and aims laser to him if possible else closest
{
	double angle_right, angle_left,angle_hunted; // these are the max limits the laser can shoot
	double degree_per_pulse_width = 500 / 90;
	double light = 1.0;
	double light_gradient = 1.0;
	double light_dir = 1.0;
	double image_noise = 1.0;
	double max_speed = 100; // max wheel speed of robot (pixels/s)
	double opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	angle_left = angle_hunter - 90;
	//cout << "Angle left = " << angle_left;
	if (angle_left < 0)
	{
		//angle_left += 360;
	}
	angle_right = angle_hunter + 90;
	if (angle_right < 0)
	{
		angle_right += 360;
	}
	angle_hunted = carangle(x_laser, y_laser, x_hunted,y_hunted)-180;
	
	bool obstacle;
	obstacle = false;
	//cout << "at angle: " << angle_hunted;
	if ((angle_hunted >= angle_left) && (angle_hunted <= angle_right))
	{
		int pw_laser = ((angle_hunted)*(degree_per_pulse_width))+1500;
		if (obstacle == false) // change this to a function call for ryans function
		{ //turn to angle and fire
			//cout << "\n Turning laser to 45 degrees left";
			set_inputs(1500, 1500, pw_laser, 1, 
				light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);
			
		}
		else
		{ //turn to angle dont fire
			
			set_inputs(1500, 1500, pw_laser, 0,
				light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);
		}
	}
	else
	{
		set_inputs(1750, 1750, 0, 0,
			1, 1, 1, 1,
			100, 100);
		// its behind you so you cant get there anyway
	}
}
void equation_line_without_label(double x1, double x2,double y1,double y2, double &slope_return, double &b_return)
{
	double slope;
	double b;
	slope = ((y2) - (y1)) / ((x2) - (x1));
	b = (y2) - (slope*(x2));
	//equation of line = y=slope*x+b
	slope_return = slope;
	b_return = b;
}

double carangle(int xfront, int yfront, int xback, int yback) {
	double PI = 3.14159;
	double temp = atan2(yfront - yback, xfront - xback) * 180 / PI;
	if (temp < 0) temp += 360;
	return temp;
}

