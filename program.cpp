
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
double carangle(int xfront, int yfront, int xback, int yback);
void convert(int pwl, int pwr);
void move(double xcurrent, double ycurrent, double xback, double yback, int xdesired, int ydesired);
int rgbthreshold(image &rgb, image &output, int Rin, int Gin, int Bin);
extern robot_system S1;
int pw_l, pw_r;
int main()
{
	
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int  pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	
	// TODO: it might be better to put this model initialization
	// section in a separate function
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
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
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
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

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file
	
	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 1;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
//	x0 = 150;
//	y0 = 375;
//	theta0 = 3.14159/4;
//	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1250; // pulse width for left wheel servo (us)
	pw_r = 2000; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	// NOTE: for two player mode you shouldn't set the opponent inputs 

	// opponent inputs
//	pw_l_o = 1300; // pulse width for left wheel servo (us)
//	pw_r_o = 1600; // pulse width for right wheel servo (us)
//	pw_laser_o = 1500; // pulse width for laser servo (us)
//	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
//	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
//				opponent_max_speed);

	// regular vision program ////////////////////////////////
	
	// note that at this point you can write your vision program
	// exactly as before.
	
	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.
	
	image rgb;
	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
	height = 480;
	double ox = 0;
	double oy = 0;
	double bx = 0;
	double by = 0;
	double gx = 0;
	double gy = 0;
	double rx = 0;
	double ry = 0;
	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;
	image a;
	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;
	allocate_image(a);
	image b;
	b.type = LABEL_IMAGE;
	b.width = width;
	b.height = height;
	allocate_image(b);
	// allocate memory for the images
	allocate_image(rgb);

	wait_for_player();
	int labels = 0;
	// measure initial clock time
	tc0 = high_resolution_time(); 
	//PLAYER1 :
	//Head : Green R = 70, G = 180, B = 133
	//Tail : Red R = 227, G = 91, B = 78
	//PLAYER 2:
   // Head: Orange R = 255, G = 190, B = 125
	//Tail : Blue R = 50, G = 160, B = 230

	
	while(1) {
		//int label_image(image &a, image &b, int &nlabels)
			// labels a binary image
			// labels go from 1 to nlabels
			// a - binary GREY_SCALE image type (pixel values must be 0 or 255)
			// b - LABEL_IMAGE type (pixel values are 0 to 65535)
		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);
		scale(rgb, rgb);
		rgbthreshold(rgb, a, 255, 190, 125); //Orange
		//erode(a, a);
		//dialate(a, a);
		label_image(a, b, labels);
		centroid(a, b, 1, ox, oy);
		rgbthreshold(rgb, a, 50, 160, 230);	//Blue
		//erode(a, a);
		//dialate(a, a);
		label_image(a, b, labels);
		centroid(a, b, 1, bx, by);
		rgbthreshold(rgb, a, 70, 180, 133);//Green
		//erode(a, a);
		//dialate(a, a);
		label_image(a, b, labels);
		centroid(a, b, 1, gx, gy);
		rgbthreshold(rgb, a, 227, 91, 78);//Red
		//erode(a, a);
		//dialate(a, a);
		label_image(a, b, labels);
		centroid(a, b, 1, rx, ry);
		move(rx, ry, gx, gy, 300, 300);
		printf("%lf", rx);
		printf(",");
		printf("%lf", ry);
		printf("\n");

		//convert(2000, 2000);
		tc = high_resolution_time() - tc0;

		// fire laser
		if(tc > 1) laser = 1;
		
//		if(tc > 9) laser_o = 1;

		// turn off the lasers so we can fire it again later
		if(tc > 10) { 
			laser = 0;
//			laser_o = 0;
		}
		
		// fire laser at tc = 14 s
		if(tc > 14) {
			laser = 1;
			
			// turn laser angle alpha at the same time
			pw_laser = 1000;
		}

		// change the inputs to move the robot around
		// or change some additional parameters (lighting, etc.)
		
		// only the following inputs work so far
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels
		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
//		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
//					opponent_max_speed);

		// NOTE: only one program can call view_image()
		view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}

int rgbthreshold(image &rgb, image &output, int Rin, int Gin, int Bin)
// binary threshold operation
// a - rgb image
// b - binary image

{
	int i, j, k;
	int height, width; // ints are 4 bytes on the PC
	ibyte *p; // pointer to colour components in the rgb image
	ibyte *ph; // pointer to the hue image
	int R, G, B, hint;

	height = rgb.height;
	width = rgb.width;

	p = rgb.pdata;
	ph = output.pdata;

	// check for compatibility of a, b
	if (rgb.height != output.height || rgb.width != output.width) {
		printf("\nerror in threshold: sizes of a, b are not the same!");
		return 1;
	}
	for (j = 0; j < height; j++) {

		for (i = 0; i < width; i++) {

			// equivalent 1D array index k
			k = j * width + i; // pixel number

			// how to get j and i from k ?
			// i = k % width
			// j = (k - i) / width

			// 3 bytes per pixel -- colour in order BGR
			B = p[k * 3]; // 1st byte in pixel
			G = p[k * 3 + 1]; // 2nd byte in pixel
			R = p[k * 3 + 2]; // 3rd

			if (B >= Bin - 20 && B <= Bin + 20 && G >= Gin - 20 && G <= Gin + 20 && R >= Rin - 20 && R <= Rin + 20) {
				ph[k] = 255;
			}
			else {
				ph[k] = 0;
			}
		}
	}
	return 0;
}
void move(double xcurrent, double ycurrent, double xback, double yback, int xdesired, int ydesired) {

	int pwr = 1500; //0 wheel movement as a default.
	int pwl = 1500;
	float distance = sqrt((xdesired - xcurrent)*(xdesired - xcurrent) + (ydesired - ycurrent)*(ydesired - ycurrent));
	double anglerequired = atan2(xdesired - xcurrent, ydesired - ycurrent);
	double anglediff = anglerequired - carangle(xcurrent, ycurrent, xback, yback);
	if (abs(anglediff) <= 90 || abs(anglediff) >= 270) { //going forward should be faster in this case
		if (anglediff >= 0) {
			pwr = 2000;
			pwl = 1500 + (abs(anglediff) * 500 / 90); //Scales rotation based on angle change required.
			//pwr = 1500;
			//pwl = 1500;
		}
		else {
			pwl = 2000;
			pwr = 1500 +  (abs(anglediff) * 500 / 270);
			//pwl = 1500;
			//pwr = 1500;
		}
	}
	else {
		if (180 - abs(anglediff) <= 0) {
			pwl = 1000;
			pwr = 1000 + (abs(anglediff)-180) * 500 / 90;
		    pwr = 1500;
			pwl = 1500;
		}
		else {
			pwr = 1000;
			pwl = 1000 + (180 - abs(anglediff)) * 500 / 90;
			//pwr = 1500;
			//pwl = 1500;
		}
	}
	convert(pwl, pwr);
}

void convert(int pwl, int pwr) { //function so that 2000 is positive for both wheels.
	pw_r = pwr;
	pw_l = 1500 + (1500 - pwl);
}
double carangle(int xfront, int yfront, int xback, int yback) {
	return atan2(xfront - xback, yfront - yback);
}