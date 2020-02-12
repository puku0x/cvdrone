#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------


cv::Mat tmpl;   //Mat�^��tmpl��p�ӂ�

int main(int argc, char *argv[])
{
	// AR.Drone class
	ARDrone ardrone;

	// Initialize
	if (!ardrone.open()) {
		std::cout << "Failed to initialize." << std::endl;
		return -1;
	}

	// Battery
	std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

	// Instructions
	std::cout << "***************************************" << std::endl;
	std::cout << "*       CV Drone sample program       *" << std::endl;
	std::cout << "*           - How to play -           *" << std::endl;
	std::cout << "***************************************" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "* - Controls -                        *" << std::endl;
	std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
	std::cout << "*    'Up'    -- Move forward          *" << std::endl;
	std::cout << "*    'Down'  -- Move backward         *" << std::endl;
	std::cout << "*    'Left'  -- Turn left             *" << std::endl;
	std::cout << "*    'Right' -- Turn right            *" << std::endl;
	std::cout << "*    'Q'     -- Move upward           *" << std::endl;
	std::cout << "*    'A'     -- Move downward         *" << std::endl;
	std::cout << "*    'B'     -- auto-up-and-down      *" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "* - Others -                          *" << std::endl;
	std::cout << "*    'C'     -- Change camera         *" << std::endl;
	std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "***************************************" << std::endl;


	////////////////////////////2020/02/06
	tmpl = cv::imread("template_circle.png", 0); //�e���v���[�g���O���[�X�P�[���œǂݍ���
	////////////////////////////

	while (1) {
		// Key input
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		// Get an image
		cv::Mat image = ardrone.getImage();

		// Take off / Landing 
		if (key == ' ') {
			if (ardrone.onGround()) ardrone.takeoff();
			else                    ardrone.landing();
		}

		// Move
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		switch (key) {
		case CV_VK_UP:		vx = 1.0; break;
		case CV_VK_DOWN:	vx = -1.0; break;
		case CV_VK_LEFT:	vy = 1.0; break;
		case CV_VK_RIGHT:	vy = -1.0; break;
		case 'j':	vy = 1.0; break;
		case 'l':	vy = -1.0; break;
		case 'q':	vz = 1.0; break;
		case 'a':	vz = -1.0; break;
		case 'b':
			double vx_now, vy_now;
			double const init_alt = ardrone.getAltitude();

			std::cout << "altitude = " << init_alt << std::endl;
			while (ardrone.getAltitude() < init_alt + 0.3) {
				ardrone.getVelocity(&vx_now, &vy_now, 0);
				ardrone.move3D(-vx_now, -vy_now, 1.0, 0.0);
				std::cout << "\r" << "altitude = " << ardrone.getAltitude();
			}
			while (ardrone.getAltitude() > init_alt) {
				ardrone.getVelocity(&vx_now, &vy_now, 0);
				ardrone.move3D(-vx_now, -vy_now, -1.0, 0.0);
				std::cout << "\r" << "altitude = " << ardrone.getAltitude();
			}
			ardrone.move3D(0.0, 0.0, 1.0, 0.0);
			std::cout << "\r" << "command b end.            ";
			break;
		}
		ardrone.move3D(vx, vy, vz, vr);

		// Change camera
		static int mode = 0;
		if (key == 'c') ardrone.setCamera(++mode % 4);

		// Display the image
		cv::imshow("camera", ardrone.detectCircle(image));
		//cv::imshow("camera", temp_match(image));
		if (ardrone.getBatteryPercentage() < 15) {
			std::cout << "Battery low !" << std::endl;
			ardrone.landing();
			break;
		}
	}

	// See you
	ardrone.close();

	return 0;
}
