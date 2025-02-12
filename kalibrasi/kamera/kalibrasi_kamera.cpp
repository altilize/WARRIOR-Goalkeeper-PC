#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

int INDEX_KAMERA=0;

using namespace std;
using namespace cv;

class Parameter{
	public:
		int brightness = 120;
		int contrast = 45;
		int saturation = 119;
		int gain = 43;
		int backlight_compensation=0;
		int white_balance_temperature_auto;
		int white_balance_temperature=5700;
		int power_line_frequency;
		int sharpness=22;
		int exposure_auto=1;
		int exposure_absolute = 400;
};

void perintah(string perintah);
void aturKamera(int index, Parameter param);

void print_menu(int index, Parameter param)
{
	if(index == 0)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("brightness : %i\n", param.brightness);

	if(index == 1)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("contrast : %i\n", param.contrast);

	if(index == 2)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("saturation : %i\n", param.saturation);

	if(index == 3)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("white_balance_temperature_auto : %i\n", param.white_balance_temperature_auto);

	if(index == 4)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("gain : %i\n", param.gain);

	if(index == 5)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("power_line_frequency : %i\n", param.power_line_frequency);

	if(index == 6)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("white_balance_temperature : %i\n", param.white_balance_temperature);

	if(index == 7)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("sharpness : %i\n", param.sharpness);

	if(index == 8)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("backlight_compensation : %i\n", param.backlight_compensation);

	if(index == 9)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("exposure_auto : %i\n", param.exposure_auto);

	if(index == 10)
	{
		printf("  >  \t");
	}
	else
	{
		printf("     \t");
	}
	printf("exposure_absolute : %i\n", param.exposure_absolute);
}

void ubah_nilai(int index, int val_change, Parameter& param)
{
	if(index == 0)
	{
		param.brightness = param.brightness+val_change;
	}

	if(index == 1)
	{
		param.contrast = param.contrast+val_change;
	}

	if(index == 2)
	{
		param.saturation = param.saturation+val_change;
	}

	if(index == 3)
	{
		param.white_balance_temperature_auto = param.white_balance_temperature_auto+val_change;
	}

	if(index == 4)
	{
		param.gain = param.gain+val_change;
	}

	if(index == 5)
	{
		param.power_line_frequency = param.power_line_frequency+val_change;
	}

	if(index == 6)
	{
		param.white_balance_temperature = param.white_balance_temperature+val_change;
	}

	if(index == 7)
	{
		param.sharpness = param.sharpness+val_change;
	}

	if(index == 8)
	{
		param.backlight_compensation = param.backlight_compensation+val_change;
	}

	if(index == 9)
	{
		param.exposure_auto = param.exposure_auto+val_change;
	}

	if(index == 10)
	{
		param.exposure_absolute = param.exposure_absolute+val_change;
	}
}

VideoCapture cap;

bool open_camera = false;

int main(int argc, char* argv[])
{
	if(argc > 1)
	{
		INDEX_KAMERA = argv[1][0] - '0';
	}
	cap.set(3, 480);
	cap.set(4, 640);
	cap.open(INDEX_KAMERA);

	Mat frame, frame2;
	string info_kamera = "v4l2-ctl -d /dev/video";
	info_kamera+=std::to_string(INDEX_KAMERA);
	info_kamera+=" -l";

	Parameter camera;
	camera.exposure_auto = 1;
	camera.backlight_compensation = 0;
	camera.white_balance_temperature_auto = 0;
	camera.white_balance_temperature=5700;
	camera.brightness = 120;
	camera.contrast = 45;
	camera.saturation = 119;
	camera.power_line_frequency = 2;
	camera.sharpness = 22;

	int index = 0;
	char menu;
	while(1)
	{
		cap >> frame;
		imshow("frame", frame);

		system("clear");
		perintah(info_kamera.c_str());
		printf("\n");

		printf("%i\n", index);

		print_menu(index, camera);

		menu = waitKey(10);
		if(menu == 'w')
		{
			index--;
			if(index < 0)
			{
				index = 10;
			}
		}
		else if(menu == 's')
		{
			index++;
			if(index > 10)
			{
				index = 0;
			}
		}
		else if(menu== 'a')
		{
			ubah_nilai(index, -1, camera);
			aturKamera(INDEX_KAMERA, camera);
		}
		else if(menu=='d')
		{
			ubah_nilai(index, 1, camera);
			aturKamera(INDEX_KAMERA, camera);
		}
		else if(menu=='e')
		{
			ubah_nilai(index, 5, camera);
			aturKamera(INDEX_KAMERA, camera);
		}
		else if(menu=='q')
		{
			ubah_nilai(index, -5, camera);
			aturKamera(INDEX_KAMERA, camera);
		}
		else if(menu==27)
		{
			break;
		}
	}

	cap.release();
}

void perintah(string perintah){
	system(perintah.c_str());
}

void aturKamera(int index, Parameter param){
//	printf(": %i\n", index);

	string base = "v4l2-ctl -d /dev/video";
	base+=std::to_string(index);

	string temp;
	temp = base;
	temp += " -c exposure_auto=";
	temp += std::to_string(param.exposure_auto);
	perintah(temp.c_str());

	temp = base;
	temp += " -c exposure_absolute=";
	temp += std::to_string(param.exposure_absolute);
	perintah(temp.c_str());

	temp = base;
	temp += " -c contrast=";
	temp += std::to_string(param.contrast);
	perintah(temp.c_str());

	temp = base;
	temp += " -c saturation=";
	temp += std::to_string(param.saturation);
	perintah(temp.c_str());

	temp = base;
	temp += " -c gain=";
	temp += std::to_string(param.gain);
	perintah(temp.c_str());

	temp = base;
	temp += " -c brightness=";
	temp += std::to_string(param.brightness);
	perintah(temp.c_str());

	temp = base;
	temp += " -c backlight_compensation=";
	temp += std::to_string(param.backlight_compensation);
	perintah(temp.c_str());

	temp = base;
	temp += " -c white_balance_temperature_auto=";
	temp += std::to_string(param.white_balance_temperature_auto);
	perintah(temp.c_str());

	temp = base;
	temp += " -c white_balance_temperature=";
	temp += std::to_string(param.white_balance_temperature);
	perintah(temp.c_str());

	temp = base;
	temp += " -c sharpness=";
	temp += std::to_string(param.sharpness);
	perintah(temp.c_str());
	temp = base;
	temp += " -c focus_auto=0";
	perintah(temp.c_str());
}