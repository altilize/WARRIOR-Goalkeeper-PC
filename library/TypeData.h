#ifndef TYPE_DATA_H
#define TYPE_DATA_H

#include <string>
#include <map>

using namespace std;

struct Parameter{
    string name;
    int value;
    int defaultValue;

    int min;
    int max;
    int step;
};

struct Camera{
    string name;
    string path;
    int index = 0;
    map<string, Parameter> parameters;
};

struct pcData
{
	unsigned char HEADER = 254;
	int MOTION;
	int HEADING;
	int KECEPATAN;
	int TENDANG;
	int HANDLER;
	int GRIDX = 0;
	int GRIDY = 0;
	bool CHANGEGRID = false;
	int STATUS_ROBOT = 0;
	int VZ;
	int VX;
	int VY;
	int KICK_MODE;
	int RESET_COMPASS;
	bool RESET_COMPASS_SIGN;
	bool isLineFollower = false;
};

struct STM32Data
{
	unsigned char HEADER;
	int KOMPAS;
	int SIGNKOMPAS;
	int BUTTON;
	bool IR_MUSUH_1;
	bool IR_MUSUH_2;
	bool IR_MUSUH_3;
	bool MUSUH;
	bool BOLA;
	int GRIDX;
	int GRIDY;
	float XPOS;
	float YPOS;
	bool isLidar;
};

struct BasestationData
{
	int kompasFlag;
	int valueCompas;
	int gridX;
	int gridY;
	int cGridX;
	int cGridY;
	int skillRobot;
	int manualRobot;
};

#endif