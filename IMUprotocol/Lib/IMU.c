#include "imu.h"

float axisAcc[3];
float axisAngVel[3];
float axisAng[3]; 

void dataHandle(uint8_t buffer[])
{
	
	if (buffer[1] == 0x51)
	{
		axisAcc[0] = ((float)((short)((short)(buffer[3] << 8) | buffer[2]))) / 32768.0 * 16.0 * 9.8;
		axisAcc[1] = ((float)((short)((short)(buffer[5] << 8) | buffer[4]))) / 32768.0 * 16.0 * 9.8;
		axisAcc[2] = ((float)((short)((short)(buffer[7] << 8) | buffer[6]))) / 32768.0 * 16.0 * 9.8;
	}
	if (buffer[12] == 0x52)
	{
		axisAngVel[0] = ((float)((short)((short)(buffer[14] << 8) | buffer[13]))) / 32768.0 * 2000.0;
		axisAngVel[1] = ((float)((short)((short)(buffer[16] << 8) | buffer[15]))) / 32768.0 * 2000.0;
		axisAngVel[2] = ((float)((short)((short)(buffer[18] << 8) | buffer[17]))) / 32768.0 * 2000.0;
	}
	if (buffer[23] == 0x53)
	{
		axisAng[0] = ((float)((short)((short)(buffer[25] << 8) | buffer[24]))) / 32768.0 * 180.0;
		axisAng[1] = ((float)((short)((short)(buffer[27] << 8) | buffer[26]))) / 32768.0 * 180.0;
		axisAng[2] = ((float)((short)((short)(buffer[29] << 8) | buffer[28]))) / 32768.0 * 180.0;
	}
}


