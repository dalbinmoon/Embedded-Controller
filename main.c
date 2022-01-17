#include "manage.h"

int main()
{
	setup_final();
	printf("setup done!\r\n");
	BT_write((uint8_t *)"Hello Bluetooth\r\n", 17);
	
	while(1)
	{
		
		switch(state)
		{
			case 0: wait_BT(); break;
			case 1: lane_tracker(); break;
			case 2: park_vehicle(); break;
			
			case 9: sensor_testing(); break;
			default: break;
		}

	}
	drive('x');
}

