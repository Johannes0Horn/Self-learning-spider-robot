
#include "motorcontrol.h"

int main(void)
{
	Init();
	/*PARAMS: motornumber, destinationPosition, clamp?*/
	changeMotorDestinationState(1,  90, false);
	changeMotorDestinationState(2, 100, false);
	changeMotorDestinationState(3, 110, false);

	goForDestinationPositions(false); // meaning of bool: if without Timing


	while (1)
	{
		
	}
	return 0;
}
