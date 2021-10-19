#include "schnittstelle.h"
#include "lib/stackmon.h"

char usartReceiveStringBuffer[255];
char *buffer_ptr;
float destintionstates[18];

void initSchnittstelle()
{
	USART_init(MYUBRR);
	char pvar[5];

	sprintf(pvar, "S");
	USART_println(pvar);

	main_init();
	sprintf(pvar, "I");
	USART_println(pvar);
	_delay_ms(3000);
}

void waitForSerialCommandAndExecuteAndSendStatus()
{
	//empty Buffer
	strncpy(usartReceiveStringBuffer, "", 255);
	//USART should already be initialised
	//USART_init(MYUBRR);

	double degree = 0;
	int motornumber = 0;
	char *partLvl1;
	char *partLvl2;

	USART_getLine(usartReceiveStringBuffer, 255);
	//usartReceiveStringBuffer looks like this: #1 90;#3 60;#4 70;... OR is "init"

	//set pointer to string begin
	buffer_ptr = usartReceiveStringBuffer;
	//char * init = "init";

	if (strstr(buffer_ptr, "init") != NULL)
	{
		//init
		//Serial Debug purposes
		//USART_println("init");
		main_init();
	}

	else
	{
		//usartReceiveStringBuffer looks like this: #1 90;#3 60;#4 70;...
		//array to safe destintionstates in
		while ((partLvl1 = strsep(&buffer_ptr, ";")))
			if (strlen(partLvl1) > 1)
			{
				{
					//Serial Debug purposes
					/*
						USART_println("partLvl1:");
						USART_println(partLvl1);
						*/
					//partLvl1 --> "#1 57.296"
					while ((partLvl2 = strsep(&partLvl1, " ")))
					{
						//Serial Debug purposes
						/*
							USART_println("partLvl2:");
							USART_println(partLvl2);
							*/
						//partLvl2 --> "#1" or "57.296"

						if (strchr(partLvl2, '#') != NULL)
						{

							//partLvl2 = motornumber inlc. '#', e.g. "#0"

							//remove # and convert to int
							motornumber = atol(partLvl2 + 1);

							//Serial Debug purposes
							/*
								char motornumber_str[12];
								sprintf(motornumber_str, "%d", motornumber);
								USART_println("motornumber:");
								USART_println(motornumber_str);
							*/
						}
						else
						{
							//partLvl2 = degree, e.g. "57.296"
							//todo convert partLvl2 to float instead of string
							degree = atof(partLvl2);

							//Serial Debug purposes
							/*
								char degree_str[12];
								dtostrf(degree, 12, 4, degree_str);
								USART_println("degree_str:");
								USART_println(degree_str);
							*/
							destintionstates[motornumber] = (float)degree;
						}
					}
				}
			}

		struct Status status = main_step(destintionstates);
		sendStatus(status);
	}
	//empty Buffer
	strncpy(usartReceiveStringBuffer, "", 255);
}

void remove_spaces(char *s)
{
	const char *d = s;
	do
	{
		while (*d == ' ')
		{
			++d;
		}
	} while ((*s++ = *d++));
}

void sendStatus(struct Status status)
{
	double ultrasonicDistance = status.distance;
	int IRDegree = status.direction;
	struct GyroData gyroData = status.gyro_data;

	//convert double ultrasonicDistance to str
	char ultrasonicDistance_str[12];
	dtostrf(ultrasonicDistance, 12, 4, ultrasonicDistance_str);
	remove_spaces(ultrasonicDistance_str);

	//convert int IRDegree to string
	char IRDegree_str[8];
	snprintf(IRDegree_str, 8, "%d", IRDegree);

	//convert gyroDataInts into strings
	char gyroDataInts_str[100];
	gyroDataInts_str[0] = '\0';
	sprintf(gyroDataInts_str, "%d;%d;%d;%d;%d;%d;%d", gyroData.accelerationX, gyroData.accelerationY,
			gyroData.accelerationZ, gyroData.temprature, gyroData.rotAccelX, gyroData.rotAccelY,
			gyroData.rotAccelZ);

	//convert gyroDataDoubles into strings
	char rotationX_str[10];
	dtostrf(gyroData.rotationX, 8, 4, rotationX_str);
	remove_spaces(rotationX_str);

	char rotationY_str[10];
	dtostrf(gyroData.rotationY, 8, 4, rotationY_str);
	remove_spaces(rotationY_str);

	char rotationZ_str[10];
	dtostrf(gyroData.rotationZ, 8, 4, rotationZ_str);
	remove_spaces(rotationZ_str);

	//put together
	char status_str[255];
	status_str[0] = '\0';
	strcat(status_str, "!");
	strcat(status_str, ultrasonicDistance_str);
	strcat(status_str, ";");
	strcat(status_str, IRDegree_str);
	strcat(status_str, ";");
	strcat(status_str, gyroDataInts_str);
	strcat(status_str, ";");
	strcat(status_str, rotationX_str);
	strcat(status_str, ";");
	strcat(status_str, rotationY_str);
	strcat(status_str, ";");
	strcat(status_str, rotationZ_str);
	strcat(status_str, "§");

	//send status
	USART_println(status_str);
}
