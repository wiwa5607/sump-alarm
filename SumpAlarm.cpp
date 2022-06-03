/* SumpAlarm.cpp - Service that monitors simple float switches and alarms if
   a sump pump appears to have failed.

   No guarantees are given or implied. Damages resulting from bugs, faults, or
   malfunctions of this application are not the responsibility of the author.
   Use at your own risk.

   Usage:
   sumpalarm [-v]

   If used without the -v option, the application is run as a daemon and
   will produce no output.

   Using -v will execute the application in the console and write to stdout
   as opposed to a log file.

   The expected configuration includes two float switches.
   Switch0 to be placed between the low and high water mark in the sump pit
   so that it is tripped with the same frequency as the pump engages.
   Switch1 to be placed slightly above the high water mark so that it is
   activated reasonably quickly after a pump or power failure.

   Action scripts to be executed as follows:

   Switch0On   The lower switch is activated. This will happen frequently and
			   is not intended to generate an alarm. It is intended to be used
			   for an informational push such as output to file or syslog or
			   record a database entry for gathering statistics.

   Switch0Off  The lower switch has switched off

   Switch1On   In the intended configuration, this switch will only be activated
			   in the event of a power or pump failure. This is a critical alarm

   Switch1Off  Power or pump function restored, crisis averted.

   Switch2On   Made available for additional redundant switches or additional
			   sump pits up to Switch99

   Switch2Off  ...

   RateChange  There's a sudden change in flow rate as per Switch0 toggle
			   frequency. Could be a rain storm, overland flooding instead of
			   ground water, or an exterior sump has failed and this one is
			   running more frequently to catch up.

   Overdue	   If Switch0 is active and has remained active for longer than
			   expected, according to the running average frequency + the
			   OverdueThreshold parameter, this script executes as a means of
			   providing early warning that there may be a problem with power or
			   pump.

   Environment variables to be set for use in action scripts:

   SAVOLUME    An integer representing the current estimated volume of water in
			   the sump pit at a given time. Calculated using SwitchXLevel and
			   SumpDiameter
   SARATE      An integer representing the number of Litres of water per hour
			   currently flowing into the sump pit. Calculated using the time
			   between Switch0 off (low water) mark and Switch0 on (Switch0Level)
			   mark, and the diameter of the sump pit (SumpDiameter).
			   0 does not necessarily mean that there is no water entering the
			   sump pit. It may also mean that the necessary parameters are not
			   provided.
   SAFREQ      An integer representing the number of seconds between Switch0 ON
			   events, on a running average of FREQ_HISTORY instances.
   SAFREQM	   An integer representing the number of minutes between Switch0 ON
			   events, on a running average of FREQ_HISTORY instances
   SATIMELEFT  An integer representing the estimated number of minutes before the
			   available capacity of the sump pit is filled. This is a VERY rough
			   estimate as weeping tile will add additional capacity and
			   groundwater in-flow will slow as it gets nearer to the groundwater
			   level surrounding the building
   SATIMELEFTM Same as above but in minutes

   Relies on a config file being present at /etc/sumpalarm.conf

   Configuration file example:

   # SumpAlarm Sample Config File. Parameters here are case sensitive and will
   # not be parsed correctly if the case is incorrect.

   # LogFile should be the first entry. Otherwise a default log file will
   # be created before the LogFile line is reached.
   # Logging Levels:
   # 0=Log nothing
   # 1=Log errors
   # 2=Log errors, switch toggles, and config changes
   # 3=Log Everything
   LogFile=/var/log/sumpalarm.log
   LogLevel=3

   # This section is used when an alarm is produced, to estimate the volume
   # of water that the sump is taking on at the time and how much time is
   # available before water will breach the sump measurements in cm, volume in L
   SumpDepth=760
   SumpDiameter=510
   LowWater=114
   HighWater=222

   # This section defines the activation depth of the float switches, in cm from
   # the bottom of the sump, the GPIO Pin that is used as input, and
   # followed by the action scripts. Up to Switch99 is permitted so long as GPIO
   # supports it.
   Switch0Level=200
   Switch0Pin=14
   Switch0Bounce=5
   Switch0On=echo $(date) Switch0On Flow Rate $SARATE L/H Frequency $SAFREQ >> SumpAlarm.log
   Switch0Off=echo $(date) Switch0Off Flow Rate $SARATE L/H Frequency $SAFREQ >> SumpAlarm.log

   Switch1Level=300
   Switch1Pin=15
   Switch1Bounce=5
   Switch1On=echo SUMP FAILURE! $SATIMELEFTM Minutes before flooding! | mail omgomgomg@sumpalarm.com -s "Sump Failure"
   Switch1Off=echo SUMP Restored. Water level receding | mail omgomgomg@sumpalarm.com -s "Sump Restored"

   # This action script executes when the frequency of pump activations changes
   # by more than a configured percentage
   RateChangeAmt=20
   RateChange=echo The rate of flow has changed by 20 percent since last notice. New rate $SARATE Litres per hour | mail info@sumpalarm.com -s "Sump Rate Changed"

   # This script executes if Switch0On is overdue by the 'OverdueThreshold' number of seconds beyond the running average
   OverdueThreshold=120
   Overdue=echo Warning: Sump evacuation is overdue. Possible power or pump failure | mail info@sumpalarm.com -s "Pump activation overdue"

*******************************************************************************

Compile: gcc SumpAlarm.cpp bcm2835.c bcm2835.h -o sumpalarm

Revision History
Date				Author			Notes
May 11-15, 2017     Cory Whitesell	Original application development and testing
May 18, 2017        Cory Whitesell	Updated to reload action scripts following config changes
May 24, 2017		Cory Whitesell	Bug fixes, better control of logging, converted to millimeters as opposed to centimeters for dimensions, for greater accuracy
June 20, 2017		Cory Whitesell  Added Overdue action script and OverdueThreshold parameter. Previous build has run stable for 26 days when stopped for the update
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include "bcm2835.h"
#include <sys/types.h>
#include <sys/stat.h>

// Defaults
#define CONFIGFILE				"/etc/sumpalarm.conf"
#define LOGFILE					"/var/log/sumpalarm.log"
#define FREQ_HISTORY			4
#define BOUNCEDELAY				5

#define BCM2708_PERI_BASE       0x20000000
#define GPIO_BASE               BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

void trim(char *s);
int sa_strcmp(char *s1, const char *s2);	// compare two strings. If the first non-matching character is a null terminator, strings are considered equal (return 0)
void SetEnvironment(struct FloatSwitch s,struct ConfigData cd);
int GetFrequency(struct FloatSwitch s); // get an average frequency at which the sump is running, in seconds
void Action(char *action);
void RefreshConfig(struct ConfigData &cd, bool initial);
void WriteLog(const char *entry,int level);

struct FloatSwitch
{
	int initialized;		// 1=true
	int level;
	int pin;                // GPIO PIN associated with this switch
	char *OnAction;		    // Action string to execute when turned on
	char *OffAction;		// Action string to execute when turned off
	int freq[FREQ_HISTORY];	// history of seconds between activations
	int lastfreq;           // the last frequency that was reported
	int state; 				// 1 on, 0 off
	time_t LastOn;          // the last time the pump switch activated
	time_t LastOff;
	int bouncedelay;		//  time to wait before recognizing a switch toggle
};

struct ConfigData {
	int sumpdepth;
	int sumpdiameter;
	int lowwater;
	int highwater;
	int capacity;
	int vol;
	int rate;
	int freq;
	int ratechangeamt;
	char *ratechange;
	struct FloatSwitch switchlist[100];
	int overduethreshold;
	char *overdue;
};

bool Terminated=false;
bool verbose=false;
int LogLevel=3;		// default to log everything
char LogFileName[1000]=LOGFILE;

char logme[960];

// Handler for signals from OS so that the application can exit gracefully if terminated
void INTHandler(int sig)
{
	char  c;

	signal(sig, SIG_IGN);

	// ctrl-c
	if (sig==SIGINT)
	{
		WriteLog("Process terminated by user.",1);
		Terminated=true;
		return;
	}

	// system shutdown or session terminated
	if (sig==SIGTERM)
	{
		WriteLog("Process terminated by system.",1);
		Terminated=true;
		return;
	}

	// process killed with pskill
	if (sig==SIGKILL||sig==SIGHUP)
	{
		WriteLog("Process killed by system.",1);
		Terminated=true;
		return;
	}

	// Program caused a segmentation fault. Best to catch it than to inexplicably terminate
	if (sig==SIGSEGV)
	{
		WriteLog("Segmentation fault.",1);
		Terminated=true;
		return;
	}
}

int main(int argc, char **argv)
{
	// check for a -v switch. By default this will run as a daemon and does not
	// produce output to stdout or stderr. But if -v is specified it will run
	// in the terminal
	if (argc>=2)
	{
		if (strcmp(argv[1],"-v")==0)
			verbose=true;
	}
	else
	{
		// fork the process to spawn a daemon (service)
		pid_t dpid=fork();
		pid_t session_id;

		// if the daemon fails to fork
		if (dpid < 0)
		{
			printf("Unable to initialize Daemon\n");
			exit(1);
		}

		if (dpid > 0) exit(0); // terminate original process indicating the daemon is started

		umask(0);

		session_id=setsid();	// give the daemon a session ID

		if (session_id <0) exit(1);

		// this service will not have any interaction with a terminal, so close the streams
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);
	}

	// since we will be in an infinite loop, monitor signals to detect interruptions such as ctrl-c
	signal(SIGINT,INTHandler);
	signal(SIGTERM,INTHandler);
	signal(SIGKILL,INTHandler);
	signal(SIGSEGV,INTHandler);

	// Since we will be forking system calls, we will tell the OS that we're a terrible parent
	// and don't care about the welfare of our children. This prevents <defunct>
	// processes resulting from forks
	signal(SIGCHLD, SIG_IGN);

	// initialize the GPIO
	if (!bcm2835_init())
	{
		WriteLog("Unable to initialize GPIO. Use sudo.",1);
		return 2;
	}

	// Declare and initialize non-switch related variables
	int ID, i;
	char cline[65536];

	// temp variables
	int state=0;
	int freqtemp=0;
	bool overduenotice=false;

	struct ConfigData cd;
	// sump dimensions
	cd.sumpdepth=0;
	cd.sumpdiameter=0;
	cd.lowwater=0;
	cd.highwater=0;

	cd.ratechange=NULL;
	cd.ratechangeamt=0;
	cd.overduethreshold=0;
	cd.overdue=NULL;

	time_t t;
	time_t LastConfigCheck;
	time(&LastConfigCheck);

	// set all switches to uninitialized and initialize other variables to zero/NULL
	for (ID=0;ID<100;ID++)
	{
		cd.switchlist[ID].initialized=0;
		cd.switchlist[ID].OnAction=NULL;
		cd.switchlist[ID].OffAction=NULL;
		cd.switchlist[ID].level=0;
		cd.switchlist[ID].pin=0;
		for (i=0;i<FREQ_HISTORY;i++)
			cd.switchlist[ID].freq[i]=0;
		cd.switchlist[ID].lastfreq=0;
		cd.switchlist[ID].state=0;
		cd.switchlist[ID].LastOn=(time_t)0;
		cd.switchlist[ID].LastOff=(time_t)0;
		cd.switchlist[ID].bouncedelay=BOUNCEDELAY;
	}

	RefreshConfig(cd,true);

	if (cd.switchlist[0].initialized==0)
	{
		WriteLog("Error: Switch0 is not configured. Terminating.",1);
		return 1;
	}

	// configure input pins and read initial state
	for (ID=0;ID<100;ID++)
	{
		if (cd.switchlist[ID].initialized)
		{
			bcm2835_gpio_fsel(cd.switchlist[ID].pin, BCM2835_GPIO_FSEL_INPT);
			cd.switchlist[ID].state=bcm2835_gpio_lev(cd.switchlist[ID].pin);
			snprintf(logme,939,"Switch%d Initial state: ",ID);
			if (cd.switchlist[ID].state==HIGH) strcat(logme,"On");
			else strcat(logme,"Off");
			WriteLog(logme,3);
		}
	}

	char envstr[1000];

	if (verbose) WriteLog("Application started",3);
	else WriteLog("Daemon started",3);

	while (!Terminated)
	{
		time(&t);

		// Run the "Overdue" script is the conditions are met. Should Only run once until the situation is resolved rather than every few seconds
		if (cd.switchlist[0].state==HIGH&&overduenotice==false) // improve the performance by splitting conditionals so that the difficult ones aren't evaluated unless necessary
		{
			freqtemp=GetFrequency(cd.switchlist[0]);
			if (t-cd.switchlist[0].LastOff>=freqtemp+cd.overduethreshold&&freqtemp!=0)
			{
				overduenotice=true;
				Action(cd.overdue);
			}
		}

		// check to see if the configuration file has been changed and needs to be reloaded
		if (t-LastConfigCheck>180) // 3 minutes
		{
			LastConfigCheck=t;
			RefreshConfig(cd,false);
		}

		// loop through the initialized switches to see if the state has changed on any of them
		for (ID=0;ID<100;ID++)
		{
			if (cd.switchlist[ID].initialized)
			{
				state=bcm2835_gpio_lev(cd.switchlist[ID].pin);

				if (state!=cd.switchlist[ID].state)
				{

					// switch has changed to 'On'
					if (state==HIGH)
					{
						// don't react if the bounce delay hasn't expired
						if (t-cd.switchlist[ID].LastOff<cd.switchlist[ID].bouncedelay) continue;

						cd.switchlist[ID].state=HIGH;

						snprintf(logme,939,"Switch%d On",ID);
						WriteLog(logme,2);

						for (i=0;i<FREQ_HISTORY-1;i++)
							cd.switchlist[ID].freq[i]=cd.switchlist[ID].freq[i+1];
						if (cd.switchlist[ID].LastOn!=0) // prevent logging if this is the first entry since startup
							cd.switchlist[ID].freq[FREQ_HISTORY-1]=t-cd.switchlist[ID].LastOn;
						cd.switchlist[ID].LastOn=t;

						cd.freq=GetFrequency(cd.switchlist[0]);

						SetEnvironment(cd.switchlist[0],cd);

						Action(cd.switchlist[ID].OnAction);

						if (ID==0&&cd.freq!=0&&cd.ratechange!=NULL)
						{
							if (cd.switchlist[0].lastfreq==0)
							{
								int z=0;
								// only send rate if we've seen FREQ_HISTORY cycles so far
								for (i=0;i<FREQ_HISTORY;i++)
									if (cd.switchlist[0].freq[i]==0) z++;
								if (z==0)
								{
									Action(cd.ratechange);
									cd.switchlist[0].lastfreq=cd.freq;
								}
							}
							else
							{
								double rat=(double)cd.switchlist[0].lastfreq/(double)cd.freq;
								if (rat>(1.0+(double)cd.ratechangeamt/100)||rat<(1.0-(double)cd.ratechangeamt/100))
								{
									Action(cd.ratechange);
									cd.switchlist[ID].lastfreq=cd.freq;
								}
							}
						}
					}
					else // switch has changed to 'Off'
					{
						// don't react if the bounce delay hasn't expired
						if (t-cd.switchlist[ID].LastOff<cd.switchlist[ID].bouncedelay) continue;

						if (ID==0) overduenotice=false;

						snprintf(logme,939,"Switch%d Off",ID);
						WriteLog(logme,2);
						cd.switchlist[ID].state=state;
						cd.switchlist[ID].LastOff=t;

						SetEnvironment(cd.switchlist[0],cd);

						Action(cd.switchlist[ID].OffAction);

					}
				}
			}
		}

		// release the processor for a second before scanning again
		sleep(1);
	}

	// free allocated space
	for (ID=0;ID<100;ID++)
	{
		if (cd.switchlist[ID].OnAction!=NULL) free(cd.switchlist[ID].OnAction);
		if (cd.switchlist[ID].OffAction!=NULL) free(cd.switchlist[ID].OffAction);
	}

	if (cd.ratechange!=NULL) free(cd.ratechange);
	if (cd.overdue!=NULL) free(cd.overdue);

	return 0;
}


void trim(char *s)
{
	// trim leading and trailing white space from a string
	int textstart=-1;
	int i,len;

	len=strlen(s);

	// find where the printable text starts
	for (i=0;i<len;i++)
	{
		if (s[i]=='\t'||s[i]=='\n'||s[i]=='\r'||s[i]==' '||s[i]=='\t')
			continue;
		else
		{
			textstart=i;
			break;
		}
	}

	// no printable text
	if (textstart==-1)
	{
		s[0]=0;
		return;
	}

	// leading whitespace present
	if (textstart>0)
	{
		for (i=0;i<len-textstart;i++)
			s[i]=s[i+textstart];
		s[len-textstart]=0;
	}

	// work back from end to find last printable character
	for (i=len-1;i>0;i--)
	{
		if (s[i]=='\t'||s[i]=='\n'||s[i]=='\r'||s[i]==' '||s[i]=='\t')
			continue;
		else
		{
			s[i+1]=0;
			break;
		}
	}
}

// Custom string comparison function.
// Compares only to the end of the shortest string ("Hello" == "Hello World!")
// Case insensitive
int sa_strcmp(char *s1, const char *s2)
{
	unsigned int i;
	int diff;
	unsigned int len=strlen(s1);
	if (len>strlen(s2)) len=strlen(s2);

	for (i=0;i<len;i++)
	{
		diff=s1[i]-s2[i];
		diff=s1[i]-s2[i];

		if (diff==0||(diff==32&&s1[i]>='a'&&s1[i]<='z')||(diff==-32&&s1[i]>='A'&&s1[i]<='Z'))
			continue;
		return s1[i]-s2[i];
	}
	return 0;
}

// Set environment variables in advance of running an action script
void SetEnvironment(struct FloatSwitch s,struct ConfigData cd)
{
	char envstr[1000];
	int timeleft;

	snprintf(envstr,999,"%d",cd.freq);
	setenv("SAFREQ",envstr,1);

	snprintf(envstr,999,"%dm %ds",cd.freq/60,cd.freq%60);
	setenv("SAFREQF",envstr,1);

	cd.vol=((s.level/10.0)*(3.14159265*(cd.sumpdiameter/20.0)*(cd.sumpdiameter/20.0)))/1000.0;
	snprintf(envstr,999,"%d",cd.vol);
	setenv("SAVOLUME", envstr,1);
	if (cd.freq!=0)
		cd.rate=(((cd.highwater-cd.lowwater)/10.0*(3.14159265*(cd.sumpdiameter/20.0)*(cd.sumpdiameter/20.0)))/1000.0)*3600/cd.freq;
	else cd.rate=0;
	snprintf(envstr,999,"%d",cd.rate);
	setenv("SARATE",envstr,1);
	if (cd.rate==0) timeleft=0;
	else timeleft=(cd.capacity-cd.vol)*3600/cd.rate;
	snprintf(envstr,999,"%d",timeleft);
	setenv("SATIMELEFT",envstr,1);
	snprintf(envstr,999,"%d",timeleft/60);
	setenv("SATIMELEFTM",envstr,1);
}

// determine the frequency of activations for the selected switch
int GetFrequency(struct FloatSwitch s)
{
	int f=0;
	int z=0;

	for (int i=0;i<FREQ_HISTORY;i++)
	{
		f+=s.freq[i];
		if (s.freq[i]==0) z++;
	}

	if (z!=FREQ_HISTORY) f=f/(FREQ_HISTORY-z);

	return f;
}

// Execute an action script in a forked process to avoid slow scripts interfering with intended application behavior
void Action(char *action)
{
	if (action==NULL) return;
	#ifdef DEBUG
	snprintf(logme,939,"Executing Action \"%s\"",action);
	WriteLog(logme,3);
	#endif

	// fork and forget
	pid_t pid=fork();

	if (pid==0)
	{
		system(action);
		exit(0);
	}
}

void RefreshConfig(struct ConfigData &cd, bool initial)
{
	static char hash[65]="";

	int ID;
	char cline[65535];

	if (!initial)
	{
		// decide whether the config has changed
		system("sha256sum /etc/sumpalarm.conf > ./configsum");
		FILE *check=fopen("./configsum","r");
		if (check==NULL) return;
		fgets(cline,64,check);
		cline[64]=0;
		fclose(check);

		if (strcmp(hash,cline)==0) return; // no change to config

		WriteLog("Config changed",2);
		snprintf(logme,939,"Old: %s",hash);
		WriteLog(logme,2);
		snprintf(logme,939,"New: %s",hash,cline);
		WriteLog(logme,2);
		memcpy((void *)hash,(void *)cline,64);
		hash[64]=0;
	}
	else
	{
		// remember the checksum of the config file upon first load
		system("sha256sum /etc/sumpalarm.conf > ./configsum");
		FILE *check=fopen("./configsum","r");
		if (check==NULL) return;
		fgets(cline,64,check);
		cline[64]=0;
		fclose(check);
		memcpy((void *)hash,(void *)cline,64);
		hash[64]=0;
		WriteLog("Reading Config...",3);
	}

	FILE *conf=fopen(CONFIGFILE,"r");
	if (conf==NULL)
	{
		// if the file is locked or missing, it is a problem on startup but not during execution
		if (initial)
		{
			WriteLog("Unable to open config file /etc/sumpalarm.conf",1);
			exit(1);
		}
		else return;
	}

	// read each line of configuration
	while (fgets(cline,65534,conf)!=NULL)
	{
		trim(cline);	// remove leading or trailing whitespace from config line

		// if the line is blank or if the line is a comment, move to the next line
		if (strlen(cline)<1) continue;
		if (cline[0]=='#') continue;

		if (sa_strcmp(cline,"LogLevel")==0)
		{
			// remove whitespace around '=' (SumpDepth =  x --> SumpDepth=x)
			trim(cline+8);
			trim(cline+9);

			LogLevel=atoi(cline+9);
			if (LogLevel>3||LogLevel<0) LogLevel=3;
			snprintf(logme,939,"Logging level %d set",LogLevel);
			WriteLog(logme,3);
			continue;
		}
		if (sa_strcmp(cline,"LogFile")==0)
		{
			// remove whitespace around '=' (SumpDepth =  x --> SumpDepth=x)
			trim(cline+7);
			trim(cline+8);

			strncpy(LogFileName,cline+8,999);
			snprintf(logme,939,"LogFile set: %s",LogFileName);
			WriteLog(logme,3);
			continue;
		}

		// Sump pit sizing parameters
		if (sa_strcmp(cline,"SumpDepth")==0)
		{
			if (!initial) continue; // only load action scripts if not initial load
			// remove whitespace around '=' (SumpDepth =  x --> SumpDepth=x)
			trim(cline+9);
			trim(cline+10);

			cd.sumpdepth=atoi(cline+10);
			snprintf(logme,939,"SumpDepth set to %d",cd.sumpdepth);
			WriteLog(logme,3);
			continue;
		}

		if (sa_strcmp(cline,"SumpDiameter")==0)
		{
			if (!initial) continue; // only load action scripts if not initial load

			// remove whitespace around '='
			trim(cline+12);
			trim(cline+13);

			cd.sumpdiameter=atoi(cline+13);

			snprintf(logme,939,"SumpDiameter set to %d",cd.sumpdiameter);
			WriteLog(logme,3);

			continue;
		}
		if (sa_strcmp(cline,"LowWater")==0)
		{
			if (!initial) continue; // only load action scripts if not initial load

			// remove whitespace around '='
			trim(cline+8);
			trim(cline+9);

			cd.lowwater=atoi(cline+9);
			snprintf(logme,939,"LowWater set to %d",cd.lowwater);
			WriteLog(logme,3);

			continue;
		}

		if (sa_strcmp(cline,"HighWater")==0)
		{
			if (!initial) continue; // only load action scripts if not initial load

			// remove whitespace around '='
			trim(cline+9);
			trim(cline+10);

			cd.highwater=atoi(cline+10);
			snprintf(logme,939,"HighWater set to %d",cd.lowwater);
			WriteLog(logme,3);

			continue;
		}

		if (sa_strcmp(cline,"RateChangeAmt")==0)
		{
			// remove whitespace around '='
			trim(cline+13);
			trim(cline+14);

			cd.ratechangeamt=atoi(cline+14);
			snprintf(logme,939,"Rate Change percentage set to %d",cd.ratechangeamt);
			WriteLog(logme,3);

			continue;
		}

		if (sa_strcmp(cline,"OverdueThreshold")==0)
		{
			// remove whitespace around '='
			trim(cline+16);
			trim(cline+17);

			cd.overduethreshold=atoi(cline+17);
			snprintf(logme,939,"Rate Change percentage set to %d",cd.overduethreshold);
			WriteLog(logme,3);

			continue;
		}

		if (sa_strcmp(cline,"RateChange")==0)
		{
			// remove whitespace around '='
			trim(cline+10);
			trim(cline+11);

			if (strlen(cline+11)<=0) continue;

			if (cd.ratechange!=NULL)
			{
				if (strcmp(cd.ratechange,cline+11)==0) continue;
				else
				{
					free(cd.ratechange);
					cd.ratechange=(char *)malloc(strlen(cline+11)+1);
					strcpy(cd.ratechange,cline+11);
				}
			}
			else
			{
				cd.ratechange=(char *)malloc(strlen(cline+11)+1);
				strcpy(cd.ratechange,cline+11);
			}
			snprintf(logme,939,"Rate Change command string set: %s",cd.ratechange);
			WriteLog(logme,3);

			continue;
		}

		if (sa_strcmp(cline,"Overdue")==0&&sa_strcmp(cline,"OverdueT")!=0)
		{
			// remove whitespace around '='
			trim(cline+7);
			trim(cline+8);

			if (strlen(cline+8)<=0) continue;

			if (cd.overdue!=NULL)
			{
				if (strcmp(cd.overdue,cline+8)==0) continue;
				else
				{
					free(cd.overdue);
					cd.overdue=(char *)malloc(strlen(cline+8)+1);
					strcpy(cd.overdue,cline+8);
				}
			}
			else
			{
				cd.overdue=(char *)malloc(strlen(cline+8)+1);
				strcpy(cd.overdue,cline+8);
			}
			snprintf(logme,939,"Overdue command string set: %s",cd.overdue);
			WriteLog(logme,3);

			continue;
		}

		if (sa_strcmp(cline,"Switch")==0)
		{
			// Determine ID of switch
			if (cline[6]<'0'||cline[6]>'9')
				continue; // Invalid (non numeric) switch ID
			ID=cline[6]-'0';
			if (cline[7]>='0'&&cline[7]<='9')
				ID=ID*10+(cline[7]-'0'); // 2-digit switch ID

			int digits=1; // note the number of digits in the Switch ID for parsing
			if (ID>9)digits=2;

			// determine which parameter of the switch is being set
			if (sa_strcmp(cline+6+digits,"Level")==0)
			{
				trim(cline+11+digits);
				trim(cline+12+digits);
				cd.switchlist[ID].level=atoi(cline+12+digits);
				snprintf(logme,939,"Switch %d Level set: %d",ID,cd.switchlist[ID].level);
				WriteLog(logme,3);
				continue;
			}

			if (sa_strcmp(cline+6+digits,"Pin")==0)
			{
				if (!initial) continue;
				trim(cline+9+digits);
				trim(cline+10+digits);
				cd.switchlist[ID].pin=atoi(cline+10+digits);

				// validate value
				if (cd.switchlist[ID].pin!=0)
				{
					// a switch is only considered initialized when a Pin number has been set
					cd.switchlist[ID].initialized=1;
					snprintf(logme,939,"Switch %d Pin set: %d",ID,cd.switchlist[ID].pin);
					WriteLog(logme,3);
					continue;
				}
				else
				{
					snprintf(logme,939,"Switch%d GPIO PIN invalid");
					WriteLog(logme,1);
					exit(1);
				}
			}

			if (sa_strcmp(cline+6+digits,"Bounce")==0)
			{
				trim(cline+12+digits);
				trim(cline+13+digits);
				cd.switchlist[ID].bouncedelay=atoi(cline+13+digits);
				snprintf(logme,939,"Switch %d Bounce Delay set: %d",ID,cd.switchlist[ID].bouncedelay);
				continue;
			}

			if (sa_strcmp(cline+6+digits,"On")==0)
			{
				trim(cline+8+digits);
				trim(cline+9+digits);

				if (strlen(cline+9)<=0) continue;

				if (cd.switchlist[ID].OnAction!=NULL)
				{
					if (strcmp(cd.switchlist[ID].OnAction,cline+9)==0) continue;
					else
					{
						free(cd.switchlist[ID].OnAction);
						cd.switchlist[ID].OnAction=(char *)malloc(strlen(cline+9+digits)+1);
						strcpy(cd.switchlist[ID].OnAction,cline+9+digits);
					}
				}
				else
				{
					cd.switchlist[ID].OnAction=(char *)malloc(strlen(cline+9+digits)+1);
					strcpy(cd.switchlist[ID].OnAction,cline+9+digits);
				}
				snprintf(logme,939,"Switch %d On Action set: %s",ID,cd.switchlist[ID].OnAction);
				WriteLog(logme,3);

				continue;
			}

			if (sa_strcmp(cline+6+digits,"Off")==0)
			{
				trim(cline+9+digits);
				trim(cline+10+digits);

				if (strlen(cline+10)<=0) continue;

				if (cd.switchlist[ID].OffAction!=NULL)
				{
					if (strcmp(cd.switchlist[ID].OffAction,cline+10)==0) continue;
					else
					{
						free(cd.switchlist[ID].OffAction);
						cd.switchlist[ID].OffAction=(char *)malloc(strlen(cline+10+digits)+1);
						strcpy(cd.switchlist[ID].OffAction,cline+10+digits);
					}
				}
				else
				{
					cd.switchlist[ID].OffAction=(char *)malloc(strlen(cline+10+digits)+1);
					strcpy(cd.switchlist[ID].OffAction,cline+10+digits);
				}
				snprintf(logme,939,"Switch %d Off Action set: %s",ID,cd.switchlist[ID].OffAction);
				WriteLog(logme,3);
				continue;
			}
		}
	}

	fclose(conf);

	cd.capacity=(3.14159265*(cd.sumpdiameter/20.0)*(cd.sumpdiameter/20.0)*(cd.sumpdepth/10.0))/1000.0;
	snprintf(logme,939,"Capacity set to %d Litres",cd.capacity);
	WriteLog(logme,3);
}

// Write a log entry to file or to the console if running verbose
void WriteLog(const char *entry,int level)
{
	time_t lt;
	time(&lt);
	char logtime[40],logentry[1000];
	strftime(logtime, 39, "%Y-%m-%d %T", localtime(&lt));
	snprintf(logentry,999,"%s,\"%s\"\n",logtime,entry);

	if (!verbose&&level<=LogLevel)
	{
		FILE *logfile=fopen(LogFileName,"a");
		if (logfile==NULL) return;
		fputs(logentry,logfile);
		fclose(logfile);
	}
	else fputs(logentry,stdout);
}
