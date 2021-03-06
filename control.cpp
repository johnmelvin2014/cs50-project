// Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Universität Bremen nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cflie/CCrazyflie.h>
#include <stdio.h>
#include <unistd.h>
#include "../leap/leap_c.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <pthread.h>


using namespace std;

//CS50_TODO:  define your own states
//These are all the states the copter will have 
//#define FLY_STATE 1
//#define *other states*
//#define the states you will use here
#define PRE_NORMAL_STATE 1

#define NORMAL_STATE 2

#define PRE_HOVER_STATE 3

#define HOVER_STATE 4

#define LAND_STATE 5

//signals

//CS50_TODO:  define your own signals
//Define All the signals
#define NO_SIG 11//Each time when our state machine process the current signal
//A good practice is that when the current signal is processd, set the current signal variable back to "no signal"
#define CHANGE_HOVER_SIG 12//Use to transit state between Normal and Hover
#define TIME_OUT_SIG 13
#define RETURN_NORMAL_SIG 14
//The "time out signal" should be created every several seconds
//#define *other signals*
//#define the signals you will use here

//Define the trim value
//Some copters are not very balanced
//If it constantly drift to one direction
//You may want to define a "trim"
#define TRIM_ROLL 0
#define TRIM_PITCH 0
#define THRUST_CONSTANT 38500
#define HOVER_THRUST_LEVEL 32767
#define LAND_THRUST_CONST 50
#define ROLL_CONSTANT 20.0
#define PITCH_CONSTANT 20.0

//CS50_TODO:  define other variables here
//Such as: current states, current thrust
//variable for state and signals
int current_signal = 1;

int current_state = NORMAL_STATE;

float current_thrust = 30000;

float current_pitch;

float current_roll;

float factor;

float handPos;


//The pointer to the crazy flie data structure
CCrazyflie *cflieCopter=NULL;

//CS50_TODO:  define other helper function here
//In normal state, the flyNormal function will be called, set different parameter
//In hover state, different function should be called, because at that time, we should set the thrust as a const value(32767), see reference 
//(http://forum.bitcraze.se/viewtopic.php?f=6&t=523&start=20)

//The helper functions
void changeCurrentThrust(CCrazyflie *cflieCopter) {
  factor=52-4*batteryLevel(cflieCopter);
  current_thrust=35700+handPos*factor;
  //current_thrust = THRUST_CONSTANT+handPos*batteryLevel(cflieCopter);
}

void flyNormal(CCrazyflie *cflieCopter){
	changeCurrentThrust(cflieCopter);
	setThrust(cflieCopter,current_thrust);
	setRoll(cflieCopter, current_roll);
	setPitch(cflieCopter, current_pitch);
}

void flyHover(CCrazyflie *cflieCopter) {
  current_thrust = HOVER_THRUST_LEVEL;
  setThrust(cflieCopter, current_thrust);
  setRoll(cflieCopter, current_roll);
  setPitch(cflieCopter, current_pitch);
}

void flyLand(CCrazyflie *cflieCopter) {
  current_thrust -= (current_thrust>10000)?30:0;
  current_roll = 0.0;
  current_pitch = 0.0;
  setThrust(cflieCopter, current_thrust);
  setRoll(cflieCopter, current_roll);
  setPitch(cflieCopter, current_pitch);
}

void setHandPos(float hp){
	handPos = hp;
}

//passed int will be -1/0/1 to set roll direction
void changeRoll(int num) {
  current_roll = ROLL_CONSTANT*num;
}

//passed int will be -1/0/1 to set pitch direction
void changePitch(int num) {
 current_pitch = PITCH_CONSTANT*num;
}

//The leap motion call back functions
//Leap motion functions
void on_init(leap_controller_ref controller, void *user_info) {
	printf("init\n");
}

void on_connect(leap_controller_ref controller, void *user_info) {
	printf("connect\n");
}

void on_disconnect(leap_controller_ref controller, void *user_info) {
	printf("disconnect\n");
}

void on_exit(leap_controller_ref controller, void *user_info) {
	printf("exit\n");
}

//This function will be called when leapmotion detect hand gesture, 
void on_frame(leap_controller_ref controller, void *user_info) 
{
	leap_frame_ref frame = leap_controller_copy_frame(controller, 0);
	if ( current_signal == 1 )
	{
		if ( leap_frame_hands_count(frame) == 0 )
		{
	//		printf("\nNo hands detected. Entering land mode.\n");
			current_state = LAND_STATE;
		}
		else
		{
			for (int i = 0; i < leap_frame_hands_count(frame); i++) 
			{
				leap_hand_ref hand = leap_frame_hand_at_index(frame, i);
				leap_vector velo;
				leap_vector pos;
				leap_vector dir;
				leap_hand_direction(hand, &dir);
				leap_hand_palm_position(hand, &pos);
				leap_hand_palm_velocity(hand, &velo);
				if ( (velo.x >= 1500) && (current_state == NORMAL_STATE) )
				{
					current_state = HOVER_STATE;
					printf("HOVER\n");
					//current_state = PRE_HOVER_STATE;
				}
				else if ( (velo.x >= 1500) && (current_state == HOVER_STATE) )
				{
					current_state = NORMAL_STATE;
					//current_state =  PRE_NORMAL_STATE;
				}
				//else if ( current_state == NORMAL_STATE || current_state == HOVER_STATE)
				
				setHandPos(pos.y);

					if ( dir.y < -.5 )
					{
				//		printf("\nMove forward (negative pitch)\n");
				//		printf("dir.y has value of: %f\n", dir.y);
						changePitch(-1);
					}
					else if ( dir.y > .5 )
					{
				//		printf("\nMove backward (positive pitch)\n");
			//			printf("dir.y has value of: %f\n", dir.y);
						changePitch(-1);
					}
					else 
					{
			//			printf("\nNo pitch\n");
						changePitch(0);
					}

					if ( dir.x < -0.5 )
					{
			//			printf("\nMove left (negative roll)\n");
						changeRoll(-1);
					}
					else if ( dir.x > 0.5 )
					{
			//			printf("\nMove right (positive roll)\n");
						changeRoll(1);
					}
					else
					{
			//			printf("\nZero roll\n");
						changeRoll(0);
					}
				
			}
		}
		leap_frame_release(frame);
		current_signal = 0;		// set current signal to not processed, prevent processing multiple sigs before copter can handle them
	}
	else
	{
		leap_frame_release(frame);
		return;					// we haven't processed the last signal yet, do nothing
	}
}
	
//This the leap motion control callback function
//You don't have to modifiy this
void* leap_thread(void * param){
  struct leap_controller_callbacks callbacks;
  callbacks.on_init = on_init;
  callbacks.on_connect = on_connect;
  callbacks.on_disconnect = on_disconnect;
  callbacks.on_exit = on_exit;
  callbacks.on_frame = on_frame;
  leap_listener_ref listener = leap_listener_new(&callbacks, NULL);
  leap_controller_ref controller = leap_controller_new();
  leap_controller_add_listener(controller, listener);
  while(1);
}

//This thread will check the current state and send corrsponding command to the copter
void* main_control(void * param){
  CCrazyflie *cflieCopter=(CCrazyflie *)param;
  int i = 0;
  
  
 while(cycle(cflieCopter)) { 
	//transition depend on the current state
      //CS50_TODO : depend on the current signal and current state, you can call the helper function here to control the copter
	//current_state = NORMAL_STATE;
	//MJA


	
	switch(current_state) {
		
	  case NORMAL_STATE: {
		flyNormal(cflieCopter);
	  } break;

	  case HOVER_STATE: {
		flyHover(cflieCopter);
	  } break;

	  /*case PRE_NORMAL_STATE: {
		if(hoverPoint(cflieCopter) != 0)
			setHoverPoint(cflieCopter, 0);
	  //  if(/*TIME OUT SIGNAL)
	  //    current_signal = NORMAL_STATE;
	  } break;

	  case PRE_HOVER_STATE: {
		if(hoverPoint(cflieCopter) != 1)
			setHoverPoint(cflieCopter, 1);
	  //  if(/*TIME OUT SIGNAL)
	  //    current_signal = HOVER_STATE;
	  } break;*/

	  case LAND_STATE: {
		flyLand(cflieCopter);
	  } break; 
	}
	current_signal = 1;
  	
  	printf("Id=%d, Stabilizer: Roll=%f, Pitch=%f, Thrust=%f\r",
		31, rollValue(cflieCopter), pitchValue(cflieCopter), getThrust(cflieCopter));
  



	
  }
  printf("%s\n", "exit");
  return 0;
}

//This this the main function, use to set up the radio and init the copter
int main(int argc, char **argv) {
  CCrazyRadio *crRadio = new CCrazyRadio;
  //CS50_TODO
  //The second number is channel ID
  //The default channel ID is 10
  //Each group will have a unique ID in the demo day 
  CCrazyRadioConstructor(crRadio,"radio://0/36/250K");
  

  if(startRadio(crRadio)) 
  {
	cflieCopter=new CCrazyflie;
	CCrazyflieConstructor(crRadio,cflieCopter);

	//Initialize the set value
	setThrust(cflieCopter,10000);
	
	// Enable sending the setpoints. This can be used to temporarily
	// stop updating the internal controller setpoints and instead
	// sending dummy packets (to keep the connection alive).
	setSendSetpoints(cflieCopter,true);

	//CS50_TODO
	//Do initialization here
	//And set up the threads
	pthread_t copterControl, leapControl;

	pthread_create(&copterControl, NULL, main_control, cflieCopter);

 	pthread_create(&leapControl, NULL, leap_thread, NULL);

 	pthread_join(copterControl, NULL);
	pthread_join(leapControl, NULL);
	

   	delete cflieCopter; 
  } 
  else 
  {
	printf("%s\n", "Could not connect to dongle. Did you plug it in?");
  }
  return 0;
}
