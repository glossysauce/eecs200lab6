/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <signal.h>
#include <stdio.h>
#include <math.h> // for M_PI
#include "robot.h"
#include <stdlib.h>

#define show_raw_sensor 1
#define show_odometry   1
#define show_controller 1

#define record_odometry 0 // 1 to record, 0 not to record

// number of meters needed for not crashing while turning near obstacles:
#define CLEARANCE 0.20
// number of meters for deciding we are not obstructed any more
#define FAR 1.6



#define PI 3.14159265358979323846
#define DEG2RAD (PI/180.)

// sonar numbers:
#define RIGHT 0
#define FRONT 1
#define LEFT  2

#define LT_CLEARANCE 1
#define GT_FAR 0


// turning angles:
#define TURN_RIGHT (-90.*DEG2RAD)
#define TURN_LEFT  (+90.*DEG2RAD)


float dutyL, dutyR;

uint8_t uart_buf[BUF_SIZE];
int obstacle_known=0;

int using_gzweb=0;



/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
  setvbuf(stdout, NULL, _IONBF, 0);
    float x[100], y[100];
    int n;

    int go_forward=1;
    int go_path=0;
   int do_nothing=0;

    init_all();
    //sd_sensor.psi_dot0=0.0;
 

    //do_nothing=0;
    if(do_nothing) {
        sleep_sec(10.);
        rc_set_state(EXITING);
    }//endif

    // 1. go forward 1 meter:
    // set up path for robot to follow:
    // remember: +x is the direction the robot is pointing at t=0
    //           +y is pointing left
    obstacle_known = 0;
 go_forward=1;
    if(go_forward){
        n=1;
        x[0]=1.5;
        y[0]=0.0;
        execute_path(x,y,n);
        rc_set_state(EXITING);
    }// endif

    //go_path=1;
    if(go_path) {
        n=2;
        x[0]=1.5;
        y[0]=0.0;
        x[1]=1.5;
        y[1]=-1.5;
        execute_path(x,y,n);
        rc_set_state(EXITING);
    }//endif
    


    // turn off LEDs and close file descriptors
    finish();

}

//**********************************************************************
void sleep_sec(float sec)
{
    int i = sec*1000000; // in micro

    while (i > 4000000) {
	    rc_usleep(4000000);
	    i -= 4000000;
    }	
    rc_usleep(i);
}//endfunc: sleep_sec




//*************************************
int forward_nosonar(float xmeters)
{
    double accuracy=0.0;
	printf("forward_nosonar: %f\n",xmeters);
	//sleep_sec(0.25);
	//printf(".........................................................\n");
	printf("...\n");
  
	sd_setpoint.phi += xmeters / WHEEL_D * 2.0;

        if(using_gzweb) {
            accuracy = 0.2;
        } else {
            accuracy = 0.05;
        }//endif
	
	while(fabs(sd_odometry.phi - sd_setpoint.phi) > accuracy) {
            if(rc_get_state() == EXITING) finish();
	    //printf("phi=%f, desired-phi=%f\n",sd_odometry.phi,sd_setpoint.phi);
	    if(using_gzweb) rc_usleep(50000);  // Needed for virtual lab version (Gzweb)
	}
	//sd_setpoint.phi = sd_odometry.phi;  // NEW JAN (b/c virtual robot drifting)
	//sd_setpoint.psi = sd_odometry.psi;  //            "
        sleep_sec(1.0);
	//cal_gyro();
        return 1;
}//endfunc: forward_nosonar

//**********************************************************************
// go forward until the designated sonar is either close (or not) to an object
// sonar should be either: FRONT, LEFT, RIGHT
// close should be 1 if we should stop if too close
//                 0 if we should stop if too far
// too-close threshhold is CLEARANCE (0.15 m or so?)
// far threshhold is FAR (0.30 m or so?)
// returns 1 if finished without sonar, else 0.
//**********************************************************************
int forward(float xmeters,int sonar, int close)
{
    double accuracy=0.0;
    int sonar_stop =0;
    float extra_clearance=0.0;

    printf("forward: %f    sonar=%d  \n",xmeters,sonar);
    //sleep_sec(0.25);

    // add in an extra 0.10 m clearance for front sonar:
    if(sonar == FRONT) {
        extra_clearance = 0.15;
    } else {
        extra_clearance = 0.0;
        //extra_clearance = 0.05;
    }//endif
    
    // xmeters is distance in meters.
    sd_setpoint.phi += xmeters / WHEEL_D * 2.0; // move forward x meters.
    
    sonar_stop=0;
    //printf(".................................................\n");
    printf("...\n");

    // for virtual lab version (Gzweb): 0.2 instead of 0.1
    //while(fabs(sd_odometry.phi - sd_setpoint.phi) > 0.1) {
    if(using_gzweb){
        accuracy=0.2;
    } else {
        accuracy = 0.1;
    }//endif
    while(fabs(sd_odometry.phi - sd_setpoint.phi) > accuracy) {
      //printf(" ... forwardloop\n");
      //printf("phi setpt %f   phi-odom %f \n",sd_setpoint.phi,    sd_odometry.phi);

        if(sonar >= 0) {
	  //printf("forward: sonar = %f m\n",sd_sensor.sonar[sonar]);
            if(close) {
	      if(sd_sensor.sonar[sonar] < (CLEARANCE +extra_clearance)) {
		sonar_stop=1;
		if(sonar==FRONT){
		  stop_motors();
		  sd_setpoint.phi = sd_odometry.phi;
		  stop_motors();
		  //break;  // out of while loop, to skip sleep
		} else {
		  sd_setpoint.phi = sd_odometry.phi;
		  //break;  // out of while loop, to skip sleep
		}//endif
		//printf("forward: stopping because sonar < clearance: %f m\n",sd_sensor.sonar[sonar]);
		//printf("forward: stopping because %f m < %f: m\n",sd_sensor.sonar[sonar],(CLEARANCE +extra_clearance));
	      }//endif: sonar
	      
            } else {
                if(sd_sensor.sonar[sonar] > FAR) {
		  printf("forward: stopping because sonar > far: %f m\n",sd_sensor.sonar[sonar]);
		  
		  sonar_stop=1;
		  //stop_motors();
		  sd_setpoint.phi = sd_odometry.phi;
		  //stop_motors();
		  //break;  // out of while loop, to skip sleep
		}//endif
            }//endif

	    //let other threads run
	      rc_usleep(50000);    // Needed for virtual lab version (Gzweb)
	    
	    
        } else {
            // dont use sonar
        }//endif
        
	if(rc_get_state() == EXITING) finish();
        
    }//endwhile
    //sleep_sec(0.25); //rc_usleep(500000); // sleep 0.5 sec to settle down

    // OR LEAVE IN SLEEP, and set sd_setpoint.phi = sd_odometry.phi;
    //   b/c it's changed in the meantime!
    //sd_setpoint.phi = sd_odometry.phi;  // NEW JAN (b/c virtual robot drifting)
    //sd_setpoint.psi = sd_odometry.psi;  // NEW JAN (b/c virtual robot drifting)
    //rc_motor_set(MOTOR_CHANNEL_L, 0.0);
    //rc_motor_set(MOTOR_CHANNEL_R, 0.0);
    //sleep_sec(0.25);
    //cal_gyro();
    
    
    if(sonar_stop) return 0;
    return 1;
    
}//endfunc: forward

//******************************************
void cal_gyro() {
  //sd_sensor.psi_dot0 = mpu_data.gyro[2];
}

//****************************************
void stop_motors()
{
  rc_motor_set(MOTOR_CHANNEL_L, 0.0);
  rc_motor_set(MOTOR_CHANNEL_R, 0.0);

}//endfunc: stop_motors

//**********************************************************************
void turn(float theta_radians)
{
    printf("turn: %f rad, %f deg\n",theta_radians,theta_radians*180./3.14159);
    //sleep_sec(0.25);
    //printf(".........................................................\n");
    printf(".....\n");
   
    // theta_radians is delta-angle from current heading, radians.
    // positive rotates to the right.
    sd_setpoint.psi += theta_radians;
    while(fabs( warp_to_pi(sd_odometry.psi - sd_setpoint.psi) ) > 0.05){
    //while(fabs( warp_to_pi(sd_odometry.psi - sd_setpoint.psi) ) > 0.1){
        if(rc_get_state()==EXITING) finish();

	//printf("psi-setpt %f   psi-odom %f \n",sd_setpoint.psi,    sd_odometry.psi);
	
	//printf(" ... turnloop\n");
	if(using_gzweb) rc_usleep(50000);     // Needed for virtual lab version (Gzweb)
    }

    //sleep_sec(0.25); //rc_usleep(500000); // sleep 0.5 sec to settle down
    //sd_setpoint.phi = sd_odometry.phi;  // NEW JAN (b/c virtual robot drifting)
    //sd_setpoint.psi = sd_odometry.psi;  // NEW JAN (b/c virtual robot drifting)



}//endfunc: turn

//***************************************************************************
void heading(float theta_radians)
{
    printf("heading: %f rad, %f deg\n",theta_radians,theta_radians*180./3.14159);
    //sleep_sec(0.25);
    
 
    sd_setpoint.psi = theta_radians;
    while(fabs( warp_to_pi(sd_odometry.psi - sd_setpoint.psi) ) > 0.05){
    //while(fabs( warp_to_pi(sd_odometry.psi - sd_setpoint.psi) ) > 0.1){
        if(rc_get_state()==EXITING) finish();
	//printf(" ... turnloop\n");
	if(using_gzweb) rc_usleep(50000);     // Needed for virtual lab version (Gzweb)
    }
}//endfunc: heading


//***************************************************************************
void execute_path(float *x, float *y, int n){
    int i;

    float theta_radians, xmeters;
    float dx, dy;

    // loop involves these 2 steps, repeated:
    // 1. turn toward next point in waypoints list
    // 2. move to the next point
    // assumes x[0],y[0] is current location.
    for(i=0;i<n;i++){
        // figure out angle to turn:
        dx = x[i] - sd_odometry.x;
        dy = y[i] - sd_odometry.y;
        //printf("dx=%f, dy=%f\n",dx, dy);
        theta_radians = atan2(dy, dx);
        turn(theta_radians-sd_odometry.psi);
        
        // figure out distance to go:
        xmeters = sqrt(dx*dx + dy*dy);
        
        // go there as long as not obstructed in front:
	//
	//
	//printf("a0 \n");
        while(!forward(xmeters,FRONT,LT_CLEARANCE)) {
            // need to go around an obstacle:
	  //printf("a \n");
            go_around();
	    //printf("b \n");
            
            // continue to waypoint:
            dx = x[i] - sd_odometry.x;
            dy = y[i] - sd_odometry.y;
	    
	    theta_radians = atan2(dy, dx);
	    turn(theta_radians-sd_odometry.psi);
	
            xmeters = sqrt(dx*dx + dy*dy);
        }//endwhile
        //printf("c \n");
        printf("execute_path: x,y=%f, %f\n",sd_odometry.x,sd_odometry.y);
    }//endfor

}//endfunc

//************************************************************
// go around an obstacle that is rectangular, and aligned
// with the x,y coordinate axes
void go_around()
{
    if(obstacle_known) {
        go_around_known();
    } else {
        go_around_unknown();
    }//endif
}//endif

//************************************************************
// go around an obstacle that is rectangular, and aligned
// with the x,y coordinate axes, with known dimensions.
void go_around_known()
{
    // to go around:
    // we are currently stopped about 15 cm from an obstacle
    // that is in front of us.
    //
    // 1. turn left
    // 2. go forward half the obstacle width + 15cm
    // 3. turn right
    // 4. go forward half the obstacle depth + 30cm
    // 5. turn right
    // 6. go forward half the obstacle width + 15cm
    // 7. turn left

    // define these for your obstacle, in meters:
    //float width=0.18;   
    //float depth=0.12;  box1 dimensions: 0.24 0.36 0.22


    // your code goes here.
    
}//endfunc: go-around_known

//************************************************************
// go around an obstacle that is rectangular, and aligned
// with the x,y coordinate axes, but with unknown dimensions.
void go_around_unknown()
{
    // to go around:
    // we are currently stopped about 15 cm from an obstacle
    // that is in front of us.
    //
    // 1. turn left
turn(TURN_LEFT);
    // 2. note position: pt1=(x,y)
double pt1y = sd_setpoint.phi;
    // 3. go forward until right-sonar is unobstructed
forward(2, RIGHT, GT_FAR);
    // 4. go forward 15cm more
forward_nosonar(.15);
    // 5. not new position: pt2=(x,y)
double pt2y = sd_setpoint.phi;
    // 6. turn right
turn(TURN_RIGHT);
    // 7. go forward until right-sonar is obstructed
forward(0.1, RIGHT, LT_CLEARANCE);
    // 8. go forward until right-sonar is unobstructed
forward(1, RIGHT, GT_FAR);
    // 9. go forward 15cm more
forward_nosonar(.15);
    // 10. turn right
turn(TURN_RIGHT);
    // 11. d= dist between pt1 and pt2
double d = pt2y - pt1y;
    // 12. go forward d
forward_nosonar(d);
    // 13. turn left
turn(TURN_LEFT);
    // should now be pointing at next waypoint,
    //  but we are on the other side of the obstacle.

    // your code goes here.
    
}//endfunc: go_around



//**************************************************************
void robot_controller(){
    //lock state mutex
    pthread_mutex_lock(&state_mutex);

    //printf("1    \n");

    // check for exiting state
    if(rc_get_state()==EXITING){
        rc_motor_set(MOTOR_CHANNEL_L,0.0);
        rc_motor_set(MOTOR_CHANNEL_R,0.0);
        return;
    }

    /*** sensor and odometry update ***/
    sd_odometry_update(&sd_odometry, &sd_sensor, &mpu_data);

    if(record_odometry){
        printf("%lld,%f,%f,%f,%f\n", (long long)rc_nanos_since_boot(), sd_odometry.x, sd_odometry.y, sd_odometry.psi, sd_odometry.phi);
    }

    /*** controller update ***/
    controller_update(&D1, &D2, &sd_setpoint, &sd_odometry, &dutyL, &dutyR);

    if(0){
    printf("  - setpoint->psi = %f   odometry->psi = %f \n",  sd_setpoint.psi*180./3.14159, sd_odometry.psi*180./3.14159);
    printf("  - setpoint->phi = %f   odometry->phi = %f \n",  sd_setpoint.phi, sd_odometry.phi);
    printf("  ... and dutyL= %f   dutyR = %f \n",dutyL,dutyR);
    }
    
    /*** motor output ***/
    rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
    rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);

    //unlock state mutex
    pthread_mutex_unlock(&state_mutex);
}

/*******************************************************************************
* battery_checker() 
*
* update setpoints for controller
*******************************************************************************/
void* battery_checker(void* ptr){
    battery_v = V_NOMINAL;
    while(rc_get_state()!=EXITING){
        battery_v = rc_adc_batt();
        // if value doesn't make sense, use nominal voltage
        if(battery_v > 9.0) battery_v = V_NOMINAL;

        // exit when battery is low
        if(battery_v < V_NOMINAL){
            printf("\n\nLow Battery!!! Stop Now!!!\n\n");
            rc_set_state(EXITING);
        }

        sleep_sec(1./BATTERY_CHECK_HZ);
    }
    return ptr;
}

//*******************************************************************************
//*******************************************************************************
void* sonar_loop(void* ptr){
    while(rc_get_state()!=EXITING){
        read_sonars();
	// Needed for virtual lab version (Gzweb):
        if(using_gzweb) rc_usleep(100000);// 100 msec  (0.1 seconds)
    }
    return ptr;
}


/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void* printf_loop(void* ptr){
    if(!record_odometry){
        printf("\n");
        if(show_raw_sensor) printf("       Raw Sensor                                                      |");
        if(show_odometry)   printf("                   Odometry                    |");
        if(show_controller) printf("                  Controller                               |");
        printf("\n");

        if(show_raw_sensor){
            printf("  left Enc |");
            printf(" right Enc |");
            printf(" rate of ψ |");
            printf("  sonar R  |");
            printf("  sonar F  |");
            printf("  sonar L  |");
        }

        if(show_odometry){
            printf("      x    |");
            printf("      y    |");
            printf("      ψ    |");
            printf("      Φ    |");
        }

        if(show_controller){
            printf("  Battery  |");
            printf("   dutyL   |");
            printf("   dutyR   |");
            printf("   phi_r   |");
            printf("   psi_r   |");
        }

        printf(" \n");

        while(rc_get_state()!=EXITING){
            pthread_mutex_lock(&state_mutex);
            printf("\r");
            //printf("\n");

	    //read_sonars();

            if(show_raw_sensor){
                printf("%10d |", sd_sensor.left_encoder); 
                printf("%10d |", sd_sensor.right_encoder); 
                printf("  %6.3f   |", (double)sd_sensor.psi_dot); 
                printf("  %6.3f   |", (double)sd_sensor.sonar[0]); 
                printf("  %6.3f   |", (double)sd_sensor.sonar[1]); 
                printf("  %6.3f   |", (double)sd_sensor.sonar[2]); 
            }

            if(show_odometry){
                printf("  %6.3f   |", (double)sd_odometry.x);
                printf("  %6.3f   |", (double)sd_odometry.y);
                printf("  %6.3f   |", (double)sd_odometry.psi);
                printf("  %6.3f   |", (double)sd_odometry.phi);
            }

            if(show_controller){
                printf("%10.3f |", (double)battery_v);

                printf("  %6.3f   |", (double)dutyL);
                printf("  %6.3f   |", (double)dutyR);

                printf("  %6.3f   |", (double)sd_setpoint.phi);
                printf("  %6.3f   |", (double)sd_setpoint.psi);
            }

            pthread_mutex_unlock(&state_mutex);
            fflush(stdout);
            if(rc_get_state()==EXITING) finish();
            rc_usleep(1E6/PRINT_RATE_HZ);
        }
        finish();
    }
    return ptr;
}


//**************************************************
void read_sonars(){
    // flush anything revceived but not read
    rc_uart_flush(UART_BUS);

    // read back as line
    memset(uart_buf,0,sizeof(uart_buf));
    int ret = 0;
    while(ret <= 0){ // keep reading until read something
        ret = rc_uart_read_line(UART_BUS, uart_buf, sizeof(uart_buf));
    }

    // parse to int array
    char delim[] = ", ";
    char *ptr;

    ptr = strtok((char *)uart_buf, delim);
    for(int i = 0; i < SENSOR_NUM; ++i){
        if(ptr){
            sd_sensor.sonar[i] = atoi(ptr)*0.01; // meters
            if(atoi(ptr)==0){
                sd_sensor.sonar[i]=4.00; // 4 meters = infinity
            }//endif
            ptr = strtok(NULL, delim);
        }//endif
    }//endfor

    //printf("RIGHT sonar %f \n",sd_sensor.sonar[0]);

    //printf("SONARS: %f, %f, %f\n",sd_sensor.sonar[0],sd_sensor.sonar[1],sd_sensor.sonar[2]);

    sd_sensor.new_sonar_ready = 1;

}

//************************************************************************8
// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
    rc_set_state(EXITING);
    return;
}

//************************************************************************8
int init_all()
{
    if(!record_odometry) printf("initializing... ");

    // make sure another instance isn't running
    if(rc_kill_existing_process(2.0)<-2) return -1;

    // just in case: set signal handler so control-C works
    signal(SIGINT, __signal_handler);

    // start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        exit(-1);
    }
    signal(SIGHUP, __signal_handler);

    // initialize enocders
    if(rc_encoder_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize encoders\n");
        exit(-1);
    }

    // initialize motors
    if(rc_motor_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize motors\n");
        exit(-1);
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        exit(-1);
    }

    // initialize UART
    //printf("\ninitializing UART bus %d\n\n", UART_BUS);
    // disable canonical (0), 1 stop bit (1), disable parity (0)
    if(rc_uart_init(UART_BUS, BAUDRATE, TIMEOUT_S, 0,1,0)){
            printf("Failed to rc_uart_init%d\n", UART_BUS);
            exit(-1);
    }


    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    // set up D1 Phi controller and D2 gamma (steering) controller
    if(controller_init(&D1, &D2) == -1){
        fprintf(stderr,"ERROR in controller_init\n");
        exit(-1);
    }

    // set up IMU configuration
    //if(!record_odometry) printf("initializing imu... \n");

    // set up mpu configuration
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.i2c_bus = 2;
    rc_mpu_calibrate_gyro_routine(mpu_config);
    
    mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_fetch_accel_gyro = 1;

    // start mpu
    if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
        fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
        exit(-1);
    }

    pthread_mutex_init(&state_mutex, NULL);

    if(!record_odometry) { 
        //printf("attaching imu interupt...\n");
     } else {
        printf("timestamp(nano seconds since boot),x(m),y(m),ψ(rad),Φ(rad)\n");
    }//endif
    rc_mpu_set_dmp_callback(&robot_controller);

    // set odometry origin at (0,0) with 0 heading
    sd_odometry_init(&sd_odometry, 0, 0, 0);

    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother
    //if(!record_odometry) printf("starting battery checker thread... \n");

    if(!using_gzweb){
      pthread_t  battery_checker_thread;
      rc_pthread_create(&battery_checker_thread, battery_checker, (void*) NULL, SCHED_OTHER, 0);
    }//endif
    
    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother
    //if(!record_odometry) printf("starting setpoint thread... \n");
    //pthread_t  setpoint_thread;
    //rc_pthread_create(&setpoint_thread, setpoint_loop, (void*) NULL, SCHED_OTHER, 0);

    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother
    //if(!record_odometry) printf("starting print thread... \n");

    if(!using_gzweb) {
      pthread_t  printf_thread;
      rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);
    }//endif

    
    pthread_t  sonar_thread;
    pthread_attr_t tattr;
    int ret=0;
    int newprio = 20;
    struct sched_param param;
    
    ret = pthread_attr_init (&tattr); // initialized w/ default attrs
    ret = pthread_attr_getschedparam (&tattr, &param); // get existing param
    param.sched_priority = newprio;  // set this new thread prio
    ret = pthread_attr_setschedparam (&tattr, &param); // set new sched param
    ret = ret+1;
    
    //rc_pthread_create(&sonar_thread, sonar_loop, (void*) NULL, SCHED_OTHER, 0);
    pthread_create(&sonar_thread, &tattr, sonar_loop, NULL);

    // enable motor to run
    rc_motor_free_spin(0);

    // Keep looping until state changes to EXITING
    if(!record_odometry) printf("starting... \n");
    
    rc_set_state(RUNNING);
    sleep_sec(1.);
    return 0;
}//endfunc: init_all   


//*************************************************************************************
void finish()
{
    controller_cleanup(&D1, &D2);
    rc_led_cleanup();
    rc_encoder_cleanup();
    rc_mpu_power_off();
    rc_remove_pid_file();   // remove pid file LAST
    exit(0);
}//endfunc: finish
