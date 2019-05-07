/**
* rc_test_paraglider
*
* RC Paraglider Test 1
**/

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI
#include <robotcontrol.h>
#include <ncurses.h>
#include <rc/servo.h>
#include <time.h>
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/uart.h>
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>

#include "rc_balance_defs.h"

#define PORT     44444 
#define MAXLINE 1024

//Paraglider Constants
#define ANGLE_OFFSET 0.1
#define kPYaw 0.01
#define kIYaw 0.000000000000000000000002
//#define kDRoll 00 //2300
#define PARALLEL -0.05

double kDYaw = 0;

/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.
 */
typedef struct setpoint_t{
	arm_state_t arm_state;	///< see arm_state_t declaration
	double yaw;
	double throttle;		
}setpoint_t;

/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t{
	double roll;	
	double pitch;		
	double yaw;
	double vBatt;		///< battery voltage
	double servo_pos;		///< output of steering controller D
	double D;	///< u compensated for battery voltage
} core_state_t;

// possible modes, user selected with command line arguments
typedef enum m_input_mode_t{
	NONE,
	DSM,
	STDIN,
	TUNE,
	KEYBOARD,
	TIME,
	JOYSTICK
} m_input_mode_t;


static void __print_usage(void);
static void __balance_controller(void);		///< mpu interrupt routine
static void* __setpoint_manager(void* ptr);	///< background thread
static void* __battery_checker(void* ptr);	///< background thread
static void* __printf_loop(void* ptr);		///< background thread
static void* __network_manager(void* ptr);
static void* __time_manager(void* ptr);
static void* __serial_manager(void* ptr);
static int __zero_out_controller(void);
static int __disarm_controller(void);
static int __arm_controller(void);
static int __wait_for_starting_condition(void);
static void __on_pause_press(void);
static void __on_mode_release(void);


// global variables
core_state_t cstate;
setpoint_t setpoint;
rc_filter_t D = RC_FILTER_INITIALIZER;
rc_mpu_data_t mpu_data;
m_input_mode_t m_input_mode = NONE;
struct timespec spec1;
struct timespec spec2;
double steering_stick = 0.0;
double throttle_stick = 0.0;
int wait_time = 100000;
int n = 1;
int waiting_on_udp = 0;

static int running;
int first_crtlc = 1;
int first_balance_run = 1;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

/*
 * printed if some invalid argument was given
 */
static void __print_usage(void)
{
	printf("\n");
	printf("-i {dsm|stdin|none}     specify input\n");
	printf("-h                      print this help message\n");
	printf("\n");
}

/**
 * Initialize the filters, mpu, threads, & wait until shut down
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char *argv[])
{
	int c;
	pthread_t setpoint_thread = 0;
	pthread_t battery_thread = 0;
	pthread_t printf_thread = 0;
	pthread_t network_thread = 0;
	pthread_t time_thread = 0;
	pthread_t serial_thread = 0;
	int i;
	int frequency_hz = 50;	// default 50hz frequency to send pulses
	int wakeup_en = 1;	// wakeup period enabled by default
	double wakeup_s = 3.0;	// wakeup period in seconds
	double wakeup_val = -0.1;// wakeup value

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "i:")) != -1){
		switch (c){
		case 'i': // input option
			if(!strcmp("dsm", optarg)) {
				m_input_mode = DSM;
			} else if(!strcmp("stdin", optarg)) {
				m_input_mode = STDIN;
			} else if(!strcmp("tune", optarg)) {
				m_input_mode = TUNE;
			} else if(!strcmp("none", optarg)){
				m_input_mode = NONE;
			} else if(!strcmp("keyboard", optarg)){
				m_input_mode = KEYBOARD;
			} else if(!strcmp("time", optarg)){
				m_input_mode = TIME;
			} else if(!strcmp("joystick", optarg)){
				m_input_mode = JOYSTICK;
			} else {
				__print_usage();
				return -1;
			}
			break;
		case 'h':
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize mode button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,__on_pause_press,NULL);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,NULL,__on_mode_release);

	// initialize adc
	if(rc_adc_init()==-1){
		fprintf(stderr, "failed to initialize adc\n");
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	printf("\nPress and release MODE button to toggle DSM drive mode\n");
	printf("Press and release PAUSE button to pause/start the motors\n");
	printf("hold pause button down for 2 seconds to exit\n");

	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 1)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
		return -1;
	}

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_X_DOWN;
	setpoint.arm_state = DISARMED;

	// if gyro isn't calibrated, run the calibration routine
	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}

	// make sure setpoint starts at normal values


	// // set up D1 Theta controller
	// double D1_num[] = D1_NUM;
	// double D1_den[] = D1_DEN;
	// if(rc_filter_alloc_from_arrays(&D1, DT, D1_num, D1_NUM_LEN, D1_den, D1_DEN_LEN)){
	// 	fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
	// 	return -1;
	// }
	// D1.gain = D1_GAIN;
	// rc_filter_enable_saturation(&D1, -1.0, 1.0);
	// rc_filter_enable_soft_start(&D1, SOFT_START_SEC);

	// // set up D2 Phi controller
	// double D2_num[] = D2_NUM;
	// double D2_den[] = D2_DEN;
	// if(rc_filter_alloc_from_arrays(&D2, DT, D2_num, D2_NUM_LEN, D2_den, D2_DEN_LEN)){
	// 	fprintf(stderr,"ERROR in rc_balance, failed to make filter D2\n");
	// 	return -1;
	// }
	// D2.gain = D2_GAIN;
	// rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
	// rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

	// printf("Inner Loop controller D1:\n");
	// rc_filter_print(D1);
	// printf("\nOuter Loop controller D2:\n");
	// rc_filter_print(D2);

	// // set up D3 gamma (steering) controller
	// if(rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
	// 	fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
	// 	return -1;
	// }
	// rc_filter_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

	// start a thread to slowly sample battery
	if(rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}
	// wait for the battery thread to make the first read
	while(cstate.vBatt<1.0 && rc_get_state()!=EXITING) rc_usleep(10000);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}

	// start mpu
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}

	// start balance stack to control setpoints
	if(rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}

	// start balance stack to control setpoints
	if(rc_pthread_create(&network_thread, __network_manager, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start network thread\n");
		return -1;
	}

	if(rc_pthread_create(&time_thread, __time_manager, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start network thread\n");
		return -1;
	}

	if(rc_pthread_create(&serial_thread, __serial_manager, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start network thread\n");
		return -1;
	}

	// start in the RUNNING state, pressing the pause button will swap to
	// the PAUSED state then back again.
	rc_set_state(RUNNING);

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	// initialize PRU
	if(rc_servo_init()) return -1;
	int x = 1;
	// turn on power
	printf("Turning On 6V Servo Power Rail\n");
	if ((x = rc_servo_power_rail_en(1))) {
		printf("Failed to turn on 6v rail\n");
		printf("Error Code: %d\n", x);
		return -1;
	}

	if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US)) return -1;

	if(wakeup_en){
		printf("waking ESC up from idle for 3 seconds\n");
		for(i=0;i<=frequency_hz*wakeup_s;i++){
			if(running==0) return 0;
			if(rc_servo_send_esc_pulse_normalized(2,wakeup_val)==-1) return -1;
			rc_usleep(1000000/frequency_hz);
		}
		printf("done with wakeup period\n");
	}

	cstate.servo_pos = PARALLEL;
	int j = 100;
	while (j > 0) {
		rc_servo_send_pulse_normalized(1, PARALLEL);
		rc_servo_send_esc_pulse_normalized(2, 0);
		usleep(100);
		j--;
	}

	setpoint.arm_state = ARMED;

/*
	initscr();
	noecho();
	cbreak();
	nodelay(stdscr, TRUE);
	*/
	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&__balance_controller);

	// chill until something exits the program
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		rc_usleep(200000);
	}

	// join threads
	rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
	rc_pthread_timed_join(battery_thread, NULL, 1.5);
	rc_pthread_timed_join(printf_thread, NULL, 1.5);
	//rc_pthread_timed_join(network_thread, NULL, 1.5);
	rc_pthread_timed_join(time_thread, NULL, 1.5);
	rc_pthread_timed_join(serial_thread, NULL, 1.5);

	// // cleanup
	// rc_filter_free(&D1);
	// rc_filter_free(&D2);
	// rc_filter_free(&D3);
	endwin();
	rc_mpu_power_off();
	rc_servo_cleanup();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}

void* __time_manager(__attribute__ ((unused)) void* ptr) {
	sleep(300);
	setpoint.arm_state = DISARMED;
	rc_set_state(EXITING);
	return NULL;
}

void* __serial_manager(__attribute__ ((unused)) void* ptr) {
	uint8_t buffer[10];
	char* test_str = "Hello World";
	int ret;

	if (rc_uart_init(1, 115200, 0.5, 0, 1, 0)) {
		printf("Failed to rc_uart_init1\n");
		rc_set_state(EXITING);
		return NULL;
	}

	
    while(rc_get_state()!=EXITING){
    	//rc_uart_flush(1);
        //rc_uart_write(1, (uint8_t*)test_str, 12);
    	
    	ret = rc_uart_read_bytes(1, buffer, 10);
        if(ret<0) fprintf(stderr,"Error reading bus\n");
        else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
        else printf("Received %d bytes: %s \n", ret, buffer);
        rc_uart_flush(1);
        usleep(10);
        
	}

	rc_uart_close(1);
    return NULL;
}

void* __network_manager(__attribute__ ((unused)) void* ptr) {
	int sockfd; 
    char buffer[MAXLINE]; 
    char *hello = "H"; 
    struct sockaddr_in servaddr, cliaddr; 
      
      // Creating socket file descriptor 
	    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
	        perror("socket creation failed"); 
	        exit(EXIT_FAILURE); 
	    } 
	      
	    memset(&servaddr, 0, sizeof(servaddr)); 
	    memset(&cliaddr, 0, sizeof(cliaddr)); 
	      
	    // Filling server information 
	    servaddr.sin_family    = AF_INET; // IPv4 
	    servaddr.sin_addr.s_addr = INADDR_ANY; 
	    servaddr.sin_port = htons(PORT); 
	      
	    // Bind the socket with the server address 
	    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
	            sizeof(servaddr)) < 0 ) 
	    { 
	        perror("bind failed");
	        exit(EXIT_FAILURE);
	    } 
	    int len;
	    int j = 0;
    while(rc_get_state()!=EXITING){
    	//steering_stick = 0.0;
    	//throttle_stick = 0.0;
    	n = 0;
    	clock_gettime(CLOCK_REALTIME, &spec1);

    	waiting_on_udp = spec1.tv_sec;
	    n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
	                MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
	                &len);
	    buffer[n] = '\0';
	    sscanf(buffer, "%lf|%lf", &steering_stick, &throttle_stick);

	    //printf("Client : %s\n", buffer); 
	    //sendto(sockfd, (const char *)hello, strlen(hello),  
	      //  MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
	        //    len); 
	    //printf("Hello\n");
	}

    return NULL;
}

/**
 * This thread is in charge of adjusting the controller setpoint based on user
 * inputs from dsm radio control. Also detects pickup to control arming the
 * controller.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void* __setpoint_manager(__attribute__ ((unused)) void* ptr) {
	double drive_stick, turn_stick; // input sticks
	int ch, stdin_timeout = 0; // for stdin input
	char in_str[11];
	int j = 0;

	while(rc_get_state()!=EXITING){

		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000);
		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		// if(setpoint.arm_state == DISARMED){
		// 	__zero_out_controller();
		// 	__arm_controller();
		// } else {
		// 	continue;
		// }
		
		//setpoint.yaw = 0;

		//sleep(10);
		//setpoint.roll = 1;
		//sleep(10);

		if(setpoint.arm_state == DISARMED) continue;

		// if dsm is active, update the setpoint rates
		switch(m_input_mode){
		case TUNE:

			if ((ch = getchar()) != EOF){
				if(ch == '['){
					kDYaw += 10;
				} else if(ch == ';'){
					kDYaw -= 10;
				}
			}

			break;
		case KEYBOARD:

			if ((ch = getch()) != ERR){
				if(ch == ' '){
					cstate.servo_pos = PARALLEL;
				} else if (ch == 'd') {
					if (cstate.servo_pos < 0.5) {
						cstate.servo_pos += 0.1;
					}
				} else if (ch == 'a') {
					if (cstate.servo_pos > -0.5) {
						cstate.servo_pos -= 0.1;
					}
				} else if (ch == '^c') {
					rc_set_state(EXITING);
				}
			}

			break;
		case TIME:

			sleep(7);
			setpoint.arm_state = DISARMED;
			rc_set_state(EXITING);
			break;
		case JOYSTICK:
			setpoint.yaw -= (steering_stick - 0.44)*0.003;
			setpoint.throttle = (throttle_stick - 0.44)*0.52 + 0.8;

			if (setpoint.throttle > 1) {
				setpoint.throttle = 1;
			} else if (setpoint.throttle < 0) {
				setpoint.throttle = 0;
			}

			clock_gettime(CLOCK_REALTIME, &spec2);

			if ((spec2.tv_sec - waiting_on_udp) > 1) {
	    		printf("Joystick Disconnected\n");
	    		rc_servo_send_esc_pulse_normalized(2, 0);
	    		rc_set_state(EXITING);
	    	}

			printf("%lf : %lf\n", steering_stick, throttle_stick);
			break;
		default:
			break;
		}
	}

	// if state becomes EXITING the above loop exists and we disarm here
	__disarm_controller();
	return NULL;
}

/**
 * discrete-time balance controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
static void __balance_controller(void)
{
	double error_yaw;
	double prev_error_yaw;
	double prev_time;
	double time_diff;
	double roll_adjusted;
	double p;
	double i;
	double d;


	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	// angle theta is positive in the direction of forward tip around X axis
	cstate.yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z];// - ANGLE_OFFSET;
	error_yaw = cstate.yaw - setpoint.yaw; //- ANGLE_OFFSET;
	error_yaw = ((double) ((int) (error_yaw*10000)) )/10000.0;

	if (first_balance_run) {
		prev_error_yaw = error_yaw;
		prev_time = 0;
		i = 0;
		first_balance_run = 0;
	}

	if (running && setpoint.arm_state == ARMED) {
		// if (roll_adjusted <= 1 && roll_adjusted >= -1) {
		// 	rc_servo_send_pulse_normalized(1, roll_adjusted);
		// } else if (roll_adjusted > 1){
		// 	rc_servo_send_pulse_normalized(1, 1);
		// } else {
		// 	rc_servo_send_pulse_normalized(1, -1);
		// }
		// if (roll_adjusted > 0.1) {
		// 	cstate.servo_pos += fabs(roll_adjusted)*kPRoll;
		// } else if (roll_adjusted < -0.1) {
		// 	cstate.servo_pos -= fabs(roll_adjusted)*kPRoll;
		// }-

		// p = -error_yaw;
		// d = ((error_yaw-prev_error_yaw)/((double) (rc_nanos_since_boot() - prev_time))*1000000000);

		// if (error_yaw < 1 && error_yaw > -1) {
		//  	i += (((rc_nanos_since_boot() - prev_time)*1000000000)*(-error_yaw - prev_error_yaw))/2.0;
		// }
		// printf("kD: %lf\n", kDYaw);

		cstate.servo_pos = error_yaw*0.5;// - PARALLEL - ANGLE_OFFSET;//p*kPYaw + i*kIYaw; // + d*kDYaw;
		

		if (cstate.servo_pos > 0.25) {
		 	cstate.servo_pos = 0.25;
		} else if (cstate.servo_pos < -0.25) {
		 	cstate.servo_pos = -0.25;
		}

		rc_servo_send_pulse_normalized(1, cstate.servo_pos);
		rc_servo_send_esc_pulse_normalized(7, setpoint.throttle);
		//printf("%lf \n", setpoint.throttle);
	} else {
		if (first_crtlc) {
			printf("\nExiting paraglider\n");
			first_crtlc = 0;
		}
		rc_set_state(EXITING);
		return;
	}

	prev_time = rc_nanos_since_boot();
	prev_error_yaw = error_yaw;

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(rc_get_state()==EXITING){
		rc_servo_send_esc_pulse_normalized(2, 0);
		return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && setpoint.arm_state==ARMED){
		__disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if(setpoint.arm_state==DISARMED){
		return;
	}

	rc_usleep(100);
	return;
}

/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
static int __zero_out_controller(void)
{
	// rc_filter_reset(&D1);
	// rc_filter_reset(&D2);
	// rc_filter_reset(&D3);
	setpoint.yaw = 0.0;
	setpoint.throttle   = 0.0;
	return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
	setpoint.arm_state = DISARMED;
	return 0;
}

/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
static int __arm_controller(void)
{
	__zero_out_controller();
	// prefill_filter_inputs(&D1,cstate.theta);
	setpoint.arm_state = ARMED;
	return 0;
}


/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
static void* __battery_checker(__attribute__ ((unused)) void* ptr)
{
	double new_v;
	while(rc_get_state()!=EXITING){
		new_v = rc_adc_batt();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
static void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			// printf("\nRUNNING: Hold upright to balance.\n");
			// printf("    Y    |");
			// printf("  Y_ref  |");
			// printf("    S    |");
			// printf("  φ_ref  |");
			// printf("    γ    |");
			// printf("  D1_u   |");
			// printf("  D3_u   |");
			// printf("  vBatt  |");
			// printf("arm_state|");
			// printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;

		// decide what to print or exit
		if(new_rc_state == RUNNING){
			// printf("\r");
			// printf("%7.3f  |", cstate.yaw);
			// printf("%7.3f  |", setpoint.yaw);
			// printf("%7.3f  |", cstate.servo_pos);
			// printf("%7.3f  |", setpoint.pitch);
			// printf("%7.3f  |", 0.0);
			// printf("%7.3f  |", 0.0);
			// printf("%7.3f  |", 0.0);
			// printf("%7.3f  |", cstate.vBatt);

			// if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			// else printf("DISARMED |");
			fflush(stdout);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

/**
 * Disarm the controller and set system state to paused. If the user holds the
 * pause button for 2 seconds, exit cleanly
 */
static void __on_pause_press(void)
{
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	switch(rc_get_state()){
	// pause if running
	case EXITING:
		return;
	case RUNNING:
		rc_set_state(PAUSED);
		__disarm_controller();
		rc_led_set(RC_LED_RED,1);
		rc_led_set(RC_LED_GREEN,0);
		break;
	case PAUSED:
		rc_set_state(RUNNING);
		__disarm_controller();
		rc_led_set(RC_LED_GREEN,1);
		rc_led_set(RC_LED_RED,0);
		break;
	default:
		break;
	}

	// now wait to see if the user want to shut down the program
	while(i<samples){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
			return; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	rc_led_blink(RC_LED_RED,5,2);
	rc_set_state(EXITING);
	return;
}

/**
 * toggle between position and angle modes if MiP is paused
 */
static void __on_mode_release(void)
{

	rc_led_blink(RC_LED_GREEN,5,1);
	return;
}
