/* nxtgt.c */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareTask(TaskInitialize);
DeclareTask(TaskControl);
DeclareTask(TaskSonar);
DeclareTask(TaskLCD);

/* Definitions */
#define PASS_KEY           "1234"  /* Bluetooth pass key */
#define MOTOR_LEFT         NXT_PORT_C
#define MOTOR_RIGHT        NXT_PORT_A
#define PORT_SONAR         NXT_PORT_S1
#define PORT_GYRO          NXT_PORT_S2
#define PORT_ACCEL         NXT_PORT_S3
#define PORT_COMPASS       NXT_PORT_S4

// #define STEERING_LIMIT             40 /* degree */
// #define STEERING_P_GAIN             2 /* proportinal gain */
// #define DIFF_GAIN_MAX            0.7F /* 1.0-DIFF_GAIN_MAX: max. differential effect */
// #define NEUTRAL_DEAD_ZONE           2 /* degree */
// #define PWM_OFFSET                 10 /* friction compensator */
// #define EDC_ON                     -1 /* Electronic Differential Control: ON */
// #define EDC_OFF                     1 /* Electronic Differential Control: OFF */
//
// static S8 EDC_flag;    				   /* EDC flag */
#define PID_ON                      -1
#define PID_OFF                      1
static S8 PID_flag = 0;

/* Prototypes */
void robot_status_monitor(const CHAR *target_name);
void robot_bt_data_logger(S8 data1, S8 data2);
void robot_bt_adc_data_logger(S8 data1, S8 data2, S16 adc1, S16 adc2, S16 adc3, U16 adc4, S16 adc5);
// S32 FrictionComp(S32 ratio, S32 offset);

/* PID constants for speed control */
#define sKp 0.0336
#define sKi 0.2688
#define sKd 0.000504

/* Returns the calculated adjustment according to desired speed
*/
int speedPIDControllerLeft(int reference, int d) {
	// Static vars where the PID values are accumulated
	static int integral = 0;
	static int prevError = 0;

	int error = (reference - d);
	integral += error;
	int derivative = (error - prevError);
	int out = (sKp * error) + (sKi * integral) + (sKd * derivative);
	prevError = error;

	if (out>90)
		out = 90;
	if (out<-90)
		out = -90;

	return out;
}

int speedPIDControllerRight(int reference, int d) {
	// Static vars where the PID values are accumulated
	static int integral = 0;
	static int prevError = 0;

	int error = (reference - d);
	integral += error;
	int derivative = (error - prevError);
	int out = (sKp * error) + (sKi * integral) + (sKd * derivative);
	prevError = error;

	if (out>90)
		out = 90;
	if (out<-90)
		out = -90;

	return out;
}

/* LEJOS OSEK hooks */
void ecrobot_device_initialize()
{
	nxt_motor_set_speed(MOTOR_LEFT, 0, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, 0, 1);
	ecrobot_init_accel_sensor(PORT_ACCEL);
	ecrobot_init_compass_sensor(PORT_COMPASS);
	ecrobot_init_sonar_sensor(PORT_SONAR);
	ecrobot_init_bt_slave(PASS_KEY);
}

void ecrobot_device_terminate()
{
	nxt_motor_set_speed(MOTOR_LEFT, 0, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, 0, 1);
	ecrobot_term_accel_sensor(PORT_ACCEL);
	ecrobot_term_compass_sensor(PORT_COMPASS);
	ecrobot_term_sonar_sensor(PORT_SONAR);
	ecrobot_term_bt_connection();
}

void user_1ms_isr_type2(void)
{
	StatusType ercd;

	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if(ercd != E_OK)
  	{
    	ShutdownOS(ercd);
  	}
}

/* TaskInitialize */
TASK(TaskInitialize)
{
	nxt_motor_set_speed(MOTOR_LEFT, 0, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, 0, 1);
	nxt_motor_set_count(MOTOR_LEFT, 0);
	nxt_motor_set_count(MOTOR_RIGHT, 0);

	PID_flag = PID_OFF;

	// EDC_flag = EDC_OFF;

	TerminateTask();
}

/* TaskControl executed every 10msec */
TASK(TaskControl)
{
    U8 cmd_type; /* command type */
	static S32 velocity_left = 0;  /* left motor velocity (or speed percent) */
	static S32 velocity_right = 0; /* right motor velocity (or speed percent) */
	static U8 bt_receive_buf[16];  /* buffer size is fixed as 16  bytes */


	static S32 motor_left = 0;
	static S32 motor_left_prev = 0;
	static S32 motor_right = 0;
	static S32 motor_right_prev = 0;


	/* receive NXTGamePad command
	* byte0 motor_left    -100(backward max.) to 100(forward max.)
	* byte1 motor_right   -100(backward max.) to 100(forward max.)
	*/
	ecrobot_read_bt(bt_receive_buf, 0, 16);

	cmd_type = (U32)(*(U8 *)(&bt_receive_buf[0]));

	if (cmd_type == 1 && PID_flag == 1) {
		PID_flag = ~PID_flag + 1; /* toggle */
	}
	if (cmd_type == 2 && PID_flag == 0) {
		PID_flag = ~PID_flag + 1; /* toggle */
	}

	switch (cmd_type) {
		// Power to left and right motor
		case 1:
			velocity_left =  (S32)(*(S8 *)(&bt_receive_buf[4]));
			velocity_right = (S32)(*(S8 *)(&bt_receive_buf[8]));
			break;
		// PID control
		case 2:
			velocity_left =  (S32)(*(S8 *)(&bt_receive_buf[4]));
			velocity_right = (S32)(*(S8 *)(&bt_receive_buf[8]));
			motor_left_prev = motor_left;
			motor_right_prev =  motor_right;
			motor_left = nxt_motor_get_count(MOTOR_LEFT);
			motor_right = nxt_motor_get_count(MOTOR_RIGHT);
			velocity_left = speedPIDControllerLeft(velocity_left,(motor_left-motor_left_prev));
			velocity_right = speedPIDControllerRight(velocity_right,(motor_right-motor_right_prev));
			break;
		default:
			velocity_left = 0;
			velocity_right = 0;
			break;
	}

	nxt_motor_set_speed(MOTOR_LEFT, velocity_left, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, velocity_right, 1);

	/* send NXT status data to NXT GamePad */
	robot_bt_data_logger((S8)velocity_left, (S8)velocity_right);
	//S16 accel_data[3];
	//ecrobot_get_accel_sensor(PORT_ACCEL, accel_data);

	//ecrobot_bt_adc_data_logger((S8)velocity_left, (S8)velocity_right, accel_data[0], accel_data[1], (S16)ecrobot_get_gyro_sensor(PORT_GYRO), (S16)ecrobot_get_compass_sensor(PORT_COMPASS));
	//robot_bt_adc_data_logger((S8)velocity_left, (S8)velocity_right, accel_data[0], accel_data[1], accel_data[2], (U16)ecrobot_get_gyro_sensor(PORT_GYRO), (S16)ecrobot_get_compass_sensor(PORT_COMPASS));
	TerminateTask();

}

// TASK(TaskControl)
// {
// 	S32 analog_stick_left;  /* speed command data from GamePad */
// 	S32 analog_stick_right; /* steering command data from GamePad */
// 	S32 steering_angle;
// 	S32 steering_err;
// 	S32 steering_speed;
// 	S32 diff_gain;
// 	U8 touch_sensor;
// 	static U8 touch_sensor_state = 0;
// 	static U8 bt_receive_buf[32];  /* buffer size is fixed as 32 */
//
// 	/* receive NXTGamePad command
//    * byte0 speed_data    -100(forward max.) to 100(backward max.)
//    * byte1 steering_data -100(left max.) to 100(right max.)
//    */
// 	ecrobot_read_bt_packet(bt_receive_buf, 32);
// 	analog_stick_left = -(S32)(*(S8 *)(&bt_receive_buf[0])); /* reverse the direction */
// 	analog_stick_right = (S32)(*(S8 *)(&bt_receive_buf[1]));
//
// 	/* read Touch Sensor to switch Electronic Differential Control */
//   	touch_sensor = ecrobot_get_touch_sensor(NXT_PORT_S4);
//   	if (touch_sensor == 1 && touch_sensor_state == 0)
//   	{
//     	EDC_flag = ~EDC_flag + 1; /* toggle */
//   	}
//   	touch_sensor_state = touch_sensor;
//
// 	/* steering control */
// 	steering_angle = nxt_motor_get_count(MOTOR_STEERING);
// 	steering_err = (STEERING_LIMIT*analog_stick_right)/100 - steering_angle;
// 	steering_speed = STEERING_P_GAIN*steering_err;
// 	nxt_motor_set_speed(MOTOR_STEERING, FrictionComp(steering_speed,PWM_OFFSET), 1);
//
// 	/* wheel motors control with Electronic Differential Control */
// 	diff_gain = 10;
// 	if (steering_angle > NEUTRAL_DEAD_ZONE) /* turn right */
// 	{
//     	if (EDC_flag == EDC_ON)
//     	{
//       		diff_gain = (S32)((1.0F - (float)steering_angle*DIFF_GAIN_MAX/STEERING_LIMIT)*10);
//     	}
//     	nxt_motor_set_speed(MOTOR_RIGHT, FrictionComp((analog_stick_left*diff_gain)/10,PWM_OFFSET), 1);
//     	nxt_motor_set_speed(MOTOR_LEFT, FrictionComp(analog_stick_left,PWM_OFFSET), 1);
//   	}
//   	else if (steering_angle < -NEUTRAL_DEAD_ZONE) /* turn left */
//   	{
//     	if (EDC_flag == EDC_ON)
//     	{
//       		diff_gain = (S32)((1.0F + (float)steering_angle*DIFF_GAIN_MAX/STEERING_LIMIT)*10);
//     	}
//     	nxt_motor_set_speed(MOTOR_RIGHT, FrictionComp(analog_stick_left,PWM_OFFSET), 1);
//     	nxt_motor_set_speed(MOTOR_LEFT, FrictionComp((analog_stick_left*diff_gain)/10,PWM_OFFSET), 1);
//   	}
//   	else /* go straight */
//   	{
//     	nxt_motor_set_speed(MOTOR_RIGHT, FrictionComp(analog_stick_left,PWM_OFFSET), 1);
//     	nxt_motor_set_speed(MOTOR_LEFT, FrictionComp(analog_stick_left,PWM_OFFSET), 1);
//   	}
//
// 	/* send NXT status data to NXT GamePad */
// 	ecrobot_bt_data_logger((S8)analog_stick_left, (S8)analog_stick_right);
//
//   	TerminateTask();
// }

/* TaskSonar executed every 50msec */
TASK(TaskSonar)
{
	S32 sonar;

  	/* Sonar Sensor is invoked just for data logging */
	sonar = ecrobot_get_sonar_sensor(PORT_SONAR);

	TerminateTask();
}

/* TaskLCD executed every 500msec */
TASK(TaskLCD)
{
  	robot_status_monitor("NXT GT");


  	TerminateTask();
}

/* Sub functions */
void robot_status_monitor(const CHAR *target_name)
{
	display_clear(0);

	display_goto_xy(0, 0);
   	display_string(target_name);

   	display_goto_xy(0, 1);
   	display_string("TIME:");
   	display_unsigned(systick_get_ms()/1000, 0);

   	display_goto_xy(0, 2);
   	display_string("BATT:");
   	display_unsigned(ecrobot_get_battery_voltage()/100, 0);

   	display_goto_xy(0, 3);
   	display_string("REV: ");
   	//display_int(nxt_motor_get_count(MOTOR_LEFT), 0);
   	//display_int(nxt_motor_get_count(MOTOR_RIGHT), 6);

   	display_goto_xy(0, 4);
   	display_string("PID: ");
   	display_int(PID_flag, 0);

   	display_goto_xy(0, 5);
   	display_string("ADC: ");
   	//display_unsigned(sensor_adc(0), 0);
   	//display_unsigned(sensor_adc(1), 5);

   	display_goto_xy(0, 6);
   	display_string("     ");
   	//display_unsigned(sensor_adc(2), 0);
   	//display_unsigned(sensor_adc(3), 5);

   	display_goto_xy(0, 7);
   	display_string("BT/DST: ");
	if (ecrobot_get_bt_status() == BT_STREAM)
	{
		display_unsigned(1, 0);
   	}
   	else
   	{
		display_unsigned(0, 0);
   	}
   	//display_int(ecrobot_get_sonar_sensor(PORT_SONAR), 5);

	display_update();
}

/**
 * data logging API used with NXT GamePad
 *
 * @param data1: user configurable data to be logged
 * @param data2: user configurable data to be logged
 */
void robot_bt_data_logger(S8 data1, S8 data2)
{
	static U8 data_log_buffer[32];
	static S16 accel_data[3];

	ecrobot_get_accel_sensor(PORT_ACCEL, accel_data);

	*((U32 *)(&data_log_buffer[0]))   = (U32)systick_get_ms();
	*(( S8 *)(&data_log_buffer[4]))   =  (S8)data1;
	*(( S8 *)(&data_log_buffer[5]))   =  (S8)data2;
	*((U16 *)(&data_log_buffer[6]))   = (U16)ecrobot_get_battery_voltage();
	*((S32 *)(&data_log_buffer[8]))   = (S32)nxt_motor_get_count(MOTOR_LEFT);
	*((S32 *)(&data_log_buffer[12]))  = (S32)nxt_motor_get_count(MOTOR_RIGHT);
	*((S16 *)(&data_log_buffer[16]))  = (S16)accel_data[0];
	*((S16 *)(&data_log_buffer[18]))  = (S16)accel_data[1];
	*((S16 *)(&data_log_buffer[20]))  = (S16)accel_data[2];
	*((U16 *)(&data_log_buffer[22]))  = (U16)ecrobot_get_gyro_sensor(PORT_GYRO);
	*((S16 *)(&data_log_buffer[24]))  = (S16)ecrobot_get_compass_sensor(PORT_COMPASS);
	*((S16 *)(&data_log_buffer[26]))  = (S16)0;
	*((S32 *)(&data_log_buffer[28]))  = (S32)ecrobot_get_sonar_sensor(PORT_SONAR);

	ecrobot_send_bt(data_log_buffer, 0, 32);
}

void robot_bt_adc_data_logger(S8 data1, S8 data2, S16 adc1, S16 adc2, S16 adc3, U16 adc4, S16 adc5)
{
	static U8 data_log_buffer[32];

	*((U32 *)(&data_log_buffer[0]))  = (U32)systick_get_ms();
	*(( S8 *)(&data_log_buffer[4]))  =  (S8)data1;
	*(( S8 *)(&data_log_buffer[5]))  =  (S8)data2;
	*((U16 *)(&data_log_buffer[6]))  = (U16)ecrobot_get_battery_voltage();
	*((S32 *)(&data_log_buffer[8]))  = (S32)nxt_motor_get_count(MOTOR_LEFT);
	*((S32 *)(&data_log_buffer[12])) = (S32)nxt_motor_get_count(MOTOR_RIGHT);
	*((S16 *)(&data_log_buffer[16])) = (S16)adc1;
	*((S16 *)(&data_log_buffer[18])) = (S16)adc2;
	*((S16 *)(&data_log_buffer[20])) = (S16)adc3;
	*((U16 *)(&data_log_buffer[22])) = (U16)adc4;
	*((S16 *)(&data_log_buffer[24])) = (S16)adc5;
	*((S16 *)(&data_log_buffer[26])) = (S16)0;
	*((S32 *)(&data_log_buffer[28])) = (S32)ecrobot_get_sonar_sensor(PORT_SONAR);
	//adc[0] = adc1;
	//adc[1] = adc2;
	//adc[2] = adc3;
	//adc[3] = adc4;

	ecrobot_send_bt(data_log_buffer, 0, 32);
}
// S32 FrictionComp(S32 ratio, S32 offset)
// {
//   	if (ratio > 0)
//   	{
//     	return ((100-offset)*ratio/100 + offset);
//   	}
//   	else if (ratio < 0)
//   	{
//     	return ((100-offset)*ratio/100 - offset);
//   	}
//   	else
//   	{
//     	return ratio;
//   	}
// }
