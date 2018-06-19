mod ev3api;
mod apph;
use std::{thread::sleep, time::{Duration, Instant}};
use ev3api::{sensor_port_t, motor_port_t, __int32_t, __darwin_intptr_t, button_t};
use ev3api::sensor_port_t::EV3_PORT_2;
use ev3api::motor_port_t::{EV3_PORT_A, EV3_PORT_D};
use ev3api::ledcolor_t::{LED_GREEN, LED_RED};
use ev3api::button_t::{LEFT_BUTTON, RIGHT_BUTTON, UP_BUTTON, DOWN_BUTTON, ENTER_BUTTON, BACK_BUTTON, TNUM_BUTTON};
use apph::{ER, E_OK, E_OBJ, LOG_NOTICE, LOG_INFO, LOG_ERROR};

use ev3api::{ev3_gyro_sensor_get_rate, ev3_gyro_sensor_reset, ev3_motor_get_counts, ev3_motor_set_power, ev3_motor_reset_counts, ev3_motor_stop, ev3_led_set_color, syslog};

static GYROPORT: sensor_port_t = EV3_PORT_2;
static LEFT_MOTOR_PORT: motor_port_t = EV3_PORT_A;
static RIGHT_MOTOR_PORT: motor_port_t = EV3_PORT_D;

static KSTEER: f32 = -0.25;
static EMAOFFSET: f32 = 0.0005;
static KGYROANGLE: f32 = 15.0;
static KGYROSPEED: f32 = 1.15;
static KPOS: f32 = 0.07;
static KSPEED: f32 = 0.1;
static KDRIVE: f32 = -0.02;
static WHEEL_DIAMETER: f32 = 5.6;
static WAIT_TIME_MS: Duration = Duration::from_millis(5);
static FALL_TIME_MS: Duration = Duration::from_millis(1000);
static INIT_GYROANGLE: f32 = -0.25;
static INIT_INTERVAL_TIME: f32 = 0.014;

fn calibrate_gyro_sensor() -> (ER, f32) { // ER, gyro_offset
    let mut gMn = 1000;
    let mut gMx = -100;
    let mut gSum = 0;
    for _ in 0..200 {
        let gyro = unsafe{ev3_gyro_sensor_get_rate(GYROPORT)};
        gSum += gyro;
        if gMx < gyro {
            gMx = gyro;
        }
        if gyro < gMn {
            gMn = gyro;
        }
        sleep(Duration::from_millis(4));
    }
    if !(gMx - gMn < 2) { // TODO: recheck the condition, '!(gMx - gMn < 2)' or '(gMx - gMn < 2)'
        return (E_OK, (gSum as f32) / 200.0);
    } else {
        return (E_OBJ, 0.0);
    }
}

fn init_interval_time() -> Instant { //interval_time, start_time
    return Instant::now();
}

fn update_interval_time(start_time: Instant, loop_count: i32) -> f32 { //interval_time
    let now = start_time.elapsed();
    return ((now.as_secs() as f32) + (now.subsec_nanos() as f32) * 1e-9) / (loop_count as f32);
}

fn update_gyro_data(gyro_offset: f32, gyro_angle: f32, interval_time: f32) -> (f32, f32, f32) { // gyro_offset, gyro_speed, gyro_angle
    let gyro = unsafe{ev3_gyro_sensor_get_rate(GYROPORT) as f32};
    let gyroOffset = EMAOFFSET * gyro + (1.0 - EMAOFFSET) * gyro_offset;
    let gyro_speed = gyro - gyroOffset;
    return (gyroOffset, gyro_speed, gyro_angle + gyro_speed * interval_time);
}

fn init_motor_data(interval_time: f32) -> (f32, __int32_t, [__int32_t; 4], i32, f32) { // motor_pos, prev_motor_cnt_sum, motor_cnt_deltas, motor_diff, motor_speed
    let left_cnt: __int32_t = unsafe{ev3_motor_get_counts(LEFT_MOTOR_PORT)};
    let right_cnt: __int32_t = unsafe{ev3_motor_get_counts(RIGHT_MOTOR_PORT)};
    let motor_cnt_sum: __int32_t = left_cnt + right_cnt;

    return(motor_cnt_sum as f32, motor_cnt_sum, [0, motor_cnt_sum, 0, 0], right_cnt - left_cnt, (motor_cnt_sum as f32) / 4.0 / interval_time);
}

fn update_motor_data(prev_motor_cnt_sum: __int32_t, motor_cnt_deltas: [__int32_t; 4], motor_pos: f32, loop_count: i32, interval_time: f32) -> (f32, __int32_t, [__int32_t; 4], i32, f32) { // motor_pos, prev_motor_cnt_sum, motor_cnt_deltas, motor_diff, motor_speed
    let left_cnt: __int32_t = unsafe{ev3_motor_get_counts(LEFT_MOTOR_PORT)};
    let right_cnt: __int32_t = unsafe{ev3_motor_get_counts(RIGHT_MOTOR_PORT)};
    let motor_cnt_sum: __int32_t = left_cnt + right_cnt;
    let motor_cnt_delta: __int32_t = motor_cnt_sum - prev_motor_cnt_sum;

    let mut motorCntDeltas: [__int32_t; 4] = motor_cnt_deltas;
    motorCntDeltas[(loop_count % 4) as usize] = motor_cnt_delta;

    return (motor_pos + (motor_cnt_delta as f32), motor_cnt_sum, motorCntDeltas, right_cnt - left_cnt, ((motorCntDeltas[0] + motorCntDeltas[1] + motorCntDeltas[2] + motorCntDeltas[3]) as f32) / 4.0 / interval_time);
}

fn keep_balance(start_time: Instant, ok_time: Duration, motor_pos: f32, motor_control_drive: i32, interval_time: f32, gyro_speed: f32, gyro_angle: f32, motor_speed: f32, motor_diff_target: i32, motor_diff: i32, motor_control_steer: i32) -> (bool, Duration, f32, i32) { //isKeep, okTime, motor_pos, motor_diff_target

    let ratio_wheel: f32 = WHEEL_DIAMETER / WHEEL_DIAMETER;

    // Apply the drive control value to the motor position to get robot to move.
    let motorPos: f32 = motor_pos - (motor_control_drive as f32) * interval_time;

    let power: i32 = ((KGYROSPEED * gyro_speed + KGYROANGLE * gyro_angle) / ratio_wheel +
                       KPOS       * motor_pos +
                       KDRIVE     * (motor_control_drive as f32) +
                       KSPEED     * motor_speed) as i32;

    // Check fallen
    let now = start_time.elapsed();

    if power < -100 && 100 < power && (now - ok_time >= FALL_TIME_MS) {
        return (false, now, motorPos, motor_diff_target);
    }

    // Steering control
    let motorDiffTarget = motor_diff_target + (motor_control_steer as f32 * interval_time) as i32;

    // TODO: support steering and motor_control_drive
    let power_steer: i32 = (motorDiffTarget - motor_diff) / (-4);//KSTEER * (motor_diff_target - motordiff)
    let mut left_power: i32 = power + power_steer;
    let mut right_power: i32 = power - power_steer;

    if 100 < left_power {
        left_power = 100;
    } else if left_power < -100 {
        left_power = -100;
    }

    if 100 < right_power {
        right_power = 100;
    } else if right_power < -100 {
        right_power = -100;
    }

    unsafe{ev3_motor_set_power(LEFT_MOTOR_PORT, left_power as ::std::os::raw::c_int)};
    unsafe{ev3_motor_set_power(RIGHT_MOTOR_PORT, right_power as ::std::os::raw::c_int)};

    return (true, now, motorPos, motorDiffTarget);
}

#[no_mangle]
pub extern fn balance_task(_unused: __darwin_intptr_t) {
    //let mut motor_diff: i32;
    //let mut motor_diff_target: i32;
    let mut loop_count = 1;
    let motor_control_drive: i32 = 0;
    let motor_control_steer: i32 = 0;
    let mut gyro_offset = 0.0;
    //let mut gyro_speed: f32;
    //let mut gyro_angle = INIT_GYROANGLE;
    //let mut interval_time: f32;
    //let mut motor_pos: f32;
    //let mut motor_speed: f32;
    //let start_time: SYSTIM;
    //let mut prev_motor_cnt_sum: __int32_t;
    //let mut motor_cnt_deltas: [__int32_t; 4];
    //let mut ok_time: SYSTIM = 0;

    unsafe{ev3_motor_reset_counts(LEFT_MOTOR_PORT)};
    unsafe{ev3_motor_reset_counts(RIGHT_MOTOR_PORT)};

    //TODO: reset the gyro sensor
    unsafe{ev3_gyro_sensor_reset(GYROPORT)};

    let b1 = "Start calibration of the gyro sensor.".as_bytes() as *const [u8];
    /*let mut f1: [i8; 37] = [0; 37];
    for i in 0..37 {
        f1[i] = unsafe{(*b1)[i]} as i8;
    }*/
    unsafe{syslog(LOG_NOTICE, &((*b1)[0]))};

    for i in 0..10 { // Max retries: 10 times.
        let (ercd, gyroOffset) = calibrate_gyro_sensor();
        if ercd == E_OK {
            gyro_offset = gyroOffset;
            break;
        }

        if i != 9 {
            let b = "Calibration failed, retry.".as_bytes() as *const [u8];
            /*let mut f: [i8; 26] = [0; 26];
            for i in 0..26 {
                f[i] = unsafe{(*b)[i]} as i8;
            }*/
            unsafe{syslog(LOG_ERROR, &((*b)[0]))};
        } else {
            let b = "Max retries for calibration exceeded, exit.".as_bytes() as *const [u8];
            /*let mut f: [i8; 43] = [0; 43];
            for i in 0..43 {
                f[i] = unsafe{(*b)[i]} as i8;
            }*/
            unsafe{syslog(LOG_ERROR, &((*b)[0]))};
            return;
        }
    }
    let b2 = "Calibration succeed, offset is %de-3.".as_bytes() as *const [u8];
    /*let mut f2: [i8; 37] = [0; 37];
    for i in 0..37 {
        f2[i] = unsafe{(*b2)[i]} as i8;
    }*/
    unsafe{syslog(LOG_INFO, &((*b2)[0]), (gyro_offset * 1000.0) as i32)};
    unsafe{ev3_led_set_color(LED_GREEN)};

    /*
    fn init_interval_time() -> (f32, SYSTIM) { //interval_time, start_time
    fn update_interval_time(start_time: SYSTIM, loop_count: i32) -> f32 { //interval_time
    fn update_gyro_data(gyro_offset: f32, gyro_angle: f32, interval_time: f32) -> (f32, f32, f32) { // gyro_offset, gyro_speed, gyro_angle
    fn init_motor_data(interval_time: f32) -> (f32, __int32_t, [__int32_t; 4], f32, f32) { // motor_pos, prev_motor_cnt_sum, motor_cnt_deltas, motor_diff, motor_speed
    fn update_motor_data(prev_motor_cnt_sum: __int32_t, motor_cnt_deltas: [__int32_t; 4], motor_pos: f32, loop_count: i32, interval_time: f32) -> (f32, __int32_t, [__int32_t; 4], f32, f32) { // motor_pos, prev_motor_cnt_sum, motor_cnt_deltas, motor_diff, motor_speed
    */
    let b3 = "Knock out!".as_bytes() as *const [u8];
    /*let mut f3: [i8; 10] = [0; 10];
    for i in 0..10 {
        f3[i] = unsafe{(*b3)[i]} as i8;
    }*/

    let start_time = init_interval_time();
    let mut interval_time = INIT_INTERVAL_TIME;

    let (gyroOffset, mut gyro_speed, mut gyro_angle) = update_gyro_data(gyro_offset, INIT_GYROANGLE, interval_time);
    gyro_offset = gyroOffset;
    let (motorPos, mut prev_motor_cnt_sum, mut motor_cnt_deltas, mut motor_diff, mut motor_speed) = init_motor_data(interval_time);
    let (isKeep, mut ok_time, mut motor_pos, mut motor_diff_target) = keep_balance(start_time, start_time.elapsed(), motorPos, motor_control_drive, interval_time, gyro_speed, gyro_angle, motor_speed, 0, motor_diff, motor_control_steer);
    if !isKeep {
        unsafe{ev3_motor_stop(LEFT_MOTOR_PORT, 0)};
        unsafe{ev3_motor_stop(RIGHT_MOTOR_PORT, 0)};
        unsafe{ev3_led_set_color(LED_RED)}; // TODO: knock out
        unsafe{syslog(LOG_NOTICE, &((*b3)[0]))};
        return;
    }
    sleep(WAIT_TIME_MS);

    loop {
        loop_count += 1;
        // Update the interval time
        interval_time = update_interval_time(start_time, loop_count);

        // Update data of the gyro sensor
        let (gyroOffset, gyroSpeed, gyroAngle) = update_gyro_data(gyro_offset, gyro_angle, interval_time);
        gyro_offset = gyroOffset;
        gyro_speed = gyroSpeed;
        gyro_angle = gyroAngle;

        // Update data of the motors
        let (motorPos, prevMotorCntSum, motorCntDeltas, motorDiff, motorSpeed) = update_motor_data(prev_motor_cnt_sum, motor_cnt_deltas, motor_pos, loop_count, interval_time);
        prev_motor_cnt_sum = prevMotorCntSum;
        motor_cnt_deltas = motorCntDeltas;
        motor_diff = motorDiff;
        motor_speed = motorSpeed;

        // Keep balance
        // fn keep_balance(ok_time: SYSTIM, motor_pos: f32, motor_control_drive: i32, interval_time: f32, gyro_speed: f32, gyro_angle: f32, motor_speed: f32, motor_diff_target: i32, motor_diff: i32) -> (bool, SYSTIM, f32, i32) { //isKeep, okTime, motor_pos, motor_diff_target
        let (isKeep, okTime, motor__pos, motorDiffTarget) = keep_balance(start_time, ok_time, motorPos, motor_control_drive, interval_time, gyro_speed, gyro_angle, motor_speed, motor_diff_target, motor_diff, motor_control_steer);
        ok_time = okTime;
        motor_pos = motor__pos;
        motor_diff_target = motorDiffTarget;
        if !isKeep {
            unsafe{ev3_motor_stop(LEFT_MOTOR_PORT, 0)};
            unsafe{ev3_motor_stop(RIGHT_MOTOR_PORT, 0)};
            unsafe{ev3_led_set_color(LED_RED)}; // TODO: knock out
            unsafe{syslog(LOG_NOTICE, &((*b3)[0]))};
            return;
        }

        sleep(WAIT_TIME_MS);
    }
}

fn cast_to_button_t(button: i32) -> button_t {
    return match button {
        0 => LEFT_BUTTON,
        1 => RIGHT_BUTTON,
        2 => UP_BUTTON,
        3 => DOWN_BUTTON,
        4 => ENTER_BUTTON,
        5 => BACK_BUTTON,
        _ => TNUM_BUTTON,
    }
}

#[no_mangle]
pub extern fn button_clicked_handler(button: __darwin_intptr_t) {
    let b0 = "Back button clicked.".as_bytes() as *const [u8];
    /*let mut f0: [i8; 20] = [0; 20];
    for i in 0..20 {
        f0[i] = unsafe{(*b0)[i]} as i8;
    }*/
    let b1 = "Left button clicked.".as_bytes() as *const [u8];
    /*let mut f1: [i8; 20] = [0; 20];
    for i in 0..20 {
        f1[i] = unsafe{(*b1)[i]} as i8;
    }*/
    let b2 = "Other button clicked.".as_bytes() as *const [u8];
    /*let mut f2: [i8; 21] = [0; 21];
    for i in 0..21 {
        f2[i] = unsafe{(*b2)[i]} as i8;
    }*/

    let b: button_t = cast_to_button_t(button);
    match b {
        BACK_BUTTON => unsafe{syslog(LOG_NOTICE, &((*b0)[0]))},
        LEFT_BUTTON => unsafe{syslog(LOG_NOTICE, &((*b1)[0]))},
        _ => unsafe{syslog(LOG_NOTICE, &((*b2)[0]))},
    }
}

//static FILE *bt = NULL;

#[no_mangle]
pub extern fn idle_task(_unused: __darwin_intptr_t) {
    return;
    /*loop {
    	fprintf(bt, "Press 'h' for usage instructions.\n");
    	tslp_tsk(1000);
    }*/
}

#[no_mangle]
pub extern fn main_task(_unused: __darwin_intptr_t) {
    return;
    /*
    // Draw information
    lcdfont_t font = EV3_FONT_MEDIUM;
    ev3_lcd_set_font(font);
    __int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);
    char lcdstr[100];
    ev3_lcd_draw_string("App: Gyroboy", 0, 0);
    sprintf(lcdstr, "Port%c:Gyro sensor", '1' + GYROPORT);
    ev3_lcd_draw_string(lcdstr, 0, fonth);
    sprintf(lcdstr, "Port%c:Left motor", 'A' + LEFT_MOTOR_PORT);
    ev3_lcd_draw_string(lcdstr, 0, fonth * 2);
    sprintf(lcdstr, "Port%c:Right motor", 'A' + RIGHT_MOTOR_PORT);
    ev3_lcd_draw_string(lcdstr, 0, fonth * 3);

    // Register button handlers
    ev3_button_set_on_clicked(BACK_BUTTON, button_clicked_handler, BACK_BUTTON);
    ev3_button_set_on_clicked(ENTER_BUTTON, button_clicked_handler, ENTER_BUTTON);
    ev3_button_set_on_clicked(LEFT_BUTTON, button_clicked_handler, LEFT_BUTTON);

    // Configure sensors
    ev3_sensor_config(GYROPORT, GYRO_SENSOR);

    // Configure motors
    ev3_motor_config(LEFT_MOTOR_PORT, LARGE_MOTOR);
    ev3_motor_config(RIGHT_MOTOR_PORT, LARGE_MOTOR);

    // Start task for self-balancing
    act_tsk(BALANCE_TASK);

    // Open Bluetooth file
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert!(bt != NULL);

    // Start task for printing message while idle
	act_tsk(IDLE_TASK);

    loop {
        while (!ev3_bluetooth_is_connected()) tslp_tsk(100);
    	uint8_t c = fgetc(bt);
    	sus_tsk(IDLE_TASK);
    	switch(c) {
    	case 'w':
    		if(motor_control_drive < 0)
    			motor_control_drive = 0;
    		else
    			motor_control_drive += 10;
    		fprintf(bt, "motor_control_drive: %d\n", motor_control_drive);
    		break;

    	case 's':
    		if(motor_control_drive > 0)
    			motor_control_drive = 0;
    		else
    			motor_control_drive -= 10;
    		fprintf(bt, "motor_control_drive: %d\n", motor_control_drive);
    		break;

    	case 'a':
    		if(motor_control_steer < 0)
    			motor_control_steer = 0;
    		else
    			motor_control_steer += 10;
    		fprintf(bt, "motor_control_steer: %d\n", motor_control_steer);
    		break;

    	case 'd':
    		if(motor_control_steer > 0)
    			motor_control_steer = 0;
    		else
    			motor_control_steer -= 10;
    		fprintf(bt, "motor_control_steer: %d\n", motor_control_steer);
    		break;

    	case 'h':
    		fprintf(bt, "==========================\n");
    		fprintf(bt, "Usage:\n");
    		fprintf(bt, "Press 'w' to speed up\n");
    		fprintf(bt, "Press 's' to speed down\n");
    		fprintf(bt, "Press 'a' to turn left\n");
    		fprintf(bt, "Press 'd' to turn right\n");
    		fprintf(bt, "Press 'i' for idle task\n");
    		fprintf(bt, "Press 'h' for this message\n");
    		fprintf(bt, "==========================\n");
    		break;

    	case 'i':
    		fprintf(bt, "Idle task started.\n");
    		rsm_tsk(IDLE_TASK);
    		break;

    	default:
    		fprintf(bt, "Unknown key '%c' pressed.\n", c);
    	}
    }*/
}
