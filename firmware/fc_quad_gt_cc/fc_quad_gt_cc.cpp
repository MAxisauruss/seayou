#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include <hardware/uart.h>
#include <hardware/dma.h>
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <stddef.h>


#define Main_Loop_Time_MICRO_SECONDS 5000

//--------------EKF--------------//
#define mag_inclination 33.9363060f

// 4096 LSB/g = +/-8g full scale (ACCEL_CONFIG AFS_SEL=2, set in
// mpu6050_init). Was 16384 = +/-2g, which is far too little headroom on a
// quadcopter: vibration peaks clip against the 2g rail, and because the
// clipping is asymmetric about the 1g gravity vector it RECTIFIES into a
// DC offset. The EKF then tilts its gravity reference by that offset.
//
// MEASURED peaks with props OFF at full throttle (2026-09-03): 0.97g,
// 0.73g, 3.26g per axis. The old +/-2g rail clipped hard against that
// 3.26g; +/-16g was tried and gave away 8x resolution for no benefit, so
// +/-8g is the middle: 2.5x headroom over the real peaks, 4096 LSB/g kept.
//
// Earlier bench data, props OFF, drone stationary:
//   throttle 0-14%  -> roll 0.00  pitch 0.00
//   throttle 42%    -> roll -0.18 pitch -2.54
//   throttle 100%   -> roll -6.15 pitch -6.10
//   throttle back 0 -> roll 0.00  pitch 0.00
// A 6 deg error is a ~0.1g rectified bias, and it tracked motor speed on a
// drone that never moved. Filtering cannot fix this - the clipping happens
// at the ADC, before the DLPF.
//
// The accel offsets and accel_cal matrix below stay valid: they are applied
// in g, after this division, so only the scale constant has to change.
#define MPU6050_LSB_PER_g 4096.0f
// Accelerometer software low-pass strength (EMA). ~3.3 Hz cutoff at 200 Hz.
// Lower = heavier filtering / more lag. See the filter in IMU_Read().
#define ACCEL_LPF_ALPHA 0.10f
#define accel_offset_x  0.016940f
#define accel_offset_y -0.015371f
#define accel_offset_z -0.103156f
const float accel_cal[3][3] = {
    {  1.000822f, -0.000070f, -0.000529f },
    { -0.000070f,  0.999569f, -0.001073f },
    { -0.000529f, -0.001073f,  1.003285f }
};



//--------------CompassCal--------------//
#define QMC5883_LSB_PER_G 12000.0f
#define QMC5883_Gauss_to_uT 100.0f

float compass_offset_x = 3.522593f;
float compass_offset_y = -7.581154f;
float compass_offset_z = 11.464800f;
float mag_cal[3][3] = {
    {  0.997198f, -0.034667f, -0.000293f },
    { -0.034667f,  1.034516f, -0.016239f },
    { -0.000293f, -0.016239f,  1.001262f }
};

#define compass_Cal_buff_max_expected_len 100
uint8_t compassCalHoldBuff[compass_Cal_buff_max_expected_len];
bool isCompassCalibrated = false;
float cal_arr[9];
//---------------------------------------//

//--------------BMP388--------------//
// Quantized (float) calibration coefficients per the Bosch BMP388
// datasheet section 9.3 "Floating-point compensation" - read once from
// the sensor's NVM trim registers at init and converted here, then reused
// every read. Same shared I2C0 bus as the MPU6050/QMC5883 above.
struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
} bmp388_cal;
float baro_temp_c = 0.0f;
float baro_pressure_pa = 0.0f;
bool isBaroPresent = false;
//-----------------------------------//

static int addr = 0x68;
static int addr1 = 0x0D;
// BMP388 default I2C address when SDO is tied/pulled high (the common
// default on most breakout boards). If the board uses SDO-low, this is
// 0x76 instead - check the specific board if the barometer doesn't
// respond (chip ID read will read as 0x00 / bus NACK either way).
static int addr2 = 0x77;
float loop_time;
uint32_t time, timePrev;
int16_t accel[3], gyro[3], mag[3];
int32_t gyro_cal[3];
// Gyro zero-rate bias sweep, run once at boot in EKF_Init().
// 2000 samples at ~1 ms is ~2 s of averaging. The reject threshold is in
// raw LSB: at +/-2000 dps the scale is ~16.4 LSB per deg/s, so 300 LSB is
// about 18 deg/s peak-to-peak - far above the sensor's own noise sitting
// still, but well below anything that counts as the airframe being moved.
#define GYRO_CAL_SAMPLES     2000
#define GYRO_CAL_MAX_SPREAD  300
bool is_gyro_calibrated = false;
// Times the EKF quaternion went NaN and had to be reinitialised. Should be
// 0 forever; anything else means the filter is being fed something bad.
uint8_t ekf_nan_resets = 0;
// DIAGNOSTIC: peak |raw accel| per axis since the last telemetry packet, in
// raw LSB. Telemetry only samples at 30 Hz so an instantaneous reading
// aliases vibration badly; the peak-hold catches what the sensor actually
// sees. At +/-16g the rail is 32767 LSB, so a peak parked near that means
// the accelerometer is clipping.
uint16_t accel_peak[3] = {0, 0, 0};
// DIAGNOSTIC: running SUM and count of raw accel between telemetry packets,
// so the mean can be reported. The mean is what actually matters for a tilt
// error: a 7 deg attitude error is a mean gravity vector rotated by 7 deg,
// i.e. a ~0.12g DC shift. Peaks say how hard it is shaking; only the mean
// says whether the shaking is being RECTIFIED into a false gravity
// direction, which is the thing that tilts the estimate.
int32_t accel_sum[3] = {0, 0, 0};
uint16_t accel_n = 0;

// |accel| in g, BEFORE normalisation. 1.0 when the only thing the sensor
// feels is gravity; anything else means it is also feeling vibration.
float accel_mag_g = 1.0f;
// DIAGNOSTIC: |mag| in uT before normalisation. The compass was calibrated
// with the motors OFF, so if this shifts when they spin, motor current is
// generating a magnetic field the calibration knows nothing about - and the
// EKF fuses the magnetometer into the whole quaternion, so that pulls roll
// and pitch, not just yaw.
float mag_mag_ut = 0.0f;
// Vibration-adaptive accelerometer trust. The accel is the EKF's gravity
// reference, but a shaken MEMS part rectifies vibration into a DC bias that
// no amount of filtering removes - it happens inside the sensor. So when
// |accel| departs from 1g, raise its measurement noise and let the (now
// bias-calibrated) gyro carry the attitude through the shake instead.
// Quadratic so gentle noise barely changes anything while real vibration
// pushes trust down hard: dev 0.05 -> R 0.40, dev 0.5 -> R 15, dev 0.75 -> R 34.
#define ACCEL_R_BASE 0.25f
#define ACCEL_R_GAIN 60.0f
float sigma, ax,ay,az,mx,my,mz, gx,gy,gz,ry,rz;
float wx,wy,wz, qw,qx,qy,qz;
float P[4][4];
float R[2];
float roll,pitch,yaw, wx_crct, wy_crct, wz_crct, q_prev[4];
//leveling :
float   roll_offset_angle, pitch_offset_angle,   q_leveled[4], q_level_rot[4] = { 1, 0, 0, 0 };
//-------------------------------//


//----------Level zero-point----------//
// The sensor board is never mounted perfectly parallel to the frame, so
// "sensors level" and "drone level" differ by a small fixed rotation.
// Captured on command ('L','C') with the drone standing still on a level
// surface: average the EKF's own roll/pitch at full float precision and
// store the negative of that as the zero point. It has to happen here on
// the Pico, not on the Pi - the telemetry packet encodes each quaternion
// component as int(q*100)+100, which is ~1.15 degrees per step, far too
// coarse to level anything from the other end of the link.
// Kept in the last flash sector so it survives a power cycle, and added
// to (not replacing) the pilot's manual trim bytes.
#define LEVEL_CAL_FLASH_OFFSET   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define LEVEL_CAL_MAGIC          0x4C56454Cu
#define LEVEL_CAL_SETTLE_LOOPS   200      /* 200 x 5 ms = 1.0 s discarded */
#define LEVEL_CAL_SAMPLE_LOOPS   400      /* 400 x 5 ms = 2.0 s averaged  */
#define LEVEL_CAL_MAX_TRIM_DEG   15.0f    /* beyond this, remount the board */
#define LEVEL_CAL_MAX_RATE_RADS  0.06f    /* ~3.4 deg/s of CHANGE - see below */
#define LEVEL_CAL_ABS_RATE_RADS  1.0f     /* ~57 deg/s - actively being waved */

#define LEVEL_CAL_IDLE      0
#define LEVEL_CAL_SETTLING  1
#define LEVEL_CAL_SAMPLING  2

#define LEVEL_RES_NONE      0
#define LEVEL_RES_OK        1
#define LEVEL_RES_MOVED     2
#define LEVEL_RES_RANGE     3
#define LEVEL_RES_THROTTLE  4

float    stored_roll_offset = 0.0f, stored_pitch_offset = 0.0f;
uint8_t  level_cal_state = LEVEL_CAL_IDLE;
uint8_t  level_cal_result = LEVEL_RES_NONE;
uint16_t level_cal_loops = 0;
float    level_cal_sum_roll = 0.0f, level_cal_sum_pitch = 0.0f;
bool     level_cal_cmd_latched = false;
// Gyro reading while standing still, learned during the settle window.
// gyro_cal[] is all zeros - the bias sweep in EKF_Init() is commented out -
// so wx/wy/wz carry the raw MPU6050 bias, several deg/s even on a bench.
// The stillness test therefore has to measure change from this baseline,
// not absolute rate, or it can never pass.
float    level_cal_w_sum[3] = { 0.0f, 0.0f, 0.0f };
float    level_cal_w_mean[3] = { 0.0f, 0.0f, 0.0f };

struct LevelCalRecord {
    uint32_t magic;
    float    roll;
    float    pitch;
    uint32_t checksum;
};
//------------------------------------//


//--------------PID--------------//
#define Roll_PID_lim 400
#define Pitch_PID_lim 400
#define Yaw_PID_lim 400
// Reduced 3.0 -> 2.0 on 2026-09-04. With the CORRECT (full-size) props the
// drone produces far more thrust per unit of PID output than it did on the
// undersized props these gains were set with, so the attitude loop went
// unstable above ~75% throttle: a flight-log oscillation grew from 3 deg to
// 90 deg in ~1.5 s (sticks centred the whole time) and put it in the
// ceiling. Lower loop gain is the fix; this and twoX_P_gain below are cut
// together. STARTING POINT - needs a low, tethered test to confirm, then
// tune back up only if it feels sluggish.
#define angle_to_rate_gain 2.0f
int Roll_PID, Pitch_PID, Yaw_PID;
float pid_Integral[3], pid_Derivative[3];
float error[3], prev_error[3], setpoint_free_error[3], setpoint_free_prev_error[3], desired_angular_rate[3];
float twoX_P_gain = 1.7f, twoX_I_gain = 0.0045f, twoX_D_gain = 0.03f;
float Yaw_P_gain = 1.2f, Yaw_I_gain = 0.0045f, Yaw_D_gain = 0.03f;
float ctrl_roll = 0, ctrl_pitch = 0, ctrl_yaw = 0;
//-------------------------------//

//--------------PWM--------------//
int motor_out[4] = { 0, 0, 0, 0};
int ctrl_channel[4] = { 50, 50, 0, 50};
//-------------------------------//


//-------------UART0--------------//
#define uart0_in_buff_size	13
// 81, not 79: bytes 78-79 now carry the VSYS rail voltage. See the
// rail-voltage block in Tx_Rx_Update_Variables().
#define uart0_out_buff_size 81
uint8_t uart0_in_buff[uart0_in_buff_size], uart0_out_buff[uart0_out_buff_size];
bool uart0_is_receiving = false;
volatile uint32_t uart0_irq_time_stamp;
#define not_recvd_since_threshold   1000000 /* in microseconds : t*1000*1000 : t is in seconds */
//-------------------------------//


//--------------GPS--------------//
#define gps_buf_max_expctd_len 500
uint8_t gps_raw_buff[gps_buf_max_expctd_len];
// Filtered VSYS, volts. The Pico ties VSYS to ADC3 through an internal
// 3:1 divider, so this is the drone's 5 V rail measured on-board with no
// extra hardware - and per V4_Wiring_Diagram.png it is the SAME rail that
// feeds the GPS module's VCC and the I2C sensors.
//
// Around 4.8 V means the rail is only being backfed from USB through the
// Pico's Schottky; 5.0 V or more means the ESC BEC is actually supplying
// it. That distinction is the difference between the GPS having a supply
// of its own and not.
float vsys_volts = 0.0f;

volatile int gps_array_pos = 0; // exclusive access by dma0 - read from the main
                               // loop in GPS_decode(), so it must be volatile
int dma_chan1 = 0;
uint8_t Lattitude[10], Longitude[11], Sat_count, Fix_type, HDOP[3], GND_velocity[5];
uint8_t gps_decode_loop_shape_count;

uint32_t time_Stamp, Prev_time_Stamp;
//-------------------------------//


//-------GPS_Position_Hold-------//
#define Lattitude_shifted_myGPS_offset  -0.0000345f
#define Longitude_shifted_myGPS_offset  -0.0000717f
#define ctrl_stick_to_velocity_div_gain  0.1f

#define GPHC_MAX_Horizontal_Velocity        8.0f        // in meters per second.
#define GPHC_Vehicle_MAX_Lean_Angle         25.0f        // in degrees

bool is_GPS_mode_ON = false, is_GPS_Hold_once_run_done = false;
float GPHC_P_gain = 7.0f, GPHC_I_gain = 0.03f, GPHC_D_gain = 7.0f;
double LAT_DEG_TO_METERS=0, LON_DEG_TO_METERS=0;
double Vehicle_Lattitude=0, Vehicle_Longitude=0, Vehicle_Lattitude_Prev=0, Vehicle_Longitude_Prev=0, Vehicle_desired_Lattitude=0, Vehicle_desired_Longitude=0;
float velocity_vector_north_frame[2], velocity_vector_north_frame_Prev[2];
float v_p_n_f_error[2];
float GPHC_roll_P, GPHC_roll_I, GPHC_roll_D,  GPHC_pitch_P, GPHC_pitch_I, GPHC_pitch_D,  GPHC_roll_angle_out_body, GPHC_pitch_angle_out_body,  GPHC_roll_angle_out_north, GPHC_pitch_angle_out_north;
float Vehicle_desired_lattitude_body_frame_increment, Vehicle_desired_longitude_body_frame_increment,Vehicle_desired_lattitude_north_frame_increment, Vehicle_desired_longitude_north_frame_increment, yaw_rad;
//-------------------------------//

//--------------LED--------------//
uint8_t loop_div_counter;
uint8_t counter_temp = 0;
//-------------------------------//

//--------------CMD--------------//
uint8_t in_cmd[2];
uint8_t out_status[8];
//-------------------------------//


struct Quaternion{
    float w, x, y, z;
};

struct Angles{
    float roll, pitch, yaw;
};

Quaternion quat_inv(Quaternion q){
    static Quaternion q_inv;
    q_inv.w = q.w;
    q_inv.x = -q.x;
    q_inv.y = -q.y;
    q_inv.z = -q.z;
    return q_inv;
}

Quaternion q1_dot_q2 ( float a, float b, float c, float d,     float e, float f, float g, float h ){
    static Quaternion q;
    q.w = (a*e - b*f - c*g - d*h);
    q.x = (a*f + b*e + c*h - d*g);
    q.y = (a*g - b*h + c*e + d*f);
    q.z = (a*h + b*g - c*f + d*e);
    return q;
}

Quaternion get_Quaternion_from_bodyframe_angles( float roll, float pitch, float yaw ){
    Quaternion q_ = q1_dot_q2( cosf(yaw*0.0174532925f*0.5f), 0, 0, sinf(yaw*0.0174532925f*0.5f),     cosf(roll*0.0174532925f*0.5f), 0, sinf(roll*0.0174532925f*0.5f), 0 );
    Quaternion q = q1_dot_q2( q_.w, q_.x, q_.y, q_.z,     cosf(pitch*0.0174532925f*0.5f), sinf(pitch*0.0174532925f*0.5f), 0, 0 );
    return q;
}

Angles get_error_angles_from_Quaternion(Quaternion q){
    static Angles angles;
    // angles.roll = atan2f( 2.0f * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y) ) * 57.2957795f;
    // angles.pitch = (2.0f * atan2f(sqrt(1.0f + 2.0f * (q.w * q.y - q.x * q.z)), sqrt(1.0f - 2.0f * (q.w * q.y - q.x * q.z))) - M_PI/2.0f) * 57.2957795f;
    // angles.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 57.2957795f;

    angles.pitch = 57.2958f*atan2(2*(q.w*q.x + q.y*q.z) , 1 - 2*(q.x*q.x + q.y*q.y));
    angles.roll  = 57.2958f*asin(2*(q.w*q.y - q.z*q.x));
    angles.yaw   = 57.2958f*atan2( (2*(q.w*q.z + q.x*q.y)) , (1 - 2*(q.y*q.y + q.z*q.z)));

    // float temp_x, temp_y;

    return angles;
}

static void Led_init(){
    sleep_ms(1);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

static void Led_set(){
    loop_div_counter++;
    if(uart0_is_receiving){
        if(is_GPS_mode_ON){
            if(loop_div_counter == 30){ gpio_put(PICO_DEFAULT_LED_PIN,1); }
            else if(loop_div_counter == 10 || loop_div_counter == 40){ gpio_put(PICO_DEFAULT_LED_PIN,0); }
            else if(loop_div_counter == 140){ gpio_put(PICO_DEFAULT_LED_PIN,1); loop_div_counter = 0; }
        }
        else if(!is_GPS_mode_ON){
            if(loop_div_counter == 10 ){ gpio_put(PICO_DEFAULT_LED_PIN,0); }
            else if(loop_div_counter == 140){ gpio_put(PICO_DEFAULT_LED_PIN,1); loop_div_counter = 0; }
        }

    }
    else if(!uart0_is_receiving){
            if(loop_div_counter == 125){ gpio_put(PICO_DEFAULT_LED_PIN,0); }
        else if(loop_div_counter == 250){ gpio_put(PICO_DEFAULT_LED_PIN,1); loop_div_counter = 0; }
    }
    
}

static void Is_uart0_receiving_and_Action(){
    if ( ( time_us_32() - uart0_irq_time_stamp ) <= not_recvd_since_threshold ) { uart0_is_receiving = true; }
    else if ( ( time_us_32() - uart0_irq_time_stamp ) > not_recvd_since_threshold ) { uart0_is_receiving = false; }

    if(!uart0_is_receiving){ ctrl_channel[0] =50; ctrl_channel[1] =50; ctrl_channel[2] =0; ctrl_channel[3] =50; }
}

static void UART0_irq_OnRecv(){
    if(isCompassCalibrated){
        if(uart_is_readable(uart0)){
            uart_read_blocking(uart0, uart0_in_buff, uart0_in_buff_size);
            if(uart_is_writable(uart0)){
                uart_write_blocking(uart0, uart0_out_buff, uart0_out_buff_size);
            }
            uart0_irq_time_stamp = time_us_32();
        }
    }
    else {
        if(uart_is_readable(uart0)){
            uart_read_blocking(uart0, compassCalHoldBuff, compass_Cal_buff_max_expected_len);   isCompassCalibrated = true;
            if(uart_is_writable(uart0)){
                uart_write_blocking(uart0, compassCalHoldBuff, compass_Cal_buff_max_expected_len);
            }
            uart0_irq_time_stamp = time_us_32();
        }
    }
}

static void UART0_setup(int baud){
    // Moved from GP12/13 to GP16/17 - GP12 (UART0 TX) tested as non-functional
    // on this board (no bytes ever reaching the Pi5's RX no matter how the
    // wiring was checked), on a Pico that already has one other confirmed
    // damaged GPIO (GP6, see PWM_out_init()). GP14/15 were tried in between
    // but are UART0 CTS/RTS, not TX/RX, and could never have worked - the
    // RP2040's UART0 TX/RX pins repeat in blocks of 4 (0/1, 12/13, 16/17),
    // and GP16/17 is the next real TX/RX pair, unused anywhere else here.
    sleep_ms(20);
    uart_init(uart0, 2400);
    gpio_set_function(16, GPIO_FUNC_UART);
    gpio_set_function(17, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(uart0, baud);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, false);
    irq_set_exclusive_handler(UART0_IRQ, UART0_irq_OnRecv);
    irq_set_priority(UART0_IRQ, 1);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
    sleep_ms(20);
}

//----------------------------------------------------------------------//
// USB CDC host link - replaces UART0 (GP16/17) as the actual transport
// to the Pi5. UART0_setup()/UART0_irq_OnRecv() above are left in place
// but unused (not called from main()) in case this ever needs reverting;
// they touch different pins so there's no conflict leaving them defined.
//
// This reuses the Pico's own USB port, which was already carrying power
// from the Pi5's USB port over a real USB cable - a physically far more
// robust connection than the jumper-wire UART link that preceded it.
//
// Deliberately not using stdio_usb's printf/getchar layer for the actual
// protocol traffic (binary framed packets, not text) - stdio_usb_init()
// is used only to bring up TinyUSB and its automatic background IRQ task
// servicing (confirmed in pico-sdk's stdio_usb.c: it calls tusb_init()
// and wires up a low-priority IRQ that keeps tud_task() serviced without
// needing to be pumped from the main loop). Raw tud_cdc_read()/write()
// calls below talk to the same underlying CDC buffers directly.
//----------------------------------------------------------------------//

// Shared assembly buffer for USB_CDC_read_exact() - was function-local
// static, moved to file scope so USB_CDC_reset_input_state() can clear it.
static uint8_t usb_cdc_read_buf[compass_Cal_buff_max_expected_len];
static uint32_t usb_cdc_read_buf_len = 0;

// If the Pico's power was never actually cycled (only the Pi/dashboard
// service restarted around it, e.g. mid-session on USB power that stayed
// up) it can already be sitting in the normal main loop - past
// USB_CDC_wait_for_calibration() entirely - reading everything in 13-byte
// (uart0_in_buff_size) chunks. A fresh 100-byte calibration payload sent
// into that state lands as 7 complete 13-byte reads (91 bytes) plus a
// 9-byte leftover fragment that never completes a read and is never
// reset - confirmed by a live byte-level trace: every subsequent real
// control packet landed rotated by a fixed 5 bytes (13-9-...=... the
// leftover eats into the next real packet's head), so
// uart0_in_buff[0]/[12] never actually held '$'/'*' again, the framing
// check in Tx_Rx_Update_Variables() silently failed forever, and
// ctrl_channel[] stayed frozen at its {50,50,0,50} startup default -
// motors permanently reading zero throttle no matter what was sent.
// Call this once right when calibration is confirmed done (whichever of
// the two ways that happened) so the main loop's 13-byte reads always
// start from a clean, empty buffer.
static void USB_CDC_reset_input_state(){
    usb_cdc_read_buf_len = 0;
}

static bool USB_CDC_read_exact(uint8_t *dst, uint32_t len){
    while(tud_cdc_available() && usb_cdc_read_buf_len < len){
        usb_cdc_read_buf_len += tud_cdc_read(usb_cdc_read_buf + usb_cdc_read_buf_len, len - usb_cdc_read_buf_len);
    }
    if(usb_cdc_read_buf_len >= len){
        memcpy(dst, usb_cdc_read_buf, len);
        usb_cdc_read_buf_len = 0;
        return true;
    }
    return false;
}

static void USB_CDC_wait_for_calibration(){
    stdio_usb_init();
    while(!USB_CDC_read_exact(compassCalHoldBuff, compass_Cal_buff_max_expected_len)){
        tight_loop_contents();
    }
    isCompassCalibrated = true;
    tud_cdc_write(compassCalHoldBuff, compass_Cal_buff_max_expected_len);
    tud_cdc_write_flush();
    USB_CDC_reset_input_state();
    uart0_irq_time_stamp = time_us_32();
}

static bool usb_cdc_has_fresh_input = false;

// Called early each loop, before Is_uart0_receiving_and_Action()/
// Tx_Rx_Update_Variables() - so a freshly read packet is already sitting
// in uart0_in_buff in time for this same cycle's processing, exactly as
// if it had just arrived over UART0.
//
// Self-resyncing on '$': confirmed by a live byte-level trace that a
// single desync event (e.g. the 100-byte calibration payload getting
// consumed by these ordinary 13-byte reads instead of the dedicated
// USB_CDC_wait_for_calibration() path - which can happen even on a clean
// boot, since the handshake typically takes 2 attempts and the Pico can
// finish attempt 1 before the Pi's own timeout does) rotates every
// subsequent packet by a fixed offset forever, since the plain
// accumulate-13-bytes-then-check approach has no way to recover once
// misaligned - '$' never lands at index 0 again, the framing check in
// Tx_Rx_Update_Variables() silently fails every cycle, and ctrl_channel[]
// (throttle included) stays frozen at its startup default permanently.
// Discarding bytes that don't look like a real packet start, one at a
// time, until '$' is found guarantees recovery within at most
// uart0_in_buff_size-1 stray bytes, whatever caused the desync.
static void USB_CDC_Read(){
    static uint8_t buf[uart0_in_buff_size];
    static uint32_t buf_len = 0;

    while(tud_cdc_available() && buf_len < uart0_in_buff_size){
        if(buf_len == 0){
            uint8_t b;
            if(tud_cdc_read(&b, 1) != 1){ break; }
            if(b != '$'){ continue; }
            buf[0] = b;
            buf_len = 1;
        } else {
            buf_len += tud_cdc_read(buf + buf_len, uart0_in_buff_size - buf_len);
        }
    }

    if(buf_len < uart0_in_buff_size){
        usb_cdc_has_fresh_input = false;
        return;
    }

    if(buf[uart0_in_buff_size - 1] == '*'){
        memcpy(uart0_in_buff, buf, uart0_in_buff_size);
        buf_len = 0;
        usb_cdc_has_fresh_input = true;
        uart0_irq_time_stamp = time_us_32();
    } else {
        // Started with '$' but the expected '*' isn't where it should be -
        // either genuine corruption or a coincidental '$'-valued data byte
        // that wasn't really a frame start. Drop just the leading byte and
        // let the next call keep resyncing from byte 1 onward, rather than
        // discarding the whole window and losing more data than necessary.
        memmove(buf, buf + 1, buf_len - 1);
        buf_len -= 1;
        usb_cdc_has_fresh_input = false;
    }
}

// Called after Tx_Rx_Update_Variables() - so the reply carries this same
// cycle's freshly computed telemetry rather than the previous cycle's.
static void USB_CDC_Reply(){
    if(usb_cdc_has_fresh_input){
        // tud_cdc_write()'s return value is the number of bytes actually
        // queued into TinyUSB's CDC TX FIFO, which can be less than
        // requested - any shortfall was previously discarded silently,
        // which looks exactly like a permanently truncated reply from the
        // Pi's side (observed: a consistent 5-byte reply instead of the
        // full 34). Loop until everything is queued. Bounded to 5ms so a
        // stalled USB link can never block the main loop - and therefore
        // ESC PWM updates / the no-data failsafe - for more than that.
        uint32_t written = 0;
        uint32_t deadline = time_us_32() + 5000;
        while(written < uart0_out_buff_size && time_us_32() < deadline){
            uint32_t n = tud_cdc_write(uart0_out_buff + written, uart0_out_buff_size - written);
            if(n > 0){
                written += n;
                tud_cdc_write_flush();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// NMEA receiver.
//
// Rewritten 20 Sep 2026. The original parser could not read the module that
// is actually fitted, for four independent reasons - any one of which alone
// produces "0 satellites, no fix, forever":
//
//   1. It required the 'N' of a $GNxxx talker ID. A GPS-only receiver says
//      $GPGGA, so the test failed on every sentence. Now any talker is
//      accepted ($GP, $GN, $GL, $GA, $GB ...).
//   2. GPS_init() forced the receiver to 921600 baud with a u-blox UBX
//      message. A non-u-blox module ignores it and stays at 9600 while the
//      Pico listens at 921600 - silence. Baud is now discovered, not assumed.
//   3. Fields were cut at fixed character offsets, which assumes every
//      receiver pads identically (5 decimal places, 4-character HDOP). Real
//      sentences vary. Parsing is now comma-delimited, and the sentence
//      checksum is verified before anything is believed.
//   4. The hemisphere character was never stored. The Pi decodes the packet
//      with _nmea_to_decimal(), which requires a trailing N/S/E/W and returns
//      None without one - so position could never have reached the dashboard
//      even if a fix existed. It also meant southern latitudes came through
//      positive, which here is a 50 km error in the wrong hemisphere.
//
// Fix_type now also distinguishes silence from searching, which the old code
// could not express: '0' = no valid NMEA arriving at all (dead module, wiring
// or baud), '1' = receiver alive and talking but no satellite fix yet,
// '2'/'3' = 2D/3D fix. Byte 26 of the packet carries it as-is.
// ---------------------------------------------------------------------------

// Baud candidates, most likely first. The hunt below rotates through these
// until valid sentences appear, then stays put. 9600 is the default of every
// module considered for this airframe (ATGM336H, NEO-6M, NEO-M8N).
static const int gps_baud_candidates[] = { 9600, 38400, 115200, 57600, 4800 };
#define GPS_BAUD_CANDIDATE_COUNT  (sizeof(gps_baud_candidates)/sizeof(gps_baud_candidates[0]))
#define GPS_BAUD_HUNT_TIMEOUT_US  3000000   /* no valid sentence for 3 s -> try the next baud */

static uint8_t  gps_baud_idx = 0;
static uint32_t gps_last_sentence_us = 0;
static uint16_t gps_read_pos = 0;           // chases gps_array_pos around the DMA ring

static char    nmea_line[100];
static uint8_t nmea_len = 0;
static bool    nmea_in_sentence = false;

// Diagnostics, readable over SWD or a debugger and used for Fix_type above.
uint32_t gps_valid_sentences = 0;           // checksum-passing sentences since boot
uint32_t gps_checksum_errors = 0;
// Words the DMA has delivered from the UART.
//
// NOT a trustworthy measure of "is data arriving", and it must not be read as
// one. Measured 20 Sep: this climbed steadily at ~50/s while a direct GPIO
// read of the same pin showed 8.68 MILLION consecutive samples with the line
// high and not one transition, and every baud rate from 4800 to 230400
// decoded silence with the UART error bits read explicitly. The DMA re-arms
// on completion and delivers a word whether or not that word came from a real
// start bit, so a counter of "words the DMA handed over" counts artifacts as
// readily as data.
//
// gps_line_transitions below is the honest measure, and the two are reported
// side by side precisely so they can be compared.
uint32_t gps_raw_bytes = 0;

// High-to-low transitions on GP5, sampled off the pad once per main loop.
//
// TREAT AS A HINT, NOT AS EVIDENCE. Measured 20 Sep: this counted ~10/s at
// the same time as the dedicated gps_sniffer build - sampling the same pin
// several million times a second, and decoding the UART with the error bits
// read explicitly at six baud rates - measured a line that was static at
// 3.3 V with ZERO transitions and not one received word.
//
// Both in-loop counters therefore pick up something that is not data. Most
// likely aliasing: sampling at the loop rate says nothing reliable about a
// signal, and this loop also runs I2C, PWM and USB alongside. Whatever the
// cause, neither counter can settle "is the GPS transmitting".
//
// The authoritative test is the separate gps_sniffer firmware in
// ../gps_sniffer/, which does nothing else and reads the pin directly.
uint32_t gps_line_transitions = 0;
static bool gps_line_prev = true;
uint8_t  gps_sats_in_view = 0;              // from GGA, meaningful with or without a fix
int      gps_locked_baud = 0;               // 0 until a sentence has ever been decoded

// Fix state is derived from both sentences rather than either alone.
//
// GGA carries a fix quality and GSA carries 2D/3D, and not every receiver
// emits both - a GGA-only module would otherwise parse a perfectly good
// position and have it thrown away for want of a GSA that is never coming.
// Equally, once GSA IS being emitted, a loss reported by either sentence is
// treated as a loss: stale coordinates on an aircraft are worse than none.
static uint8_t gps_gga_quality = 0;         // GGA field 6; 0 = no fix
static uint8_t gps_gsa_fix     = 1;         // GSA field 2; 1 none, 2 2D, 3 3D
static bool    gps_gsa_seen    = false;     // does this receiver emit GSA at all?

static bool hex_nibble(char c, uint8_t *out){
    if(c >= '0' && c <= '9'){ *out = (uint8_t)(c - '0');      return true; }
    if(c >= 'A' && c <= 'F'){ *out = (uint8_t)(c - 'A' + 10); return true; }
    if(c >= 'a' && c <= 'f'){ *out = (uint8_t)(c - 'a' + 10); return true; }
    return false;
}

// "$GPGGA,...*4A" -> true when the XOR of everything between '$' and '*'
// matches the two hex digits after it. Without this a corrupted sentence -
// which at a wrong baud rate is most of them - can still look parseable.
static bool nmea_checksum_ok(const char *s, uint8_t len){
    if(len < 4 || s[0] != '$'){ return false; }
    uint8_t sum = 0;
    uint8_t i = 1;
    for(; i < len && s[i] != '*'; i++){ sum ^= (uint8_t)s[i]; }
    if((uint16_t)i + 2 >= (uint16_t)len){ return false; }   // no '*', or too few digits after it
    uint8_t hi, lo;
    if(!hex_nibble(s[i+1], &hi) || !hex_nibble(s[i+2], &lo)){ return false; }
    return sum == (uint8_t)((hi << 4) | lo);
}

// Points *start at comma-separated field `idx` (0 = "$GPGGA") and returns its
// length. Length 0 means the field is absent or empty - which for GGA is the
// normal state before a fix, not an error.
static uint8_t nmea_field(const char *s, uint8_t len, uint8_t idx, const char **start){
    uint8_t field = 0;
    uint8_t begin = 0;
    for(uint8_t i = 0; i < len; i++){
        if(s[i] == ',' || s[i] == '*'){
            if(field == idx){ *start = &s[begin]; return (uint8_t)(i - begin); }
            field++;
            begin = (uint8_t)(i + 1);
            if(s[i] == '*'){ break; }
        }
    }
    *start = &s[0];
    return 0;
}

// "ddmm.mmmm" / "dddmm.mmmm" -> degrees, unsigned. Accepts any number of
// decimal places, which is the whole point - the old fixed-offset cut assumed
// exactly five and silently mangled anything else.
static double nmea_coord_to_degrees(const char *f, uint8_t len, bool is_lat, bool *ok){
    const uint8_t deg_len = is_lat ? 2 : 3;
    *ok = false;
    if(len < (uint8_t)(deg_len + 3)){ return 0.0; }

    double degrees = 0.0;
    for(uint8_t i = 0; i < deg_len; i++){
        if(f[i] < '0' || f[i] > '9'){ return 0.0; }
        degrees = degrees * 10.0 + (double)(f[i] - '0');
    }

    double minutes = 0.0;
    uint8_t i = deg_len;
    for(; i < len && f[i] != '.'; i++){
        if(f[i] < '0' || f[i] > '9'){ return 0.0; }
        minutes = minutes * 10.0 + (double)(f[i] - '0');
    }
    if(i < len && f[i] == '.'){
        i++;
        double scale = 0.1;
        for(; i < len; i++){
            if(f[i] < '0' || f[i] > '9'){ return 0.0; }
            minutes += (double)(f[i] - '0') * scale;
            scale *= 0.1;
        }
    }
    if(minutes >= 60.0){ return 0.0; }      // not a coordinate

    *ok = true;
    return degrees + minutes / 60.0;
}

// Signed degrees -> the exact ASCII layout server.py::_nmea_to_decimal()
// expects: "ddmm.mmmm" + N/S in 10 bytes, "dddmm.mmmm" + E/W in 11. Rebuilding
// it from the parsed value rather than copying raw bytes is what makes the
// packet independent of how the receiver pads its own fields.
static void format_nmea_coord(double deg, bool is_lat, uint8_t *out){
    const char hemi = is_lat ? (deg < 0.0 ? 'S' : 'N')
                             : (deg < 0.0 ? 'W' : 'E');
    if(deg < 0.0){ deg = -deg; }

    uint32_t whole = (uint32_t)deg;
    uint32_t min_e4 = (uint32_t)((deg - (double)whole) * 600000.0 + 0.5);   // minutes x 10000
    if(min_e4 > 599999u){ min_e4 = 599999u; }                              // rounding guard
    const uint32_t mins = min_e4 / 10000u;
    const uint32_t frac = min_e4 % 10000u;

    uint8_t p = 0;
    if(!is_lat){ out[p++] = (uint8_t)('0' + (whole / 100u) % 10u); }
    out[p++] = (uint8_t)('0' + (whole / 10u) % 10u);
    out[p++] = (uint8_t)('0' +  whole        % 10u);
    out[p++] = (uint8_t)('0' + (mins / 10u)  % 10u);
    out[p++] = (uint8_t)('0' +  mins         % 10u);
    out[p++] = '.';
    out[p++] = (uint8_t)('0' + (frac / 1000u) % 10u);
    out[p++] = (uint8_t)('0' + (frac / 100u)  % 10u);
    out[p++] = (uint8_t)('0' + (frac / 10u)   % 10u);
    out[p++] = (uint8_t)('0' +  frac          % 10u);
    out[p++] = (uint8_t)hemi;
}

// $xxGGA - position, fix quality, satellites used, HDOP.
static void nmea_parse_GGA(const char *s, uint8_t len){
    const char *lat_f;  const uint8_t lat_n  = nmea_field(s, len, 2, &lat_f);
    const char *ns_f;   const uint8_t ns_n   = nmea_field(s, len, 3, &ns_f);
    const char *lon_f;  const uint8_t lon_n  = nmea_field(s, len, 4, &lon_f);
    const char *ew_f;   const uint8_t ew_n   = nmea_field(s, len, 5, &ew_f);
    const char *qual_f; const uint8_t qual_n = nmea_field(s, len, 6, &qual_f);
    const char *sat_f;  const uint8_t sat_n  = nmea_field(s, len, 7, &sat_f);
    const char *hdop_f; const uint8_t hdop_n = nmea_field(s, len, 8, &hdop_f);

    // Satellite count is recorded whether or not there is a fix - watching it
    // climb is how you tell "searching" from "dead", and it is the number the
    // dashboard shows while you wait.
    if(sat_n > 0 && sat_n <= 2){
        uint8_t n = 0;
        bool digits = true;
        for(uint8_t i = 0; i < sat_n; i++){
            if(sat_f[i] < '0' || sat_f[i] > '9'){ digits = false; break; }
            n = (uint8_t)(n * 10 + (sat_f[i] - '0'));
        }
        if(digits){ gps_sats_in_view = n; Sat_count = n; }
    }

    // HDOP is variable width in the wild ("1.2", "0.95", "12.4"). Take the
    // first three characters and pad, rather than assuming exactly four.
    HDOP[0] = HDOP[1] = HDOP[2] = '0';
    for(uint8_t i = 0; i < 3 && i < hdop_n; i++){ HDOP[i] = (uint8_t)hdop_f[i]; }

    // Quality 0 means no fix; anything else means the position fields are real.
    gps_gga_quality = (uint8_t)((qual_n > 0 && qual_f[0] >= '1' && qual_f[0] <= '9')
                                ? (qual_f[0] - '0') : 0);
    if(gps_gga_quality == 0 || lat_n == 0 || lon_n == 0 || ns_n == 0 || ew_n == 0){
        gps_gga_quality = 0;
        return;
    }

    bool lat_ok = false, lon_ok = false;
    double lat = nmea_coord_to_degrees(lat_f, lat_n, true,  &lat_ok);
    double lon = nmea_coord_to_degrees(lon_f, lon_n, false, &lon_ok);
    if(!lat_ok || !lon_ok){ return; }

    if(ns_f[0] == 'S' || ns_f[0] == 's'){ lat = -lat; }
    if(ew_f[0] == 'W' || ew_f[0] == 'w'){ lon = -lon; }

    Vehicle_Lattitude = lat - (double)Lattitude_shifted_myGPS_offset;
    Vehicle_Longitude = lon - (double)Longitude_shifted_myGPS_offset;

    format_nmea_coord(Vehicle_Lattitude, true,  Lattitude);
    format_nmea_coord(Vehicle_Longitude, false, Longitude);
}

// $xxGSA - field 2 is the fix type: 1 none, 2 2D, 3 3D.
static void nmea_parse_GSA(const char *s, uint8_t len){
    const char *f; const uint8_t n = nmea_field(s, len, 2, &f);
    if(n > 0 && f[0] >= '1' && f[0] <= '3'){
        gps_gsa_fix  = (uint8_t)(f[0] - '0');
        gps_gsa_seen = true;
    }
}

static void nmea_handle_sentence(const char *s, uint8_t len){
    if(!nmea_checksum_ok(s, len)){ gps_checksum_errors++; return; }

    gps_valid_sentences++;
    gps_last_sentence_us = time_us_32();
    gps_locked_baud = gps_baud_candidates[gps_baud_idx];

    // "$GPGGA" - talker is s[1..2] and varies by constellation, so only the
    // three sentence-type characters are matched.
    if(len < 6){ return; }
    if(s[3] == 'G' && s[4] == 'G' && s[5] == 'A'){ nmea_parse_GGA(s, len); }
    else if(s[3] == 'G' && s[4] == 'S' && s[5] == 'A'){ nmea_parse_GSA(s, len); }
}

static void GPS_decode(){
    // Sample the RX line itself before touching the UART. This is the only
    // GPS diagnostic here that cannot be confused by peripheral behaviour -
    // see gps_line_transitions.
    {
        const bool lvl = gpio_get(5);
        if(gps_line_prev && !lvl && gps_line_transitions < 0xFFFFFFFFu){
            gps_line_transitions++;
        }
        gps_line_prev = lvl;
    }

    // Drain everything the DMA has landed since last time. This runs every
    // loop iteration, not at 10 Hz: at 115200 baud the 500-byte ring fills in
    // 43 ms, so a 100 ms parse interval would drop most of it on the floor.
    const uint16_t write_pos = (uint16_t)gps_array_pos;
    while(gps_read_pos != write_pos){
        const char c = (char)gps_raw_buff[gps_read_pos];
        gps_read_pos = (uint16_t)((gps_read_pos + 1) % gps_buf_max_expctd_len);
        if(gps_raw_bytes < 0xFFFFFFFFu){ gps_raw_bytes++; }

        if(c == '$'){
            nmea_in_sentence = true;
            nmea_len = 0;
            nmea_line[nmea_len++] = c;
        } else if(nmea_in_sentence){
            if(c == '\r' || c == '\n'){
                if(nmea_len >= 6){ nmea_handle_sentence(nmea_line, nmea_len); }
                nmea_in_sentence = false;
                nmea_len = 0;
            } else if(nmea_len < (uint8_t)(sizeof(nmea_line) - 1)){
                nmea_line[nmea_len++] = c;
            } else {
                nmea_in_sentence = false;   // overlong: garbage, resync on the next '$'
                nmea_len = 0;
            }
        }
    }

    // Baud hunt. Nothing valid for 3 s means either the receiver is silent or
    // we are listening at the wrong rate; there is no way to tell which from
    // here, so rotate. Once sentences arrive this stops firing and the rate
    // stays put. gps_locked_baud records what worked.
    const uint32_t now = time_us_32();
    if((uint32_t)(now - gps_last_sentence_us) > GPS_BAUD_HUNT_TIMEOUT_US){
        gps_baud_idx = (uint8_t)((gps_baud_idx + 1) % GPS_BAUD_CANDIDATE_COUNT);
        uart_set_baudrate(uart1, gps_baud_candidates[gps_baud_idx]);
        gps_last_sentence_us = now;

        // Whatever is mid-flight in the ring was framed at the old rate.
        gps_read_pos = (uint16_t)gps_array_pos;
        nmea_in_sentence = false;
        nmea_len = 0;

        // Silence is its own state, distinct from "searching for satellites".
        Fix_type = '0';
        Sat_count = 0;
        gps_gga_quality = 0;
        gps_gsa_fix = 1;
        gps_gsa_seen = false;
    } else if(gps_valid_sentences > 0){
        // Sentences are arriving. A fix needs GGA to report one, and - only
        // if this receiver emits GSA at all - GSA to agree. GSA then decides
        // 2D versus 3D.
        const bool have_fix = (gps_gga_quality > 0)
                              && (!gps_gsa_seen || gps_gsa_fix >= 2);
        if(have_fix){ Fix_type = (gps_gsa_seen && gps_gsa_fix == 3) ? '3' : '2'; }
        else        { Fix_type = '1'; }
    }

    if(Fix_type == '0' || Fix_type == '1'){
        Vehicle_Lattitude = 0; Vehicle_Longitude = 0;
    }
}

static void UART1_setup(int baud){
    sleep_ms(50);
    uart_init(uart1, 2400);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(uart1, baud);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, false);
    sleep_ms(20);
}

static void GPS_init(){
    // Deliberately does NOT reconfigure the receiver any more.
    //
    // The previous version sent u-blox UBX messages to force 921600 baud and
    // 10 Hz. That is correct for a genuine u-blox M8 and wrong for everything
    // else: a CASIC or non-u-blox chip ignores the UBX frame, stays at 9600,
    // and then the Pico switches its own UART to 921600 and hears nothing for
    // the rest of the flight - a failure that looks exactly like a dead module
    // and cannot be recovered from without a reflash. Listening at the
    // receiver's own rate always works; discovering that rate costs at most a
    // few seconds at boot.
    //
    // The cost is that a 9600-baud module reports at 1 Hz, not 10 Hz. That is
    // a real limit on the velocity estimate used by position hold - but
    // position hold has never had a fix to work with, and a 1 Hz fix is worth
    // considerably more than a 10 Hz one that does not exist.
    gps_baud_idx = 0;
    uart_set_baudrate(uart1, gps_baud_candidates[gps_baud_idx]);
    gps_last_sentence_us = time_us_32();
    gps_read_pos = 0;
    Fix_type = '0';
    Sat_count = 0;
    gps_gga_quality = 0;
    gps_gsa_fix = 1;
    gps_gsa_seen = false;
    sleep_ms(20);
}

static void DMA0_irq_handler() {
    dma_hw->ints0 = 1u << dma_chan1;
    dma_channel_set_write_addr(dma_chan1, &gps_raw_buff[gps_array_pos], true);
    gps_array_pos = (gps_array_pos+1)%gps_buf_max_expctd_len;
}

static void DMA0_configure() {
    // NOT "int dma_chan1" - that declared a local that shadowed the global of
    // the same name, leaving the global at 0 while the real channel was
    // configured. DMA0_irq_handler() uses the global, so the two only agreed
    // because this happens to be the first channel claimed and gets 0. Claim
    // one more channel anywhere above this line and GPS reception dies.
    dma_chan1 = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_chan1);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, uart_get_dreq(uart1, false));
    dma_channel_configure(
        dma_chan1,
        &config,
        NULL,
        &uart1_hw->dr,
        1,
        false
    );
    dma_channel_set_irq0_enabled(dma_chan1, true);
    irq_set_exclusive_handler(DMA_IRQ_0, DMA0_irq_handler);
    irq_set_priority(UART0_IRQ, 0);
    irq_set_enabled(DMA_IRQ_0, true);
    DMA0_irq_handler();
    sleep_ms(50);
}

static void PWM_out_init(){
    // Original pin mapping, restored for a fresh Pico with no known pin
    // damage - GP6/GP9 were only moved to GP10/GP11 on the previous
    // (damaged) board. GP6/7 -> slice 3 (ch A/B), GP8/9 -> slice 4 (ch A/B).
    //
    // clkdiv=40, wrap=62500 -> 125MHz/(40*62501) ~= 50.0Hz (20ms period),
    // standard RC/ESC refresh rate. Previously ran at ~200Hz (clkdiv=10) -
    // confirmed by direct A/B hardware test (a known-good MicroPython
    // script driving these exact ESCs at 50Hz armed and spun the motors
    // fine; this firmware's 200Hz signal produced zero response despite
    // correct pulse widths measured at the pin) that these ESCs silently
    // ignore anything above 50Hz rather than tolerating it, as many newer
    // ESCs do - so the refresh rate itself, not the pulse width, was the
    // actual fault all along.
    gpio_set_function(6, GPIO_FUNC_PWM);
    gpio_set_function(7, GPIO_FUNC_PWM);
    gpio_set_function(8, GPIO_FUNC_PWM);
    gpio_set_function(9, GPIO_FUNC_PWM);
    pwm_set_clkdiv(3,40);
    pwm_set_wrap(3,62500);
    pwm_set_enabled(3, true);
    pwm_set_clkdiv(4,40);
    pwm_set_wrap(4,62500);
    pwm_set_enabled(4, true);
}

static void PWM_Write(){
    // Pulse width is 1100us at motor_out=0 (idle/disarmed) up to 2000us at
    // motor_out=1000 (max), at the 50Hz rate set up in PWM_out_init().
    // Idle floor raised from the previous 1000us - some ESCs sit too close
    // to their own internal signal-loss threshold right at 1000us and
    // refuse to arm reliably; 1100us gives them margin.
    // Outputs remapped 2026-09-01. The MPU6050 is mounted 90 degrees round
    // from the airframe - measured on the bench: lifting the FRONT edge
    // moves the firmware's ROLL (+20.4 deg), lifting the RIGHT edge moves
    // its PITCH (+20.6 deg). So the mixer's idea of front/right is rotated
    // a quarter turn from the frame's, and the stock GP6..GP9 order put
    // three of the four motors in the wrong corner. The drone could not
    // roll level: the front pair fought each other, one output sat at its
    // floor barely turning while another saturated, and it rolled over on
    // takeoff.
    //
    // Roll_PID comes out opposite in sign to the roll angle and m0/m3 carry
    // +Roll_PID, so front-up must cut front thrust => {m0,m3} = FRONT.
    // Same argument on pitch => {m0,m1} = RIGHT. That fixes all four
    // uniquely:
    //     m0 = front-right   m1 = rear-right
    //     m2 = rear-left     m3 = front-left
    // Physical positions confirmed with motor_test (spin order was
    // front-right, front-left, rear-right, rear-left on GP6..GP9).
    pwm_set_chan_level(3, PWM_CHAN_A, int(3437.5f+2.8125f*motor_out[0])); // GP6 = front-right
    pwm_set_chan_level(3, PWM_CHAN_B, int(3437.5f+2.8125f*motor_out[3])); // GP7 = front-left
    pwm_set_chan_level(4, PWM_CHAN_A, int(3437.5f+2.8125f*motor_out[1])); // GP8 = rear-right
    pwm_set_chan_level(4, PWM_CHAN_B, int(3437.5f+2.8125f*motor_out[2])); // GP9 = rear-left
}

static void I2C_Init(){
    sleep_ms(1);
    i2c_init(i2c0, 400000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
}

// Every I2C transaction below is time-bounded. The plain i2c_*_blocking
// calls block FOREVER if the bus stalls - a slave holding SDA low, an
// unterminated clock stretch, a glitch on the shared bus - and these run
// on every single loop iteration via IMU_Read() and Baro_Read().
//
// That is not merely a lockup. When the main loop stops, the PWM slices
// keep outputting their last duty cycle, so the motors keep spinning at
// whatever throttle they had, and Is_uart0_receiving_and_Action()'s
// no-signal failsafe never runs again. One bad bus cycle in flight is a
// fly-away with no way to command it down.
//
// Observed on the bench 2026-08-31: the main loop died twice with the USB
// IRQ still alive (picotool could still reboot it), once after 2h21m and
// once after about a minute - the random interval that fits a bus glitch
// rather than a code path.
//
// 1 ms is roughly 6x what a 6-byte transfer needs at this bus speed, so it
// never trips in normal operation, while bounding a completely stuck bus
// to a few ms per loop instead of forever.
#define I2C_TIMEOUT_US 1000

// Sticky count of bounded-out transactions, reported in telemetry. This is
// what makes the fix verifiable: without it, "it did not hang" cannot be
// told apart from "the bus happened not to glitch yet".
uint16_t i2c_fault_count = 0;
static void i2c_note_fault(){ if(i2c_fault_count < 0xFFFF){ i2c_fault_count++; } }

static void mpu6050_init() {
    uint8_t power_reg[] = {0x6B, 0x00};
    // ACCEL_CONFIG: AFS_SEL bits [4:3] = 10 -> +/-8g (was 0x00 = +/-2g).
    // See the MPU6050_LSB_PER_g comment for the bench data that forced this.
    uint8_t accel_scale[] = {0x1C, 0x10};
    uint8_t gyro_scale[] = {0x1B, 0x18};
    // CONFIG / DLPF_CFG, lowered 0x03 (44 Hz) -> 0x04 (21 Hz) on 2026-09-05.
    // This filter sits on BOTH gyro and accel, in hardware, before the data
    // is ever read. The rate loop reads the raw gyro and its D-term
    // differentiates it (x200), so gyro vibration was being amplified into
    // violent motor commands: flight logs showed the motors thrashing +/-150
    // while the drone sat still with attitude at 0.0 deg, and that thrash
    // grew with throttle (more vibration) until it ran away above ~55%.
    // 44 Hz passed most of the vibration; 21 Hz cuts it hard while keeping
    // enough rate-loop bandwidth to hover. If it still diverges, the next
    // step is 0x05 (10 Hz) - and check the (correct-size) props are balanced,
    // since unbalanced props are the biggest single vibration source.
    uint8_t low_pass[] = {0x1A, 0x04};
    i2c_write_timeout_us(i2c0, addr, power_reg, 2, false, I2C_TIMEOUT_US);
    i2c_write_timeout_us(i2c0, addr, accel_scale, 2, false, I2C_TIMEOUT_US);
    i2c_write_timeout_us(i2c0, addr, gyro_scale, 2, false, I2C_TIMEOUT_US);
    i2c_write_timeout_us(i2c0, addr, low_pass, 2, false, I2C_TIMEOUT_US);
}

static void mpu6050_read_raw() {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    // On a timeout, leave accel[]/gyro[] holding their previous values
    // rather than storing a half-read buffer - the EKF copes far better
    // with one stale sample than with a fabricated one.
    if( i2c_write_timeout_us(i2c0, addr, &val, 1, true, I2C_TIMEOUT_US) == 1 &&
        i2c_read_timeout_us(i2c0, addr, buffer, 6, false, I2C_TIMEOUT_US) == 6 ){
        for (int i = 0; i < 3; i++) { accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]); }
    } else { i2c_note_fault(); }

    val = 0x43;
    if( i2c_write_timeout_us(i2c0, addr, &val, 1, true, I2C_TIMEOUT_US) == 1 &&
        i2c_read_timeout_us(i2c0, addr, buffer, 6, false, I2C_TIMEOUT_US) == 6 ){
        for (int i = 0; i < 3; i++) { gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]); }
    } else { i2c_note_fault(); }
}

static void qmc5883_init(){
    uint8_t qmc_reg1[] = {0x0B, 0x01};
    uint8_t qmc_reg2[] = {0x09, 0x0D};
    i2c_write_timeout_us(i2c0, addr1, qmc_reg1, 2, false, I2C_TIMEOUT_US);
    i2c_write_timeout_us(i2c0, addr1, qmc_reg2, 2, false, I2C_TIMEOUT_US);
}

static void qmc5883_read_raw(){
    uint8_t buffer[6];
    uint8_t val = 0x00;
    if( i2c_write_timeout_us(i2c0, addr1, &val, 1, true, I2C_TIMEOUT_US) == 1 &&
        i2c_read_timeout_us(i2c0, addr1, buffer, 6, false, I2C_TIMEOUT_US) == 6 ){
        for (int i = 0; i < 3; i++) { mag[i] = (buffer[(i * 2) + 1] << 8 | buffer[i * 2]); }
    } else { i2c_note_fault(); }
}

static bool bmp388_reg_write(uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val};
    if( i2c_write_timeout_us(i2c0, addr2, buf, 2, false, I2C_TIMEOUT_US) == 2 ){ return true; }
    i2c_note_fault();
    return false;
}

static bool bmp388_reg_read(uint8_t reg, uint8_t *dst, size_t len){
    if( i2c_write_timeout_us(i2c0, addr2, &reg, 1, true, I2C_TIMEOUT_US) != 1 ){ i2c_note_fault(); return false; }
    if( i2c_read_timeout_us(i2c0, addr2, dst, len, false, I2C_TIMEOUT_US) != (int)len ){ i2c_note_fault(); return false; }
    return true;
}

static void bmp388_init(){
    uint8_t chip_id = 0;
    if(!bmp388_reg_read(0x00, &chip_id, 1) || chip_id != 0x50){
        // Not present or not responding (wrong address strap, bad wiring,
        // not actually a BMP388) - leave isBaroPresent false rather than
        // spin forever waiting on a sensor that isn't there.
        isBaroPresent = false;
        return;
    }

    bmp388_reg_write(0x7E, 0xB6); // CMD: soft reset
    sleep_ms(10);

    uint8_t trim[21];
    bmp388_reg_read(0x31, trim, 21);
    uint16_t nvm_par_t1  = (uint16_t)(trim[0]  | (trim[1]  << 8));
    uint16_t nvm_par_t2  = (uint16_t)(trim[2]  | (trim[3]  << 8));
    int8_t   nvm_par_t3  = (int8_t)trim[4];
    int16_t  nvm_par_p1  = (int16_t)(trim[5]  | (trim[6]  << 8));
    int16_t  nvm_par_p2  = (int16_t)(trim[7]  | (trim[8]  << 8));
    int8_t   nvm_par_p3  = (int8_t)trim[9];
    int8_t   nvm_par_p4  = (int8_t)trim[10];
    uint16_t nvm_par_p5  = (uint16_t)(trim[11] | (trim[12] << 8));
    uint16_t nvm_par_p6  = (uint16_t)(trim[13] | (trim[14] << 8));
    int8_t   nvm_par_p7  = (int8_t)trim[15];
    int8_t   nvm_par_p8  = (int8_t)trim[16];
    int16_t  nvm_par_p9  = (int16_t)(trim[17] | (trim[18] << 8));
    int8_t   nvm_par_p10 = (int8_t)trim[19];
    int8_t   nvm_par_p11 = (int8_t)trim[20];

    // Quantization per BMP388 datasheet Table 14 ("/2^-8" == "*2^8" etc.)
    bmp388_cal.par_t1  = (float)nvm_par_t1  * 256.0f;
    bmp388_cal.par_t2  = (float)nvm_par_t2  / 1073741824.0f;       // 2^30
    bmp388_cal.par_t3  = (float)nvm_par_t3  / 281474976710656.0f;  // 2^48
    bmp388_cal.par_p1  = ((float)nvm_par_p1 - 16384.0f) / 1048576.0f; // 2^14, 2^20
    bmp388_cal.par_p2  = ((float)nvm_par_p2 - 16384.0f) / 536870912.0f; // 2^14, 2^29
    bmp388_cal.par_p3  = (float)nvm_par_p3  / 4294967296.0f;       // 2^32
    bmp388_cal.par_p4  = (float)nvm_par_p4  / 137438953472.0f;     // 2^37
    bmp388_cal.par_p5  = (float)nvm_par_p5  * 8.0f;                // 2^-3
    bmp388_cal.par_p6  = (float)nvm_par_p6  / 64.0f;               // 2^6
    bmp388_cal.par_p7  = (float)nvm_par_p7  / 256.0f;              // 2^8
    bmp388_cal.par_p8  = (float)nvm_par_p8  / 32768.0f;            // 2^15
    bmp388_cal.par_p9  = (float)nvm_par_p9  / 281474976710656.0f;  // 2^48
    bmp388_cal.par_p10 = (float)nvm_par_p10 / 281474976710656.0f;  // 2^48
    bmp388_cal.par_p11 = (float)nvm_par_p11 / 36893488147419103232.0f; // 2^65

    bmp388_reg_write(0x1C, 0x03); // OSR: pressure x8, temperature x1
    bmp388_reg_write(0x1D, 0x03); // ODR: 25Hz
    bmp388_reg_write(0x1F, 0x00); // CONFIG: IIR filter off
    bmp388_reg_write(0x1B, 0x33); // PWR_CTRL: press_en, temp_en, normal mode
    sleep_ms(10);

    isBaroPresent = true;
}

static void Baro_Read(){
    if(!isBaroPresent){ return; }

    uint8_t buffer[6];
    if(!bmp388_reg_read(0x04, buffer, 6)){ return; }

    uint32_t uncomp_press = (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16);
    uint32_t uncomp_temp  = (uint32_t)buffer[3] | ((uint32_t)buffer[4] << 8) | ((uint32_t)buffer[5] << 16);

    // Temperature compensation (Bosch BMP388 datasheet 9.3).
    float partial_data1 = (float)uncomp_temp - bmp388_cal.par_t1;
    float partial_data2 = partial_data1 * bmp388_cal.par_t2;
    float comp_temp = partial_data2 + (partial_data1 * partial_data1) * bmp388_cal.par_t3;

    // Pressure compensation, using the just-computed compensated temperature.
    float pd1 = bmp388_cal.par_p6 * comp_temp;
    float pd2 = bmp388_cal.par_p7 * comp_temp * comp_temp;
    float pd3 = bmp388_cal.par_p8 * comp_temp * comp_temp * comp_temp;
    float po1 = bmp388_cal.par_p5 + pd1 + pd2 + pd3;

    pd1 = bmp388_cal.par_p2 * comp_temp;
    pd2 = bmp388_cal.par_p3 * comp_temp * comp_temp;
    pd3 = bmp388_cal.par_p4 * comp_temp * comp_temp * comp_temp;
    float po2 = (float)uncomp_press * (bmp388_cal.par_p1 + pd1 + pd2 + pd3);

    pd1 = (float)uncomp_press * (float)uncomp_press;
    pd2 = bmp388_cal.par_p9 + bmp388_cal.par_p10 * comp_temp;
    float pd3b = pd1 * pd2;
    float pd4 = pd3b + (float)uncomp_press * (float)uncomp_press * (float)uncomp_press * bmp388_cal.par_p11;

    baro_temp_c = comp_temp;
    baro_pressure_pa = po1 + po2 + pd4;
}

static void IMU_Read(){
    mpu6050_read_raw();
    qmc5883_read_raw();

    for(int a = 0; a < 3; a++){
        int32_t mag = accel[a] < 0 ? -(int32_t)accel[a] : (int32_t)accel[a];
        if(mag > (int32_t)accel_peak[a]){ accel_peak[a] = (uint16_t)mag; }
        accel_sum[a] += accel[a];
    }
    accel_n++;

    ax = (float)accel[0] / MPU6050_LSB_PER_g;
    ay = (float)accel[1] / MPU6050_LSB_PER_g;
    az = (float)accel[2] / MPU6050_LSB_PER_g;
    ax = ax - accel_offset_x;
    ay = ay - accel_offset_y;
    az = az - accel_offset_z;
    float temp_x = ax * accel_cal[0][0] + ay * accel_cal[0][1] + az * accel_cal[0][2];
    float temp_y = ax * accel_cal[1][0] + ay * accel_cal[1][1] + az * accel_cal[1][2];
    float temp_z = ax * accel_cal[2][0] + ay * accel_cal[2][1] + az * accel_cal[2][2];
    ax = temp_x; ay = temp_y; az = temp_z;

    // Software low-pass on the accelerometer, added 2026-09-03.
    // Bench diagnosis: props off, drone bolted down, the accel MEAN (not
    // just peaks) swung by 20-30 deg. That is mid-frequency vibration
    // (~10-40 Hz structural content) passing straight through the chip's
    // 41 Hz hardware DLPF and shifting the apparent gravity direction.
    // The mount fix cut the PEAKS 2.6x but not this band, which is why the
    // attitude error did not improve.
    //
    // The accelerometer only has to supply a SLOW gravity reference - the
    // (now bias-calibrated) gyro carries all the fast rotation - so it can
    // be filtered hard without hurting control response. Can't use the
    // chip's DLPF: on the MPU6050 that same register also filters the gyro,
    // and slowing the gyro would wreck the rate loop. So filter accel only,
    // in software. EMA at alpha 0.10 on a 200 Hz loop is a ~3.3 Hz cutoff
    // (~90 ms time constant) - kills the vibration band, keeps gravity.
    static float ax_lpf = 0.0f, ay_lpf = 0.0f, az_lpf = 0.0f;
    static bool  accel_lpf_init = false;
    if(!accel_lpf_init){
        ax_lpf = ax; ay_lpf = ay; az_lpf = az;   // seed, no startup ramp
        accel_lpf_init = true;
    } else {
        ax_lpf += ACCEL_LPF_ALPHA * (ax - ax_lpf);
        ay_lpf += ACCEL_LPF_ALPHA * (ay - ay_lpf);
        az_lpf += ACCEL_LPF_ALPHA * (az - az_lpf);
    }
    ax = ax_lpf; ay = ay_lpf; az = az_lpf;

    float norm = sqrtf( ax*ax + ay*ay + az*az );
    accel_mag_g = norm;   // 1.0 = pure gravity; see ACCEL_R_GAIN
    ax = ax/norm; ay = ay/norm; az = az/norm;

    mx = ( (float)mag[0] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
    my = ( (float)mag[1] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
    mz = ( (float)mag[2] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
    mx = mx - compass_offset_x;
    my = my - compass_offset_y;
    mz = mz - compass_offset_z;
    temp_x = mx * mag_cal[0][0] + my * mag_cal[0][1] + mz * mag_cal[0][2];
    temp_y = mx * mag_cal[1][0] + my * mag_cal[1][1] + mz * mag_cal[1][2];
    temp_z = mx * mag_cal[2][0] + my * mag_cal[2][1] + mz * mag_cal[2][2];
    mx = temp_x; my = temp_y; mz = temp_z;
    norm = sqrtf( mx*mx + my*my + mz*mz );
    mag_mag_ut = norm;
    mx = mx/norm; my = my/norm; mz = mz/norm;

    wx = (gyro[0]-gyro_cal[0])*0.00106585038f;
    wy = (gyro[1]-gyro_cal[1])*0.00106585038f;
    wz = (gyro[2]-gyro_cal[2])*0.00106585038f;
}

static void EKF_Init(){
    sigma = 0.09f;
    R[0] = 0.25f;
    R[1] = 0.64f;
    P[0][0] = 1.0f;
    P[0][1] = 0;
    P[0][2] = 0;
    P[0][3] = 0;
    P[1][0] = 0;
    P[1][1] = 1.0f;
    P[1][2] = 0;
    P[1][3] = 0;
    P[2][0] = 0;
    P[2][1] = 0;
    P[2][2] = 1.0f;
    P[2][3] = 0;
    P[3][0] = 0;
    P[3][1] = 0;
    P[3][2] = 0;
    P[3][3] = 1.0f;  
    gx = 0;  gy = 0;  gz = 1; //ENU 
    ry = cosf(0.0174532925f*mag_inclination);  rz = -sinf(0.0174532925f*mag_inclination); //ENU

    // Gyro zero-rate bias, re-enabled 2026-09-02 (was commented out, so
    // gyro_cal[] stayed all zeros).
    //
    // This became mandatory when the PID rate term moved from the
    // differentiated quaternion to the raw gyro. The old path could not
    // drift: the EKF's accelerometer correction pinned it to gravity. The
    // raw gyro has no such anchor, so an uncalibrated zero-rate offset -
    // the MPU6050 is specified at up to +/-20 deg/s - reads as a constant
    // rotation that isn't happening. The rate loop then obediently spins
    // the drone the other way to null it, and it drifts off in one
    // direction. That is a very good match for "it runs off to the same
    // side every time".
    //
    // Rejected if the airframe moves during the sweep: a bias measured
    // while moving is worse than no bias at all, so fall back to zeros
    // (exactly the old behaviour) rather than bake in a bad number.
    {
        int32_t sum[3] = {0, 0, 0};
        int16_t lo[3], hi[3];
        for(int i = 0; i < GYRO_CAL_SAMPLES; i++){
            mpu6050_read_raw();
            for(int a = 0; a < 3; a++){
                sum[a] += gyro[a];
                if(i == 0){ lo[a] = hi[a] = gyro[a]; }
                else {
                    if(gyro[a] < lo[a]){ lo[a] = gyro[a]; }
                    if(gyro[a] > hi[a]){ hi[a] = gyro[a]; }
                }
            }
            sleep_ms(1);
        }

        is_gyro_calibrated = true;
        for(int a = 0; a < 3; a++){
            if( (hi[a] - lo[a]) > GYRO_CAL_MAX_SPREAD ){ is_gyro_calibrated = false; }
        }

        if(is_gyro_calibrated){
            gyro_cal[0] = sum[0]/GYRO_CAL_SAMPLES;
            gyro_cal[1] = sum[1]/GYRO_CAL_SAMPLES;
            gyro_cal[2] = sum[2]/GYRO_CAL_SAMPLES;
        } else {
            gyro_cal[0] = 0; gyro_cal[1] = 0; gyro_cal[2] = 0;
        }
    }

    qw=1; qx=0; qy=0; qz=0;
    sleep_ms(500);
}

static void EKF_Run(){
    // REVERTED 2026-09-03. Vibration-adaptive accel trust was tried here:
    //     R[0] = 0.25 + 60 * (|accel|-1g)^2
    // The idea was sound - a shaken MEMS accelerometer rectifies vibration
    // into a DC bias, so trust it less while it is shaking - but the gain
    // was far too high. Under real vibration R[0] stayed large almost
    // continuously, which effectively switched the accelerometer OFF, left
    // the gyro integrating unchecked, and the attitude ran away to 79 deg
    // (measured; it was 7.3 deg with fixed R). The accelerometer is the
    // only thing anchoring roll and pitch to gravity - it cannot be
    // de-weighted for long.
    // If revisited: cap it hard (say R[0] <= 1.0) so the anchor is only
    // ever softened, never removed.

//--------------------------------------- Predict --------------------------------------//
    float FP[4][4];
    float FPFT[4][4];
    float W[4][3];
    float F[4][4];
    float WWT[4][4];
    float Q[4][4];
    float v[6];
    float h[6];
    float H[6][4];
    float S_inv[6][6];
    float PHT[4][6];
    float K[4][6];
    float I[4][4];
    float I_KHP[4][4];
    float Q_out[4];

    W[0][0] = -qx;
    W[0][1] = -qy;
    W[0][2] = -qz;
    W[1][0] = qw;
    W[1][1] = -qz;
    W[1][2] = qy;
    W[2][0] = qz;
    W[2][1] = qw;
    W[2][2] = -qx;
    W[3][0] = -qy;
    W[3][1] = qx;
    W[3][2] = qw;

    float qw_cap = qw + (loop_time*0.5f)*(- wx*qx - wy*qy - wz*qz);
    float qx_cap = qx + (loop_time*0.5f)*(  wx*qw - wy*qz + wz*qy);
    float qy_cap = qy + (loop_time*0.5f)*(  wx*qz + wy*qw - wz*qx);
    float qz_cap = qz + (loop_time*0.5f)*(- wx*qy + wy*qx + wz*qw);
    float norm_q = sqrt(qw_cap*qw_cap + qx_cap*qx_cap + qy_cap*qy_cap + qz_cap*qz_cap);
    qw = qw_cap/norm_q;
    qx = qx_cap/norm_q;
    qy = qy_cap/norm_q;
    qz = qz_cap/norm_q;
    
    float half_loop = loop_time*0.5f;
    F[0][0] = 1;
    F[0][1] = -wx*half_loop;
    F[0][2] = -wy*half_loop;
    F[0][3] = -wz*half_loop;
    F[1][0] = wx*half_loop;
    F[1][1] = 1;
    F[1][2] = wz*half_loop;
    F[1][3] = -wy*half_loop;
    F[2][0] = wy*half_loop;
    F[2][1] = -wz*half_loop;
    F[2][2] = 1;
    F[2][3] = wx*half_loop;
    F[3][0] = wz*half_loop;
    F[3][1] = wy*half_loop;
    F[3][2] = -wx*half_loop;
    F[3][3] = 1;

    FP[0][0] = (P[0][0] + F[0][1]*P[1][0] + F[0][2]*P[2][0] + F[0][3]*P[3][0]);
    FP[0][1] = (P[0][1] + F[0][1]*P[1][1] + F[0][2]*P[2][1] + F[0][3]*P[3][1]);
    FP[0][2] = (P[0][2] + F[0][1]*P[1][2] + F[0][2]*P[2][2] + F[0][3]*P[3][2]);
    FP[0][3] = (P[0][3] + F[0][1]*P[1][3] + F[0][2]*P[2][3] + F[0][3]*P[3][3]);
    FP[1][0] = (F[1][0]*P[0][0] + P[1][0] + F[1][2]*P[2][0] + F[1][3]*P[3][0]);
    FP[1][1] = (F[1][0]*P[0][1] + P[1][1] + F[1][2]*P[2][1] + F[1][3]*P[3][1]);
    FP[1][2] = (F[1][0]*P[0][2] + P[1][2] + F[1][2]*P[2][2] + F[1][3]*P[3][2]);
    FP[1][3] = (F[1][0]*P[0][3] + P[1][3] + F[1][2]*P[2][3] + F[1][3]*P[3][3]);
    FP[2][0] = (F[2][0]*P[0][0] + F[2][1]*P[1][0] + P[2][0] + F[2][3]*P[3][0]);
    FP[2][1] = (F[2][0]*P[0][1] + F[2][1]*P[1][1] + P[2][1] + F[2][3]*P[3][1]);
    FP[2][2] = (F[2][0]*P[0][2] + F[2][1]*P[1][2] + P[2][2] + F[2][3]*P[3][2]);
    FP[2][3] = (F[2][0]*P[0][3] + F[2][1]*P[1][3] + P[2][3] + F[2][3]*P[3][3]);
    FP[3][0] = (F[3][0]*P[0][0] + F[3][1]*P[1][0] + F[3][2]*P[2][0] + P[3][0]);
    FP[3][1] = (F[3][0]*P[0][1] + F[3][1]*P[1][1] + F[3][2]*P[2][1] + P[3][1]);
    FP[3][2] = (F[3][0]*P[0][2] + F[3][1]*P[1][2] + F[3][2]*P[2][2] + P[3][2]);
    FP[3][3] = (F[3][0]*P[0][3] + F[3][1]*P[1][3] + F[3][2]*P[2][3] + P[3][3]);

    FPFT[0][0] = (FP[0][0]+FP[0][1]*F[0][1]+FP[0][2]*F[0][2]+FP[0][3]*F[0][3]);
    FPFT[0][1] = (FP[0][0]*F[1][0]+FP[0][1]+FP[0][2]*F[1][2]+FP[0][3]*F[1][3]);
    FPFT[0][2] = (FP[0][0]*F[2][0]+FP[0][1]*F[2][1]+FP[0][2]+FP[0][3]*F[2][3]);
    FPFT[0][3] = (FP[0][0]*F[3][0]+FP[0][1]*F[3][1]+FP[0][2]*F[3][2]+FP[0][3]);
    FPFT[1][0] = (FP[1][0]+FP[1][1]*F[0][1]+FP[1][2]*F[0][2]+FP[1][3]*F[0][3]);
    FPFT[1][1] = (FP[1][0]*F[1][0]+FP[1][1]+FP[1][2]*F[1][2]+FP[1][3]*F[1][3]);
    FPFT[1][2] = (FP[1][0]*F[2][0]+FP[1][1]*F[2][1]+FP[1][2]+FP[1][3]*F[2][3]);
    FPFT[1][3] = (FP[1][0]*F[3][0]+FP[1][1]*F[3][1]+FP[1][2]*F[3][2]+FP[1][3]);
    FPFT[2][0] = (FP[2][0]+FP[2][1]*F[0][1]+FP[2][2]*F[0][2]+FP[2][3]*F[0][3]);
    FPFT[2][1] = (FP[2][0]*F[1][0]+FP[2][1]+FP[2][2]*F[1][2]+FP[2][3]*F[1][3]);
    FPFT[2][2] = (FP[2][0]*F[2][0]+FP[2][1]*F[2][1]+FP[2][2]+FP[2][3]*F[2][3]);
    FPFT[2][3] = (FP[2][0]*F[3][0]+FP[2][1]*F[3][1]+FP[2][2]*F[3][2]+FP[2][3]);
    FPFT[3][0] = (FP[3][0]+FP[3][1]*F[0][1]+FP[3][2]*F[0][2]+FP[3][3]*F[0][3]);
    FPFT[3][1] = (FP[3][0]*F[1][0]+FP[3][1]+FP[3][2]*F[1][2]+FP[3][3]*F[1][3]);
    FPFT[3][2] = (FP[3][0]*F[2][0]+FP[3][1]*F[2][1]+FP[3][2]+FP[3][3]*F[2][3]);
    FPFT[3][3] = (FP[3][0]*F[3][0]+FP[3][1]*F[3][1]+FP[3][2]*F[3][2]+FP[3][3]);

    float sigmaloop2b4 = sigma*loop_time*loop_time*0.25f;
    Q[0][0] = (W[0][0]*W[0][0]+W[0][1]*W[0][1]+W[0][2]*W[0][2])*sigmaloop2b4;
    Q[0][1] = (W[0][0]*W[1][0]+W[0][1]*W[1][1]+W[0][2]*W[1][2])*sigmaloop2b4;
    Q[0][2] = (W[0][0]*W[2][0]+W[0][1]*W[2][1]+W[0][2]*W[2][2])*sigmaloop2b4;
    Q[0][3] = (W[0][0]*W[3][0]+W[0][1]*W[3][1]+W[0][2]*W[3][2])*sigmaloop2b4;
    Q[1][0] = (W[1][0]*W[0][0]+W[1][1]*W[0][1]+W[1][2]*W[0][2])*sigmaloop2b4;
    Q[1][1] = (W[1][0]*W[1][0]+W[1][1]*W[1][1]+W[1][2]*W[1][2])*sigmaloop2b4;
    Q[1][2] = (W[1][0]*W[2][0]+W[1][1]*W[2][1]+W[1][2]*W[2][2])*sigmaloop2b4;
    Q[1][3] = (W[1][0]*W[3][0]+W[1][1]*W[3][1]+W[1][2]*W[3][2])*sigmaloop2b4;
    Q[2][0] = (W[2][0]*W[0][0]+W[2][1]*W[0][1]+W[2][2]*W[0][2])*sigmaloop2b4;
    Q[2][1] = (W[2][0]*W[1][0]+W[2][1]*W[1][1]+W[2][2]*W[1][2])*sigmaloop2b4;
    Q[2][2] = (W[2][0]*W[2][0]+W[2][1]*W[2][1]+W[2][2]*W[2][2])*sigmaloop2b4;
    Q[2][3] = (W[2][0]*W[3][0]+W[2][1]*W[3][1]+W[2][2]*W[3][2])*sigmaloop2b4;
    Q[3][0] = (W[3][0]*W[0][0]+W[3][1]*W[0][1]+W[3][2]*W[0][2])*sigmaloop2b4;
    Q[3][1] = (W[3][0]*W[1][0]+W[3][1]*W[1][1]+W[3][2]*W[1][2])*sigmaloop2b4;
    Q[3][2] = (W[3][0]*W[2][0]+W[3][1]*W[2][1]+W[3][2]*W[2][2])*sigmaloop2b4;
    Q[3][3] = (W[3][0]*W[3][0]+W[3][1]*W[3][1]+W[3][2]*W[3][2])*sigmaloop2b4;

    P[0][0] = FPFT[0][0] + Q[0][0];
    P[0][1] = FPFT[0][1] + Q[0][1];
    P[0][2] = FPFT[0][2] + Q[0][2];
    P[0][3] = FPFT[0][3] + Q[0][3];
    P[1][0] = FPFT[1][0] + Q[1][0];
    P[1][1] = FPFT[1][1] + Q[1][1];
    P[1][2] = FPFT[1][2] + Q[1][2];
    P[1][3] = FPFT[1][3] + Q[1][3];
    P[2][0] = FPFT[2][0] + Q[2][0];
    P[2][1] = FPFT[2][1] + Q[2][1];
    P[2][2] = FPFT[2][2] + Q[2][2];
    P[2][3] = FPFT[2][3] + Q[2][3];
    P[3][0] = FPFT[3][0] + Q[3][0];
    P[3][1] = FPFT[3][1] + Q[3][1];
    P[3][2] = FPFT[3][2] + Q[3][2];
    P[3][3] = FPFT[3][3] + Q[3][3];
//--------------------------------------- Update --------------------------------------//
    h[0] = 2.0f*(qx*qz-qw*qy);
    h[1] = 2.0f*(qw*qx+qy*qz);
    h[2] = 2.0f*(0.5f-qx*qx-qy*qy);
    h[3] = 2.0f*(ry*(qw*qz+qx*qy)+rz*(qx*qz-qw*qy));
    h[4] = 2.0f*(ry*(0.5f-qx*qx-qz*qz)+rz*(qw*qx+qy*qz));
    h[5] = 2.0f*(ry*(qy*qz-qw*qx)+rz*(0.5f-qx*qx-qy*qy));
    v[0] = ax - h[0];
    v[1] = ay - h[1];
    v[2] = az - h[2];
    v[3] = mx - h[3];
    v[4] = my - h[4];
    v[5] = mz - h[5];
    if(my<0 && h[2]>0){ v[3] = -v[3]; }             // *compass filter singulartiy, avoids divergence of filter while pointing South [V.V.I.]*
    
    H[0][0] = -2.0f*qy;
    H[0][1] = 2.0f*qz;
    H[0][2] = -2.0f*qw;
    H[0][3] = 2.0f*qx;
    H[1][0] = 2.0f*qx;
    H[1][1] = 2.0f*qw;
    H[1][2] = 2.0f*qz;
    H[1][3] = 2.0f*qy;
    H[2][0] = 0;
    H[2][1] = -4.0f*qx;
    H[2][2] = -4.0f*qy;
    H[2][3] = 0;
    H[3][0] = -2.0f*(ry*qz-rz*qy);
    H[3][1] = 2.0f*(ry*qy+rz*qz);
    H[3][2] = 2.0f*(ry*qx-rz*qw);
    H[3][3] = 2.0f*(ry*qw+rz*qx);
    H[4][0] = 2.0f*rz*qx;
    H[4][1] = 2.0f*(-2.0f*ry*qx+rz*qw);
    H[4][2] = 2.0f*rz*qz;
    H[4][3] = 2.0f*(-2.0f*ry*qz+rz*qy);
    H[5][0] = -2.0f*ry*qx;
    H[5][1] = 2.0f*(-ry*qw-2.0f*rz*qx);
    H[5][2] = 2.0f*(ry*qz-2.0f*rz*qy);
    H[5][3] = 2.0f*ry*qy;

    PHT[0][0] = (P[0][0]*H[0][0]+P[0][1]*H[0][1]+P[0][2]*H[0][2]+P[0][3]*H[0][3]);
    PHT[0][1] = (P[0][0]*H[1][0]+P[0][1]*H[1][1]+P[0][2]*H[1][2]+P[0][3]*H[1][3]);
    PHT[0][2] = (P[0][0]*H[2][0]+P[0][1]*H[2][1]+P[0][2]*H[2][2]+P[0][3]*H[2][3]);
    PHT[0][3] = (P[0][0]*H[3][0]+P[0][1]*H[3][1]+P[0][2]*H[3][2]+P[0][3]*H[3][3]);
    PHT[0][4] = (P[0][0]*H[4][0]+P[0][1]*H[4][1]+P[0][2]*H[4][2]+P[0][3]*H[4][3]);
    PHT[0][5] = (P[0][0]*H[5][0]+P[0][1]*H[5][1]+P[0][2]*H[5][2]+P[0][3]*H[5][3]);
    PHT[1][0] = (P[1][0]*H[0][0]+P[1][1]*H[0][1]+P[1][2]*H[0][2]+P[1][3]*H[0][3]);
    PHT[1][1] = (P[1][0]*H[1][0]+P[1][1]*H[1][1]+P[1][2]*H[1][2]+P[1][3]*H[1][3]);
    PHT[1][2] = (P[1][0]*H[2][0]+P[1][1]*H[2][1]+P[1][2]*H[2][2]+P[1][3]*H[2][3]);
    PHT[1][3] = (P[1][0]*H[3][0]+P[1][1]*H[3][1]+P[1][2]*H[3][2]+P[1][3]*H[3][3]);
    PHT[1][4] = (P[1][0]*H[4][0]+P[1][1]*H[4][1]+P[1][2]*H[4][2]+P[1][3]*H[4][3]);
    PHT[1][5] = (P[1][0]*H[5][0]+P[1][1]*H[5][1]+P[1][2]*H[5][2]+P[1][3]*H[5][3]);
    PHT[2][0] = (P[2][0]*H[0][0]+P[2][1]*H[0][1]+P[2][2]*H[0][2]+P[2][3]*H[0][3]);
    PHT[2][1] = (P[2][0]*H[1][0]+P[2][1]*H[1][1]+P[2][2]*H[1][2]+P[2][3]*H[1][3]);
    PHT[2][2] = (P[2][0]*H[2][0]+P[2][1]*H[2][1]+P[2][2]*H[2][2]+P[2][3]*H[2][3]);
    PHT[2][3] = (P[2][0]*H[3][0]+P[2][1]*H[3][1]+P[2][2]*H[3][2]+P[2][3]*H[3][3]);
    PHT[2][4] = (P[2][0]*H[4][0]+P[2][1]*H[4][1]+P[2][2]*H[4][2]+P[2][3]*H[4][3]);
    PHT[2][5] = (P[2][0]*H[5][0]+P[2][1]*H[5][1]+P[2][2]*H[5][2]+P[2][3]*H[5][3]);
    PHT[3][0] = (P[3][0]*H[0][0]+P[3][1]*H[0][1]+P[3][2]*H[0][2]+P[3][3]*H[0][3]);
    PHT[3][1] = (P[3][0]*H[1][0]+P[3][1]*H[1][1]+P[3][2]*H[1][2]+P[3][3]*H[1][3]);
    PHT[3][2] = (P[3][0]*H[2][0]+P[3][1]*H[2][1]+P[3][2]*H[2][2]+P[3][3]*H[2][3]);
    PHT[3][3] = (P[3][0]*H[3][0]+P[3][1]*H[3][1]+P[3][2]*H[3][2]+P[3][3]*H[3][3]);
    PHT[3][4] = (P[3][0]*H[4][0]+P[3][1]*H[4][1]+P[3][2]*H[4][2]+P[3][3]*H[4][3]);
    PHT[3][5] = (P[3][0]*H[5][0]+P[3][1]*H[5][1]+P[3][2]*H[5][2]+P[3][3]*H[5][3]);

    uint8_t i, j, k, n;
    float a[13][13] = {0},d;
    n = 6;
    a[1][1] = (H[0][0]*PHT[0][0]+H[0][1]*PHT[1][0]+H[0][2]*PHT[2][0]+H[0][3]*PHT[3][0]) + R[0];
    a[1][2] = (H[0][0]*PHT[0][1]+H[0][1]*PHT[1][1]+H[0][2]*PHT[2][1]+H[0][3]*PHT[3][1]);  
    a[1][3] = (H[0][0]*PHT[0][2]+H[0][1]*PHT[1][2]+H[0][2]*PHT[2][2]+H[0][3]*PHT[3][2]);  
    a[1][4] = (H[0][0]*PHT[0][3]+H[0][1]*PHT[1][3]+H[0][2]*PHT[2][3]+H[0][3]*PHT[3][3]);
    a[1][5] = (H[0][0]*PHT[0][4]+H[0][1]*PHT[1][4]+H[0][2]*PHT[2][4]+H[0][3]*PHT[3][4]);  
    a[1][6] = (H[0][0]*PHT[0][5]+H[0][1]*PHT[1][5]+H[0][2]*PHT[2][5]+H[0][3]*PHT[3][5]);
    
    a[2][1] = (H[1][0]*PHT[0][0]+H[1][1]*PHT[1][0]+H[1][2]*PHT[2][0]+H[1][3]*PHT[3][0]);
    a[2][2] = (H[1][0]*PHT[0][1]+H[1][1]*PHT[1][1]+H[1][2]*PHT[2][1]+H[1][3]*PHT[3][1]) + R[0];  
    a[2][3] = (H[1][0]*PHT[0][2]+H[1][1]*PHT[1][2]+H[1][2]*PHT[2][2]+H[1][3]*PHT[3][2]);  
    a[2][4] = (H[1][0]*PHT[0][3]+H[1][1]*PHT[1][3]+H[1][2]*PHT[2][3]+H[1][3]*PHT[3][3]);
    a[2][5] = (H[1][0]*PHT[0][4]+H[1][1]*PHT[1][4]+H[1][2]*PHT[2][4]+H[1][3]*PHT[3][4]);  
    a[2][6] = (H[1][0]*PHT[0][5]+H[1][1]*PHT[1][5]+H[1][2]*PHT[2][5]+H[1][3]*PHT[3][5]);  
    
    a[3][1] = (H[2][0]*PHT[0][0]+H[2][1]*PHT[1][0]+H[2][2]*PHT[2][0]+H[2][3]*PHT[3][0]);
    a[3][2] = (H[2][0]*PHT[0][1]+H[2][1]*PHT[1][1]+H[2][2]*PHT[2][1]+H[2][3]*PHT[3][1]);  
    a[3][3] = (H[2][0]*PHT[0][2]+H[2][1]*PHT[1][2]+H[2][2]*PHT[2][2]+H[2][3]*PHT[3][2]) + R[0];  
    a[3][4] = (H[2][0]*PHT[0][3]+H[2][1]*PHT[1][3]+H[2][2]*PHT[2][3]+H[2][3]*PHT[3][3]);
    a[3][5] = (H[2][0]*PHT[0][4]+H[2][1]*PHT[1][4]+H[2][2]*PHT[2][4]+H[2][3]*PHT[3][4]);  
    a[3][6] = (H[2][0]*PHT[0][5]+H[2][1]*PHT[1][5]+H[2][2]*PHT[2][5]+H[2][3]*PHT[3][5]);
    
    a[4][1] = (H[3][0]*PHT[0][0]+H[3][1]*PHT[1][0]+H[3][2]*PHT[2][0]+H[3][3]*PHT[3][0]);
    a[4][2] = (H[3][0]*PHT[0][1]+H[3][1]*PHT[1][1]+H[3][2]*PHT[2][1]+H[3][3]*PHT[3][1]);
    a[4][3] = (H[3][0]*PHT[0][2]+H[3][1]*PHT[1][2]+H[3][2]*PHT[2][2]+H[3][3]*PHT[3][2]);
    a[4][4] = (H[3][0]*PHT[0][3]+H[3][1]*PHT[1][3]+H[3][2]*PHT[2][3]+H[3][3]*PHT[3][3]) + R[1];
    a[4][5] = (H[3][0]*PHT[0][4]+H[3][1]*PHT[1][4]+H[3][2]*PHT[2][4]+H[3][3]*PHT[3][4]);
    a[4][6] = (H[3][0]*PHT[0][5]+H[3][1]*PHT[1][5]+H[3][2]*PHT[2][5]+H[3][3]*PHT[3][5]);
    
    a[5][1] = (H[4][0]*PHT[0][0]+H[4][1]*PHT[1][0]+H[4][2]*PHT[2][0]+H[4][3]*PHT[3][0]);
    a[5][2] = (H[4][0]*PHT[0][1]+H[4][1]*PHT[1][1]+H[4][2]*PHT[2][1]+H[4][3]*PHT[3][1]);
    a[5][3] = (H[4][0]*PHT[0][2]+H[4][1]*PHT[1][2]+H[4][2]*PHT[2][2]+H[4][3]*PHT[3][2]);
    a[5][4] = (H[4][0]*PHT[0][3]+H[4][1]*PHT[1][3]+H[4][2]*PHT[2][3]+H[4][3]*PHT[3][3]);
    a[5][5] = (H[4][0]*PHT[0][4]+H[4][1]*PHT[1][4]+H[4][2]*PHT[2][4]+H[4][3]*PHT[3][4]) + R[1];  
    a[5][6] = (H[4][0]*PHT[0][5]+H[4][1]*PHT[1][5]+H[4][2]*PHT[2][5]+H[4][3]*PHT[3][5]);
    
    a[6][1] = (H[5][0]*PHT[0][0]+H[5][1]*PHT[1][0]+H[5][2]*PHT[2][0]+H[5][3]*PHT[3][0]);
    a[6][2] = (H[5][0]*PHT[0][1]+H[5][1]*PHT[1][1]+H[5][2]*PHT[2][1]+H[5][3]*PHT[3][1]);
    a[6][3] = (H[5][0]*PHT[0][2]+H[5][1]*PHT[1][2]+H[5][2]*PHT[2][2]+H[5][3]*PHT[3][2]);
    a[6][4] = (H[5][0]*PHT[0][3]+H[5][1]*PHT[1][3]+H[5][2]*PHT[2][3]+H[5][3]*PHT[3][3]);
    a[6][5] = (H[5][0]*PHT[0][4]+H[5][1]*PHT[1][4]+H[5][2]*PHT[2][4]+H[5][3]*PHT[3][4]);
    a[6][6] = (H[5][0]*PHT[0][5]+H[5][1]*PHT[1][5]+H[5][2]*PHT[2][5]+H[5][3]*PHT[3][5]) + R[1];   
    for (i = 1; i <= n; i++){
        for (j = 1; j <= 2 * n; j++){
            if (j == (i + n)){
                a[i][j] = 1;
            }
        }
    }
    for (i = n; i > 1; i--){
        if (a[i-1][1] < a[i][1]){
            for(j = 1; j <= n * 2; j++){
                d = a[i][j];
                a[i][j] = a[i-1][j];
                a[i-1][j] = d;
            }
        }
    }
    for (i = 1; i <= n; i++){
        for (j = 1; j <= n * 2; j++){
            if (j != i){
                d = a[j][i] / a[i][i];
                for (k = 1; k <= n * 2; k++){
                    a[j][k] = a[j][k] - (a[i][k] * d);
                }
            }
        }
    }
    for (i = 1; i <= n; i++){
        d=a[i][i];
        for (j = 1; j <= n * 2; j++){
            a[i][j] = a[i][j] / d;
        }
    }
    for (i = 1; i <= n; i++){
        for (j = n + 1; j <= n * 2; j++){
            S_inv[i-1][j-7] = a[i][j];
        }
    }

    K[0][0] = (PHT[0][0]*S_inv[0][0]+PHT[0][1]*S_inv[1][0]+PHT[0][2]*S_inv[2][0]+PHT[0][3]*S_inv[3][0]+PHT[0][4]*S_inv[4][0]+PHT[0][5]*S_inv[5][0]);
    K[0][1] = (PHT[0][0]*S_inv[0][1]+PHT[0][1]*S_inv[1][1]+PHT[0][2]*S_inv[2][1]+PHT[0][3]*S_inv[3][1]+PHT[0][4]*S_inv[4][1]+PHT[0][5]*S_inv[5][1]);
    K[0][2] = (PHT[0][0]*S_inv[0][2]+PHT[0][1]*S_inv[1][2]+PHT[0][2]*S_inv[2][2]+PHT[0][3]*S_inv[3][2]+PHT[0][4]*S_inv[4][2]+PHT[0][5]*S_inv[5][2]);
    K[0][3] = (PHT[0][0]*S_inv[0][3]+PHT[0][1]*S_inv[1][3]+PHT[0][2]*S_inv[2][3]+PHT[0][3]*S_inv[3][3]+PHT[0][4]*S_inv[4][3]+PHT[0][5]*S_inv[5][3]);
    K[0][4] = (PHT[0][0]*S_inv[0][4]+PHT[0][1]*S_inv[1][4]+PHT[0][2]*S_inv[2][4]+PHT[0][3]*S_inv[3][4]+PHT[0][4]*S_inv[4][4]+PHT[0][5]*S_inv[5][4]);
    K[0][5] = (PHT[0][0]*S_inv[0][5]+PHT[0][1]*S_inv[1][5]+PHT[0][2]*S_inv[2][5]+PHT[0][3]*S_inv[3][5]+PHT[0][4]*S_inv[4][5]+PHT[0][5]*S_inv[5][5]);
    K[1][0] = (PHT[1][0]*S_inv[0][0]+PHT[1][1]*S_inv[1][0]+PHT[1][2]*S_inv[2][0]+PHT[1][3]*S_inv[3][0]+PHT[1][4]*S_inv[4][0]+PHT[1][5]*S_inv[5][0]);
    K[1][1] = (PHT[1][0]*S_inv[0][1]+PHT[1][1]*S_inv[1][1]+PHT[1][2]*S_inv[2][1]+PHT[1][3]*S_inv[3][1]+PHT[1][4]*S_inv[4][1]+PHT[1][5]*S_inv[5][1]);
    K[1][2] = (PHT[1][0]*S_inv[0][2]+PHT[1][1]*S_inv[1][2]+PHT[1][2]*S_inv[2][2]+PHT[1][3]*S_inv[3][2]+PHT[1][4]*S_inv[4][2]+PHT[1][5]*S_inv[5][2]);
    K[1][3] = (PHT[1][0]*S_inv[0][3]+PHT[1][1]*S_inv[1][3]+PHT[1][2]*S_inv[2][3]+PHT[1][3]*S_inv[3][3]+PHT[1][4]*S_inv[4][3]+PHT[1][5]*S_inv[5][3]);
    K[1][4] = (PHT[1][0]*S_inv[0][4]+PHT[1][1]*S_inv[1][4]+PHT[1][2]*S_inv[2][4]+PHT[1][3]*S_inv[3][4]+PHT[1][4]*S_inv[4][4]+PHT[1][5]*S_inv[5][4]);
    K[1][5] = (PHT[1][0]*S_inv[0][5]+PHT[1][1]*S_inv[1][5]+PHT[1][2]*S_inv[2][5]+PHT[1][3]*S_inv[3][5]+PHT[1][4]*S_inv[4][5]+PHT[1][5]*S_inv[5][5]);
    K[2][0] = (PHT[2][0]*S_inv[0][0]+PHT[2][1]*S_inv[1][0]+PHT[2][2]*S_inv[2][0]+PHT[2][3]*S_inv[3][0]+PHT[2][4]*S_inv[4][0]+PHT[2][5]*S_inv[5][0]);
    K[2][1] = (PHT[2][0]*S_inv[0][1]+PHT[2][1]*S_inv[1][1]+PHT[2][2]*S_inv[2][1]+PHT[2][3]*S_inv[3][1]+PHT[2][4]*S_inv[4][1]+PHT[2][5]*S_inv[5][1]);
    K[2][2] = (PHT[2][0]*S_inv[0][2]+PHT[2][1]*S_inv[1][2]+PHT[2][2]*S_inv[2][2]+PHT[2][3]*S_inv[3][2]+PHT[2][4]*S_inv[4][2]+PHT[2][5]*S_inv[5][2]);
    K[2][3] = (PHT[2][0]*S_inv[0][3]+PHT[2][1]*S_inv[1][3]+PHT[2][2]*S_inv[2][3]+PHT[2][3]*S_inv[3][3]+PHT[2][4]*S_inv[4][3]+PHT[2][5]*S_inv[5][3]);
    K[2][4] = (PHT[2][0]*S_inv[0][4]+PHT[2][1]*S_inv[1][4]+PHT[2][2]*S_inv[2][4]+PHT[2][3]*S_inv[3][4]+PHT[2][4]*S_inv[4][4]+PHT[2][5]*S_inv[5][4]);
    K[2][5] = (PHT[2][0]*S_inv[0][5]+PHT[2][1]*S_inv[1][5]+PHT[2][2]*S_inv[2][5]+PHT[2][3]*S_inv[3][5]+PHT[2][4]*S_inv[4][5]+PHT[2][5]*S_inv[5][5]);
    K[3][0] = (PHT[3][0]*S_inv[0][0]+PHT[3][1]*S_inv[1][0]+PHT[3][2]*S_inv[2][0]+PHT[3][3]*S_inv[3][0]+PHT[3][4]*S_inv[4][0]+PHT[3][5]*S_inv[5][0]);
    K[3][1] = (PHT[3][0]*S_inv[0][1]+PHT[3][1]*S_inv[1][1]+PHT[3][2]*S_inv[2][1]+PHT[3][3]*S_inv[3][1]+PHT[3][4]*S_inv[4][1]+PHT[3][5]*S_inv[5][1]);
    K[3][2] = (PHT[3][0]*S_inv[0][2]+PHT[3][1]*S_inv[1][2]+PHT[3][2]*S_inv[2][2]+PHT[3][3]*S_inv[3][2]+PHT[3][4]*S_inv[4][2]+PHT[3][5]*S_inv[5][2]);
    K[3][3] = (PHT[3][0]*S_inv[0][3]+PHT[3][1]*S_inv[1][3]+PHT[3][2]*S_inv[2][3]+PHT[3][3]*S_inv[3][3]+PHT[3][4]*S_inv[4][3]+PHT[3][5]*S_inv[5][3]);
    K[3][4] = (PHT[3][0]*S_inv[0][4]+PHT[3][1]*S_inv[1][4]+PHT[3][2]*S_inv[2][4]+PHT[3][3]*S_inv[3][4]+PHT[3][4]*S_inv[4][4]+PHT[3][5]*S_inv[5][4]);
    K[3][5] = (PHT[3][0]*S_inv[0][5]+PHT[3][1]*S_inv[1][5]+PHT[3][2]*S_inv[2][5]+PHT[3][3]*S_inv[3][5]+PHT[3][4]*S_inv[4][5]+PHT[3][5]*S_inv[5][5]);

    Q_out[0] = (K[0][0]*v[0]+K[0][1]*v[1]+K[0][2]*v[2]+K[0][3]*v[3]+K[0][4]*v[4]+K[0][5]*v[5]);
    Q_out[1] = (K[1][0]*v[0]+K[1][1]*v[1]+K[1][2]*v[2]+K[1][3]*v[3]+K[1][4]*v[4]+K[1][5]*v[5]);
    Q_out[2] = (K[2][0]*v[0]+K[2][1]*v[1]+K[2][2]*v[2]+K[2][3]*v[3]+K[2][4]*v[4]+K[2][5]*v[5]);
    Q_out[3] = (K[3][0]*v[0]+K[3][1]*v[1]+K[3][2]*v[2]+K[3][3]*v[3]+K[3][4]*v[4]+K[3][5]*v[5]);

    I[0][0] = 1-(K[0][0]*H[0][0]+K[0][1]*H[1][0]+K[0][2]*H[2][0]+K[0][3]*H[3][0]+K[0][4]*H[4][0]+K[0][5]*H[5][0]);
    I[0][1] = -(K[0][0]*H[0][1]+K[0][1]*H[1][1]+K[0][2]*H[2][1]+K[0][3]*H[3][1]+K[0][4]*H[4][1]+K[0][5]*H[5][1]);
    I[0][2] = -(K[0][0]*H[0][2]+K[0][1]*H[1][2]+K[0][2]*H[2][2]+K[0][3]*H[3][2]+K[0][4]*H[4][2]+K[0][5]*H[5][2]);
    I[0][3] = -(K[0][0]*H[0][3]+K[0][1]*H[1][3]+K[0][2]*H[2][3]+K[0][3]*H[3][3]+K[0][4]*H[4][3]+K[0][5]*H[5][3]);
    I[1][0] = -(K[1][0]*H[0][0]+K[1][1]*H[1][0]+K[1][2]*H[2][0]+K[1][3]*H[3][0]+K[1][4]*H[4][0]+K[1][5]*H[5][0]);
    I[1][1] = 1-(K[1][0]*H[0][1]+K[1][1]*H[1][1]+K[1][2]*H[2][1]+K[1][3]*H[3][1]+K[1][4]*H[4][1]+K[1][5]*H[5][1]);
    I[1][2] = -(K[1][0]*H[0][2]+K[1][1]*H[1][2]+K[1][2]*H[2][2]+K[1][3]*H[3][2]+K[1][4]*H[4][2]+K[1][5]*H[5][2]);
    I[1][3] = -(K[1][0]*H[0][3]+K[1][1]*H[1][3]+K[1][2]*H[2][3]+K[1][3]*H[3][3]+K[1][4]*H[4][3]+K[1][5]*H[5][3]);
    I[2][0] = -(K[2][0]*H[0][0]+K[2][1]*H[1][0]+K[2][2]*H[2][0]+K[2][3]*H[3][0]+K[2][4]*H[4][0]+K[2][5]*H[5][0]);
    I[2][1] = -(K[2][0]*H[0][1]+K[2][1]*H[1][1]+K[2][2]*H[2][1]+K[2][3]*H[3][1]+K[2][4]*H[4][1]+K[2][5]*H[5][1]);
    I[2][2] = 1-(K[2][0]*H[0][2]+K[2][1]*H[1][2]+K[2][2]*H[2][2]+K[2][3]*H[3][2]+K[2][4]*H[4][2]+K[2][5]*H[5][2]);
    I[2][3] = -(K[2][0]*H[0][3]+K[2][1]*H[1][3]+K[2][2]*H[2][3]+K[2][3]*H[3][3]+K[2][4]*H[4][3]+K[2][5]*H[5][3]);
    I[3][0] = -(K[3][0]*H[0][0]+K[3][1]*H[1][0]+K[3][2]*H[2][0]+K[3][3]*H[3][0]+K[3][4]*H[4][0]+K[3][5]*H[5][0]);
    I[3][1] = -(K[3][0]*H[0][1]+K[3][1]*H[1][1]+K[3][2]*H[2][1]+K[3][3]*H[3][1]+K[3][4]*H[4][1]+K[3][5]*H[5][1]);
    I[3][2] = -(K[3][0]*H[0][2]+K[3][1]*H[1][2]+K[3][2]*H[2][2]+K[3][3]*H[3][2]+K[3][4]*H[4][2]+K[3][5]*H[5][2]);
    I[3][3] = 1-(K[3][0]*H[0][3]+K[3][1]*H[1][3]+K[3][2]*H[2][3]+K[3][3]*H[3][3]+K[3][4]*H[4][3]+K[3][5]*H[5][3]);
    
    I_KHP[0][0] = (I[0][0]*P[0][0]+I[0][1]*P[1][0]+I[0][2]*P[2][0]+I[0][3]*P[3][0]);
    I_KHP[0][1] = (I[0][0]*P[0][1]+I[0][1]*P[1][1]+I[0][2]*P[2][1]+I[0][3]*P[3][1]);
    I_KHP[0][2] = (I[0][0]*P[0][2]+I[0][1]*P[1][2]+I[0][2]*P[2][2]+I[0][3]*P[3][2]);
    I_KHP[0][3] = (I[0][0]*P[0][3]+I[0][1]*P[1][3]+I[0][2]*P[2][3]+I[0][3]*P[3][3]);
    I_KHP[1][0] = (I[1][0]*P[0][0]+I[1][1]*P[1][0]+I[1][2]*P[2][0]+I[1][3]*P[3][0]);
    I_KHP[1][1] = (I[1][0]*P[0][1]+I[1][1]*P[1][1]+I[1][2]*P[2][1]+I[1][3]*P[3][1]);
    I_KHP[1][2] = (I[1][0]*P[0][2]+I[1][1]*P[1][2]+I[1][2]*P[2][2]+I[1][3]*P[3][2]);
    I_KHP[1][3] = (I[1][0]*P[0][3]+I[1][1]*P[1][3]+I[1][2]*P[2][3]+I[1][3]*P[3][3]);
    I_KHP[2][0] = (I[2][0]*P[0][0]+I[2][1]*P[1][0]+I[2][2]*P[2][0]+I[2][3]*P[3][0]);
    I_KHP[2][1] = (I[2][0]*P[0][1]+I[2][1]*P[1][1]+I[2][2]*P[2][1]+I[2][3]*P[3][1]);
    I_KHP[2][2] = (I[2][0]*P[0][2]+I[2][1]*P[1][2]+I[2][2]*P[2][2]+I[2][3]*P[3][2]);
    I_KHP[2][3] = (I[2][0]*P[0][3]+I[2][1]*P[1][3]+I[2][2]*P[2][3]+I[2][3]*P[3][3]);
    I_KHP[3][0] = (I[3][0]*P[0][0]+I[3][1]*P[1][0]+I[3][2]*P[2][0]+I[3][3]*P[3][0]);
    I_KHP[3][1] = (I[3][0]*P[0][1]+I[3][1]*P[1][1]+I[3][2]*P[2][1]+I[3][3]*P[3][1]);
    I_KHP[3][2] = (I[3][0]*P[0][2]+I[3][1]*P[1][2]+I[3][2]*P[2][2]+I[3][3]*P[3][2]);
    I_KHP[3][3] = (I[3][0]*P[0][3]+I[3][1]*P[1][3]+I[3][2]*P[2][3]+I[3][3]*P[3][3]);

    P[0][0] = I_KHP[0][0];
    P[0][1] = I_KHP[0][1];
    P[0][2] = I_KHP[0][2];
    P[0][3] = I_KHP[0][3];
    P[1][0] = I_KHP[1][0];
    P[1][1] = I_KHP[1][1];
    P[1][2] = I_KHP[1][2];
    P[1][3] = I_KHP[1][3];
    P[2][0] = I_KHP[2][0];
    P[2][1] = I_KHP[2][1];
    P[2][2] = I_KHP[2][2];
    P[2][3] = I_KHP[2][3];
    P[3][0] = I_KHP[3][0];
    P[3][1] = I_KHP[3][1];
    P[3][2] = I_KHP[3][2];
    P[3][3] = I_KHP[3][3];

    qw = qw + Q_out[0];
    qx = qx + Q_out[1];
    qy = qy + Q_out[2];
    qz = qz + Q_out[3];
    norm_q = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw = qw/norm_q;
    qx = qx/norm_q;
    qy = qy/norm_q;
    qz = qz/norm_q;

    // NaN is absorbing: once any component goes bad every subsequent
    // operation yields NaN too, the attitude output freezes forever, and
    // the motors keep flying on a dead estimate. Recovering is strictly
    // better than continuing - a couple of lost cycles beats an aircraft
    // with no working attitude. Detected via x!=x, which is only ever true
    // for NaN. Also catches a zero/denormal norm_q.
    // Bounds check, not a NaN check. The original x!=x test only catches
    // NaN - and infinity passes it, because inf==inf is true. The filter
    // blew up to inf on 2026-09-03, sailed straight through the guard, and
    // the attitude froze at identity for an entire session (every row of
    // flight_20260903_121449.csv) while the flag still read "no NaN".
    //
    // A normalised quaternion can never exceed 1, so anything outside +/-1.5
    // is broken by definition. This form is false for NaN too (all
    // comparisons against NaN are false), so it catches every bad case.
    if( !(qw > -1.5f && qw < 1.5f) || !(qx > -1.5f && qx < 1.5f) ||
        !(qy > -1.5f && qy < 1.5f) || !(qz > -1.5f && qz < 1.5f) ){
        qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
        for(int r = 0; r < 4; r++){
            for(int c = 0; c < 4; c++){ P[r][c] = (r == c) ? 1.0f : 0.0f; }
        }
        if( ekf_nan_resets < 0xFF ){ ekf_nan_resets++; }
    }

    // Body rates for the PID now come straight from the gyro, in deg/s.
    //
    // They used to be recovered by differentiating the EKF quaternion:
    //     q_dot   = (q - q_prev)/loop_time            <- divide by 5 ms
    //     wx_crct = 2*57.2958*( -qx*q_dot[0] + qw*q_dot[1] + ... )
    // which is algebraically the same quantity (verified: identical to
    // within 0.4 deg/s across 2000 random attitudes and rates - the
    // residual is just Euler-step error). Same axes, same signs, same
    // units. The difference is that one is MEASURED and the other is
    // DIFFERENTIATED, and dividing by a 5 ms step multiplies any wobble in
    // the estimate by 200 before it reaches the motors.
    //
    // Flight log evidence (2026-09-02): sitting still on the ground at 14%
    // throttle, attitude was steady to 1.1 deg sd, yet the recovered
    // Roll_PID swung -78..+47 and Pitch_PID -80..+75 - against a hover
    // throttle of only ~140/1000. Motor spread was 0 below 5% throttle and
    // 145 average (345 peak) at 14%, i.e. it scaled with how hard the
    // motors were shaking the airframe. The controller was chasing
    // vibration, not attitude.
    //
    // The gyro needs no differentiation, and the MPU6050's own DLPF is
    // already set to 41 Hz (register 0x1A = 0x03), so this is filtered in
    // hardware before it is ever read. This is what every modern flight
    // controller feeds its rate loop with.
    wx_crct = wx * 57.2957795f;
    wy_crct = wy * 57.2957795f;
    wz_crct = wz * 57.2957795f;


    float temp_angle_half, q[4], q1,q2;
    temp_angle_half = ( roll_offset_angle/2.0f ) * 0.0174532925f;
    q_level_rot[0] = cosf( temp_angle_half ); q_level_rot[2] = sinf( temp_angle_half );
    temp_angle_half = ( pitch_offset_angle/2.0f ) * 0.0174532925f;
    q1 = cosf( temp_angle_half ); q2 = sinf( temp_angle_half );
    q[0] = q1*q_level_rot[0]; q[1] = q2*q_level_rot[0]; q[2] = q1*q_level_rot[2]; q[3] = q2*q_level_rot[2];
    q_level_rot[0] = q[0]; q_level_rot[1] = q[1]; q_level_rot[2] = q[2]; q_level_rot[3] = q[3];

    // q_ekf (x) q_level_rot - a RIGHT multiply, i.e. the correction is
    // applied in the body frame.
    //
    // This used to be a left multiply (q_level_rot (x) q_ekf), which
    // applies the correction in the reference frame instead. Those agree
    // only at zero yaw. The tilt is measured as body-frame Euler angles,
    // so correcting in the reference frame cancels nothing once the drone
    // is pointing anywhere but north: measured on the bench at yaw -35
    // deg, a -10.23/+4.54 correction left a 4.4/5.1 deg residual instead
    // of zero, matching this model to within the telemetry's quantization.
    // Since q_level_rot is already built as Rx(pitch_off) (x) Ry(roll_off),
    // swapping the operand order is the whole fix.
    q_leveled[0] = (qw*q_level_rot[0] - qx*q_level_rot[1] - qy*q_level_rot[2] - qz*q_level_rot[3]);
    q_leveled[1] = (qw*q_level_rot[1] + qx*q_level_rot[0] + qy*q_level_rot[3] - qz*q_level_rot[2]);
    q_leveled[2] = (qw*q_level_rot[2] - qx*q_level_rot[3] + qy*q_level_rot[0] + qz*q_level_rot[1]);
    q_leveled[3] = (qw*q_level_rot[3] + qx*q_level_rot[2] - qy*q_level_rot[1] + qz*q_level_rot[0]);

    yaw_rad = atan2f( 2*(q_leveled[0]*q_leveled[3] + q_leveled[1]*q_leveled[2]) , (1 - 2*(q_leveled[2]*q_leveled[2] + q_leveled[3]*q_leveled[3])) );
    
    q_prev[0]=qw; q_prev[1]=qx; q_prev[2]=qy; q_prev[3]=qz; // update prev quaternion
}

static uint32_t Level_Cal_Checksum(const LevelCalRecord *r){
    const uint8_t *p = (const uint8_t *)r;
    uint32_t sum = 0;
    for(uint32_t i = 0; i < offsetof(LevelCalRecord, checksum); i++){ sum += p[i]; }
    return sum ^ 0xA5A5A5A5u;
}

// Read back the stored zero point. A blank/never-written sector reads as
// 0xFF everywhere, which fails the magic check, so a Pico that has never
// been levelled simply starts at 0/0 - exactly the behaviour it had
// before this existed.
static void Level_Cal_Load(){
    const LevelCalRecord *rec = (const LevelCalRecord *)(XIP_BASE + LEVEL_CAL_FLASH_OFFSET);
    if( rec->magic == LEVEL_CAL_MAGIC && rec->checksum == Level_Cal_Checksum(rec)
        && rec->roll  > -LEVEL_CAL_MAX_TRIM_DEG && rec->roll  < LEVEL_CAL_MAX_TRIM_DEG
        && rec->pitch > -LEVEL_CAL_MAX_TRIM_DEG && rec->pitch < LEVEL_CAL_MAX_TRIM_DEG ){
        stored_roll_offset  = rec->roll;
        stored_pitch_offset = rec->pitch;
    } else {
        stored_roll_offset  = 0.0f;
        stored_pitch_offset = 0.0f;
    }
}

// Erase + reprogram the last flash sector. Interrupts must be off for the
// whole operation (the IRQ handlers themselves live in flash and cannot be
// fetched while it is being erased), which stalls the USB link for a few
// tens of milliseconds - harmless, because this is only ever reached with
// the motors off and the drone sitting on the ground.
static void Level_Cal_Save(){
    static uint8_t page[FLASH_PAGE_SIZE];
    LevelCalRecord rec;
    rec.magic    = LEVEL_CAL_MAGIC;
    rec.roll     = stored_roll_offset;
    rec.pitch    = stored_pitch_offset;
    rec.checksum = Level_Cal_Checksum(&rec);

    memset(page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(page, &rec, sizeof(rec));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(LEVEL_CAL_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(LEVEL_CAL_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

static void Level_Capture_Start(){
    if( ctrl_channel[2] != 0 ){ level_cal_result = LEVEL_RES_THROTTLE; return; }
    level_cal_state      = LEVEL_CAL_SETTLING;
    level_cal_result     = LEVEL_RES_NONE;
    level_cal_loops      = 0;
    level_cal_sum_roll   = 0.0f;
    level_cal_sum_pitch  = 0.0f;
    level_cal_w_sum[0]   = 0.0f;
    level_cal_w_sum[1]   = 0.0f;
    level_cal_w_sum[2]   = 0.0f;
}

// Called every loop straight after EKF_Run(). Does nothing at all unless a
// capture is actually running.
static void Level_Capture_Run(){
    if( level_cal_state == LEVEL_CAL_IDLE ){ return; }

    // Any throttle, or any real movement, invalidates the whole average -
    // a bumped drone would otherwise bake its bump into the stored zero
    // point and fly permanently crooked from then on.
    if( ctrl_channel[2] != 0 ){
        level_cal_state = LEVEL_CAL_IDLE; level_cal_result = LEVEL_RES_THROTTLE; return;
    }
    // Absolute cap only - this catches the drone actually being picked up
    // or waved about. It cannot be tight, because the gyro readings carry
    // an uncalibrated bias (see level_cal_w_mean above).
    if( fabsf(wx) > LEVEL_CAL_ABS_RATE_RADS || fabsf(wy) > LEVEL_CAL_ABS_RATE_RADS || fabsf(wz) > LEVEL_CAL_ABS_RATE_RADS ){
        level_cal_state = LEVEL_CAL_IDLE; level_cal_result = LEVEL_RES_MOVED; return;
    }

    level_cal_loops++;

    if( level_cal_state == LEVEL_CAL_SETTLING ){
        // Learn whatever this gyro reads while standing still, so the real
        // stillness test below measures movement rather than bias.
        level_cal_w_sum[0] += wx;
        level_cal_w_sum[1] += wy;
        level_cal_w_sum[2] += wz;
        if( level_cal_loops >= LEVEL_CAL_SETTLE_LOOPS ){
            level_cal_w_mean[0] = level_cal_w_sum[0] / (float)LEVEL_CAL_SETTLE_LOOPS;
            level_cal_w_mean[1] = level_cal_w_sum[1] / (float)LEVEL_CAL_SETTLE_LOOPS;
            level_cal_w_mean[2] = level_cal_w_sum[2] / (float)LEVEL_CAL_SETTLE_LOOPS;
            level_cal_state = LEVEL_CAL_SAMPLING;
            level_cal_loops = 0;
        }
        return;
    }

    // The real "did it move" test: deviation from the rate it was already
    // reading while still.
    if( fabsf(wx - level_cal_w_mean[0]) > LEVEL_CAL_MAX_RATE_RADS ||
        fabsf(wy - level_cal_w_mean[1]) > LEVEL_CAL_MAX_RATE_RADS ||
        fabsf(wz - level_cal_w_mean[2]) > LEVEL_CAL_MAX_RATE_RADS ){
        level_cal_state = LEVEL_CAL_IDLE; level_cal_result = LEVEL_RES_MOVED; return;
    }

    // Raw, un-trimmed tilt straight off the EKF quaternion, in the same
    // axis convention get_error_angles_from_Quaternion() uses: "pitch" is
    // the rotation about body X, "roll" the rotation about body Y.
    float sin_arg = 2*(qw*qy - qz*qx);
    if( sin_arg >  1.0f ){ sin_arg =  1.0f; }
    else if( sin_arg < -1.0f ){ sin_arg = -1.0f; }
    level_cal_sum_pitch += 57.2958f*atan2f( 2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy) );
    level_cal_sum_roll  += 57.2958f*asinf( sin_arg );

    if( level_cal_loops < LEVEL_CAL_SAMPLE_LOOPS ){ return; }

    // q_leveled = q_level_rot (x) q_ekf, and two small rotations compose
    // additively to first order, so the angle the flight controller acts
    // on is always (offset + real tilt). Cancelling a measured tilt
    // therefore means storing its negative.
    float new_roll  = - level_cal_sum_roll  / (float)LEVEL_CAL_SAMPLE_LOOPS;
    float new_pitch = - level_cal_sum_pitch / (float)LEVEL_CAL_SAMPLE_LOOPS;

    if( new_roll  < -LEVEL_CAL_MAX_TRIM_DEG || new_roll  > LEVEL_CAL_MAX_TRIM_DEG ||
        new_pitch < -LEVEL_CAL_MAX_TRIM_DEG || new_pitch > LEVEL_CAL_MAX_TRIM_DEG ){
        // More than 15 degrees out is not a mounting tolerance, it is the
        // drone sitting on a slope or the board bolted on crooked. Refuse
        // rather than silently storing a huge permanent offset.
        level_cal_state = LEVEL_CAL_IDLE; level_cal_result = LEVEL_RES_RANGE; return;
    }

    stored_roll_offset  = new_roll;
    stored_pitch_offset = new_pitch;
    Level_Cal_Save();
    level_cal_state  = LEVEL_CAL_IDLE;
    level_cal_result = LEVEL_RES_OK;
}

static void PID_angular_rates_ctrl(){
    // Roll/pitch rate P, scaled down ~35% from 1.82921/0.0003 on 2026-09-04
    // (see angle_to_rate_gain note). Combined with the angle-gain cut, the
    // roll/pitch loop gain is now ~0.44x what it was when it oscillated into
    // the ceiling on the correct props. Same throttle-rolloff SHAPE kept.
    // D is left alone on purpose - it adds damping/phase-lead, which helps
    // stability; only the P path was too hot.
    twoX_P_gain = 1.20f - 0.0002f*ctrl_channel[2];
    twoX_D_gain = 0.03f - 0.0000026f*ctrl_channel[2];

    if(is_GPS_mode_ON == false){
        ctrl_roll  = (float)(ctrl_channel[0]-50) * 0.5f;
        ctrl_pitch = (float)(50-ctrl_channel[1]) * 0.5f;
    }
    else if(is_GPS_mode_ON == true){
        ctrl_roll  = (float)(GPHC_roll_angle_out_body) * 0.5f;
        ctrl_pitch = (float)(-GPHC_pitch_angle_out_body) * 0.5f;
    }

    if( ctrl_channel[2] == 0 ){ ctrl_yaw = 57.2957795f*yaw_rad; }
    ctrl_yaw += (float)(50-ctrl_channel[3]) * 0.008; // inc or dec the constant value for fast or slow yaw rates.

    Quaternion qf = get_Quaternion_from_bodyframe_angles( ctrl_roll, ctrl_pitch, ctrl_yaw);
    Quaternion q_error = q1_dot_q2( q_leveled[0], -q_leveled[1], -q_leveled[2], -q_leveled[3],    qf.w, qf.x, qf.y, qf.z );
    Angles error_angles = get_error_angles_from_Quaternion(quat_inv(q_error));

    //---------------------------------------- Roll PID --------------------------------------//
    desired_angular_rate[0] = - error_angles.roll;

    // && , not || - the original condition (x > -11 || x < 11) is true for
    // EVERY number, so the linear branch always ran and both sqrt branches
    // below were unreachable. That removed the soft limit on large tilts:
    // at 30 deg of error it demanded 90 deg/s instead of 61, at 45 deg it
    // asked for 135 instead of 75. Hence the over-correction and chop.
    // The curves are continuous at the 11 deg boundary (11*3 = 33, and
    // 13 + 10*sqrt(11-7) = 33), which confirms this was the intent.
    if( desired_angular_rate[0] > -11.0f && desired_angular_rate[0] < 11.0f ){ desired_angular_rate[0] = desired_angular_rate[0] * angle_to_rate_gain; }
    else if( desired_angular_rate[0] >= 11.0f ){ desired_angular_rate[0] =   ( 13.0f + 10.0f * sqrtf(   desired_angular_rate[0] -  7.0f) ); }
    else if( desired_angular_rate[0] < -11.0f ){ desired_angular_rate[0] = - ( 13.0f + 10.0f * sqrtf( - desired_angular_rate[0] -  7.0f) ); }

    error[0] = desired_angular_rate[0] -wy_crct;
    setpoint_free_error[0] = -wy_crct;
    pid_Integral[0] += twoX_I_gain * loop_time * ( prev_error[0] + error[0] )/2.0f;
    if ( pid_Integral[0]*error[0] < 0){ pid_Integral[0] = 0; }
    pid_Derivative[0] =  0.7f*pid_Derivative[0] + 0.3f*twoX_D_gain*( setpoint_free_error[0] - setpoint_free_prev_error[0] )*200.0f;  //200 is used insteaad of 1/looptime to avoid time noise
    Roll_PID  = int(  twoX_P_gain*error[0] + pid_Integral[0] + pid_Derivative[0] );
    if( Roll_PID < -Roll_PID_lim ){ Roll_PID = -Roll_PID_lim; }  else if( Roll_PID > Roll_PID_lim ){ Roll_PID = Roll_PID_lim; }
    prev_error[0] = error[0];
    setpoint_free_prev_error[0] = setpoint_free_error[0];
    //-----------------------------------------------------------------------------------------//

    //---------------------------------------- Pitch PID --------------------------------------//
    desired_angular_rate[1] = - error_angles.pitch;

    // && , not || - same always-true bug as the roll axis above.
    if( desired_angular_rate[1] > -11.0f && desired_angular_rate[1] < 11.0f ){ desired_angular_rate[1] = desired_angular_rate[1] * angle_to_rate_gain; }
    else if( desired_angular_rate[1] >= 11.0f ){ desired_angular_rate[1] =   ( 13.0f + 10.0f * sqrtf(   desired_angular_rate[1] -  7.0f) ); }
    else if( desired_angular_rate[1] < -11.0f ){ desired_angular_rate[1] = - ( 13.0f + 10.0f * sqrtf( - desired_angular_rate[1] -  7.0f) ); }

    error[1] = desired_angular_rate[1] -wx_crct;
    setpoint_free_error[1] = -wx_crct;
    pid_Integral[1] += twoX_I_gain * loop_time * ( prev_error[1] + error[1] )/2.0f;
    if ( pid_Integral[1]*error[1] < 0){ pid_Integral[1] = 0; }
    pid_Derivative[1] =  0.7f*pid_Derivative[1] + 0.3f*twoX_D_gain*( setpoint_free_error[1] - setpoint_free_prev_error[1] )*200.0f;  //200 is used insteaad of 1/looptime to avoid time noise
    Pitch_PID  = int(  twoX_P_gain*error[1] + pid_Integral[1] + pid_Derivative[1] );
    if( Pitch_PID < -Pitch_PID_lim ){ Pitch_PID = -Pitch_PID_lim; }  else if( Pitch_PID > Pitch_PID_lim ){ Pitch_PID = Pitch_PID_lim; }
    prev_error[1] = error[1];
    setpoint_free_prev_error[1] = setpoint_free_error[1];
    //----------------------------------------------------------------------------------------//

    //---------------------------------------- Yaw PID ---------------------------------------//
    desired_angular_rate[2] =  - error_angles.yaw * 4.0f;
    error[2] = desired_angular_rate[2] -wz_crct;
    setpoint_free_error[2] = -wz_crct;
    pid_Integral[2] += Yaw_I_gain * loop_time * ( prev_error[2] + error[2] )/2.0f;
    if ( pid_Integral[2]*error[2] < 0){ pid_Integral[2] = 0; }
    pid_Derivative[2] =  0.7f*pid_Derivative[2] + 0.3f*Yaw_D_gain*( setpoint_free_error[2] - setpoint_free_prev_error[2] )*200.0f;  //200 is used insteaad of 1/looptime to avoid time noise
    Yaw_PID  = int(  Yaw_P_gain*error[2] + pid_Integral[2] + pid_Derivative[2] );
    if( Yaw_PID < -Yaw_PID_lim ){ Yaw_PID = -Yaw_PID_lim; }  else if( Yaw_PID > Yaw_PID_lim ){ Yaw_PID = Yaw_PID_lim; }
    prev_error[2] = error[2];
    setpoint_free_prev_error[2] = setpoint_free_error[2];
    //----------------------------------------------------------------------------------------//

}

static void GPS_Position_Hold(){
    if( is_GPS_mode_ON == false ){ is_GPS_Hold_once_run_done = false; }


    if( gps_decode_loop_shape_count == 0 && is_GPS_mode_ON == true ){

        LAT_DEG_TO_METERS = 111132.954 - 559.82 * cos(2.0*Vehicle_Lattitude*0.01745329251) + 1.175 * cos(4.0*Vehicle_Lattitude*0.01745329251) - 0.0023 * cos(6.0*Vehicle_Lattitude*0.01745329251);
        LON_DEG_TO_METERS = 111132.954 * cos(Vehicle_Lattitude*0.01745329251) - 93.5*cos(3.0*Vehicle_Lattitude*0.01745329251) + 0.118*cos(5.0*Vehicle_Lattitude*0.01745329251);

        velocity_vector_north_frame[0] = 0.15f*velocity_vector_north_frame[0] + 0.85f *  (float)( (Vehicle_Longitude - Vehicle_Longitude_Prev)* LON_DEG_TO_METERS )*10.0f;      // 10Hz loop freq
        velocity_vector_north_frame[1] = 0.15f*velocity_vector_north_frame[1] + 0.85f *  (float) ( (Vehicle_Lattitude - Vehicle_Lattitude_Prev)* LAT_DEG_TO_METERS )*10.0f;

        if( is_GPS_Hold_once_run_done == false ){     //safely set velocity zero for first time run only. Just to avoid big velocity error when current_coordinates are huge and prevs are 0.
            velocity_vector_north_frame[0] = 0; velocity_vector_north_frame[1] = 0;
            Vehicle_desired_Lattitude = Vehicle_Lattitude;  Vehicle_desired_Longitude = Vehicle_Longitude;
            Vehicle_Lattitude_Prev = Vehicle_Lattitude;  Vehicle_Longitude_Prev = Vehicle_Longitude;
            GPHC_roll_I = 0;  GPHC_pitch_I = 0;
            is_GPS_Hold_once_run_done = true;
        }

        float longitude_inc_body = ( (float)(ctrl_channel[0]-50) * ctrl_stick_to_velocity_div_gain * 0.1f ) / LON_DEG_TO_METERS;  // 0.1f loop time in seconds.;
        float lattitude_inc_body = ( (float)(ctrl_channel[1]-50) * ctrl_stick_to_velocity_div_gain * 0.1f ) / LAT_DEG_TO_METERS;

        Vehicle_desired_Longitude += ( longitude_inc_body*cosf(yaw_rad) - lattitude_inc_body*sinf(yaw_rad) );     // body to north frame conversion & add.
        Vehicle_desired_Lattitude += ( longitude_inc_body*sinf(yaw_rad) + lattitude_inc_body*cosf(yaw_rad) );

        v_p_n_f_error[0] = ( Vehicle_desired_Longitude - Vehicle_Longitude ) * LON_DEG_TO_METERS;
        v_p_n_f_error[1] = ( Vehicle_desired_Lattitude - Vehicle_Lattitude ) * LAT_DEG_TO_METERS;

        GPHC_roll_P =  GPHC_P_gain * v_p_n_f_error[0];
        GPHC_pitch_P = GPHC_P_gain * v_p_n_f_error[1];
        
        GPHC_roll_I  += GPHC_I_gain * v_p_n_f_error[0] * 0.1f;       // GPHC loop time 0.1f Seconds 
        GPHC_pitch_I += GPHC_I_gain * v_p_n_f_error[1] * 0.1f;       // GPHC loop time 0.1f Seconds
        
        if ( GPHC_roll_I > 15.0f ){ GPHC_roll_I  = 15.0f; }  else if (  GPHC_roll_I < -15.0f ){ GPHC_roll_I  = -15.0f; }
        if ( GPHC_pitch_I > 15.0f ){ GPHC_pitch_I  = 15.0f; }  else if (  GPHC_pitch_I < -15.0f ){ GPHC_pitch_I  = -15.0f; }

        GPHC_roll_D =  GPHC_D_gain * velocity_vector_north_frame[0];
        GPHC_pitch_D = GPHC_D_gain * velocity_vector_north_frame[1];

        GPHC_roll_angle_out_north =  GPHC_roll_P  + GPHC_roll_I  - GPHC_roll_D;
        GPHC_pitch_angle_out_north = GPHC_pitch_P + GPHC_pitch_I - GPHC_pitch_D;

        GPHC_roll_angle_out_body  = ( GPHC_roll_angle_out_north*cosf(-yaw_rad) - GPHC_pitch_angle_out_north*sinf(-yaw_rad) );
        GPHC_pitch_angle_out_body = ( GPHC_roll_angle_out_north*sinf(-yaw_rad) + GPHC_pitch_angle_out_north*cosf(-yaw_rad) );

        if(GPHC_roll_angle_out_body > GPHC_Vehicle_MAX_Lean_Angle){ GPHC_roll_angle_out_body = GPHC_Vehicle_MAX_Lean_Angle; }
        else if(GPHC_roll_angle_out_body < -GPHC_Vehicle_MAX_Lean_Angle){ GPHC_roll_angle_out_body = -GPHC_Vehicle_MAX_Lean_Angle; }

        if(GPHC_pitch_angle_out_body > GPHC_Vehicle_MAX_Lean_Angle){ GPHC_pitch_angle_out_body = GPHC_Vehicle_MAX_Lean_Angle; }    
        else if(GPHC_pitch_angle_out_body < -GPHC_Vehicle_MAX_Lean_Angle){ GPHC_pitch_angle_out_body = -GPHC_Vehicle_MAX_Lean_Angle; }


        //================================================================//
        out_status[0] = (uint8_t) ( (int)(velocity_vector_north_frame[0]*10.0f) + 128 );
        out_status[1] = (uint8_t) ( (int)(velocity_vector_north_frame[1]*10.0f) + 128 );
        //================================================================//

        Vehicle_Lattitude_Prev = Vehicle_Lattitude;  Vehicle_Longitude_Prev = Vehicle_Longitude;
    }
}

static void Motor_Drive(){
    // Yaw signs inverted 2026-09-01, forced by the output remap in
    // PWM_Write(). Yaw torque comes from speeding up the two props that
    // spin the SAME way, so the +Yaw pair must be one rotation family.
    //
    // Stock: +Yaw went to {m0,m2} = front-left + rear-right = the CW props
    // (per V4_Wiring_Diagram.png: FL and RR are CW, FR and RL are CCW).
    // After the remap {m0,m2} = front-right + rear-left, which is the CCW
    // pair - still a valid diagonal, but the opposite family, so yaw
    // correction would have become positive feedback and wound the drone
    // into a spin. Flipping the signs puts +Yaw back on the CW pair, which
    // is now {m1,m3}.
    //
    // NOTE: this half depends on the props matching the diagram's CW/CCW
    // layout, which was not measured. Roll and pitch above were derived
    // from bench measurements and are solid; if the drone holds level but
    // slowly spins about the vertical axis, flip these four Yaw_PID signs
    // back and nothing else.
    motor_out[0] = ctrl_channel[2] + Roll_PID + Pitch_PID - Yaw_PID;
    motor_out[1] = ctrl_channel[2] - Roll_PID + Pitch_PID + Yaw_PID;
    motor_out[2] = ctrl_channel[2] - Roll_PID - Pitch_PID - Yaw_PID;
    motor_out[3] = ctrl_channel[2] + Roll_PID - Pitch_PID + Yaw_PID;

    if(motor_out[0] > 1000){ motor_out[0] = 1000; }
    else if(motor_out[0] < 50){ motor_out[0] = 50; }
    if(motor_out[1] > 1000){ motor_out[1] = 1000; }
    else if(motor_out[1] < 50){ motor_out[1] = 50; }
    if(motor_out[2] > 1000){ motor_out[2] = 1000; }
    else if(motor_out[2] < 50){ motor_out[2] = 50; }
    if(motor_out[3] > 1000){ motor_out[3] = 1000; }
    else if(motor_out[3] < 50){ motor_out[3] = 50; }
    if(ctrl_channel[2] == 0){ motor_out[0] = 0; motor_out[1] = 0; motor_out[2] = 0; motor_out[3] = 0; pid_Integral[0]=0; pid_Integral[1]=0; pid_Integral[2]=0; }

    PWM_Write();
}

static void Tx_Rx_Update_Variables(){

    //----------INPUT----------//
    if(uart0_in_buff[0] == '$' && uart0_in_buff[12] == '*'){
        ctrl_channel[0] = uart0_in_buff[1];
        ctrl_channel[1] = uart0_in_buff[2];
        ctrl_channel[2] = uart0_in_buff[3]*10;
        ctrl_channel[3] = uart0_in_buff[4];

        in_cmd[0] = uart0_in_buff[5];
        in_cmd[1] = uart0_in_buff[6];

        // Stored zero point (the sensor-to-frame mounting tilt captured by
        // Level_Capture_Run()) plus the pilot's manual trim bytes, which
        // still work exactly as before as a fine adjustment on top of it.
        roll_offset_angle  = stored_roll_offset  + ( (float)uart0_in_buff[10] - 25.0f );
        pitch_offset_angle = stored_pitch_offset + ( (float)uart0_in_buff[11] - 25.0f );

        //-- ========= CMD_Input ========= --//

        // Edge-triggered: the Pi holds a command in the packet for as long
        // as the button is held, and a level capture must start once, not
        // restart itself every 33 ms.
        if( in_cmd[0] != 'L' ){ level_cal_cmd_latched = false; }

        if( in_cmd[0] == 'M' && in_cmd[1] == 'G' ){ is_GPS_mode_ON = true; }
        else if( in_cmd[0] == 'M' && in_cmd[1] == 'N' ){ is_GPS_mode_ON = false; }
        else if( in_cmd[0] == 'L' && in_cmd[1] == 'C' ){
            if( !level_cal_cmd_latched ){ level_cal_cmd_latched = true; Level_Capture_Start(); }
        }
        else if( in_cmd[0] == 'L' && in_cmd[1] == 'R' ){
            if( !level_cal_cmd_latched ){
                level_cal_cmd_latched = true;
                if( ctrl_channel[2] == 0 ){
                    stored_roll_offset  = 0.0f;
                    stored_pitch_offset = 0.0f;
                    Level_Cal_Save();
                    level_cal_state  = LEVEL_CAL_IDLE;
                    level_cal_result = LEVEL_RES_OK;
                } else {
                    level_cal_result = LEVEL_RES_THROTTLE;
                }
            }
        }

        //-- ============================= --//
    }
    //-------------------------//

    //---------OUTPUT----------//
    uart0_out_buff[0] = '$';
        
    uart0_out_buff[1] = int(q_leveled[0]*100.0f) + 100;
    uart0_out_buff[2] = int(q_leveled[1]*100.0f) + 100;
    uart0_out_buff[3] = int(q_leveled[2]*100.0f) + 100;
    uart0_out_buff[4] = int(q_leveled[3]*100.0f) + 100;


    // GPS block, packet bytes 5-30.
    //
    // Byte 26 is the fix state and is now always one of '0'/'1'/'2'/'3'. It
    // used to be possible for Fix_type to be 0 (its value at boot, before any
    // sentence had ever been parsed) which matched neither branch below - so
    // bytes 5-30 kept whatever was last left in the buffer and the dashboard
    // was shown stale position data. There is an explicit else now.
    //
    // The satellite count goes out even with no fix. It used to be forced to
    // zero here, which defeated server.py's own handling ("watching satellites
    // climb while you WAIT for a fix") - the one number that tells you the
    // receiver is alive and searching rather than dead.
    if(Fix_type == '2' || Fix_type == '3'){
        for (uint8_t i = 5; i < 15; i++){ uart0_out_buff[i] = Lattitude[i-5]; }
        for (uint8_t i = 15; i < 26; i++){ uart0_out_buff[i] = Longitude[i-15]; }
        uart0_out_buff[26] = Fix_type;
        uart0_out_buff[27] = Sat_count;
        uart0_out_buff[28] = HDOP[0]; uart0_out_buff[29] = HDOP[1]; uart0_out_buff[30] = HDOP[2];
    }
    else {
        // No fix. Position fields are filled with a syntactically valid but
        // obviously null coordinate; the Pi ignores them while has_fix is
        // false in any case.
        for (uint8_t i = 5; i < 31; i++){ uart0_out_buff[i] = '0'; }
        uart0_out_buff[9] = '.'; uart0_out_buff[20] = '.';
        uart0_out_buff[26] = (Fix_type == '0' || Fix_type == '1') ? Fix_type : '0';
        uart0_out_buff[27] = Sat_count;                      // real count, may be 0

        // GPS receive diagnostics, in the three bytes HDOP occupies when
        // there IS a fix. Nothing on the Pi reads HDOP, and with no fix it
        // has no value to report anyway - whereas these three bytes say
        // exactly where in the chain the GPS is failing:
        //   byte 28 - which baud the hunt is currently listening at, '0'-'4'
        //             indexing gps_baud_candidates[]
        //   byte 29 - high-to-low transitions on the RX line, saturating at 255
        //   byte 30 - words the DMA delivered, saturating at 255
        //
        // Both are HINTS ONLY - see the comment on gps_line_transitions. On
        // 20 Sep both climbed steadily while the dedicated gps_sniffer build
        // measured the same pin as completely static and decoded nothing at
        // any baud. Do not conclude "data is arriving" from either of them.
        //
        // The number that is trustworthy here is the fix state in byte 26:
        // it only ever leaves '0' when a sentence has actually passed its
        // checksum, which no amount of line noise will do.
        uart0_out_buff[28] = (uint8_t)('0' + gps_baud_idx);
        uart0_out_buff[29] = (uint8_t)((gps_line_transitions > 255u) ? 255u : gps_line_transitions);
        uart0_out_buff[30] = (uint8_t)((gps_raw_bytes > 255u) ? 255u : gps_raw_bytes);
    }

    uart0_out_buff[31] = out_status[0]; uart0_out_buff[32] = out_status[1];

    // BMP388 barometer - appended after the original 33-byte packet so
    // GPS/status field offsets above are untouched. Temperature is
    // offset-encoded one byte like the quaternion fields (int(C)+100,
    // clamped implicitly by the sensor's real-world range). Pressure is
    // sent as a big-endian uint16 in units of 10 Pa (i.e. 0.1 hPa) - 0 if
    // the sensor isn't present, which the dashboard should treat as "no
    // barometer" rather than a real 0 Pa reading.
    if(isBaroPresent){
        int temp_byte = (int)baro_temp_c + 100;
        if(temp_byte < 0){ temp_byte = 0; } else if(temp_byte > 255){ temp_byte = 255; }
        uart0_out_buff[33] = (uint8_t)temp_byte;
        uint16_t press_scaled = (uint16_t)(baro_pressure_pa / 10.0f);
        uart0_out_buff[34] = (uint8_t)(press_scaled >> 8);
        uart0_out_buff[35] = (uint8_t)(press_scaled & 0xFF);
    } else {
        uart0_out_buff[33] = 0;
        uart0_out_buff[34] = 0;
        uart0_out_buff[35] = 0;
    }

    // TEMPORARY debug readout: the actual computed motor_out[] values
    // (0-1000 each), big-endian uint16, one per channel - so we can see
    // directly what PWM_Write() is being told to output, correlated with
    // multimeter readings at the pins, without guessing from the firmware
    // source alone. Remove once the motor-spin issue is resolved.
    for(int i = 0; i < 4; i++){
        uart0_out_buff[37 + i*2]     = (uint8_t)(motor_out[i] >> 8);
        uart0_out_buff[37 + i*2 + 1] = (uint8_t)(motor_out[i] & 0xFF);
    }

    // TEMPORARY debug readout: echo back the raw 13 bytes of
    // uart0_in_buff exactly as received, so the Pi side can compare
    // "what I sent" against "what the Pico actually saw" byte-for-byte -
    // this will directly reveal any input-side framing/desync issue.
    for(int i = 0; i < uart0_in_buff_size; i++){
        uart0_out_buff[45 + i] = uart0_in_buff[i];
    }

    // Level-capture status: high nibble = state (0 idle, 1 settling,
    // 2 sampling), low nibble = result of the last attempt (0 none, 1 ok,
    // 2 aborted-moved, 3 out of range, 4 throttle not zero). Byte 36 was
    // the one spare slot left in the original packet.
    // Bit 7 additionally carries whether the boot-time gyro bias sweep
    // succeeded. It matters now that the rate loop runs off the raw gyro:
    // a rejected sweep means zero bias correction and a slow drift to one
    // side, and that must not fail silently. level_cal_state only ever
    // holds 0-2, so bits 6-7 were free and the packet layout is unchanged.
    uart0_out_buff[36] = (uint8_t)( (is_gyro_calibrated ? 0x80 : 0x00)
                                    | (ekf_nan_resets ? 0x40 : 0x00)
                                    | ((level_cal_state & 0x03) << 4)
                                    | (level_cal_result & 0x0F) );

    // Stored zero point, signed int16 big-endian in hundredths of a degree
    // - so the dashboard can show the actual captured numbers rather than
    // just "done". Appended after the debug block; packet is now 65 bytes.
    int16_t roll_off_enc  = (int16_t)( stored_roll_offset  * 100.0f );
    int16_t pitch_off_enc = (int16_t)( stored_pitch_offset * 100.0f );
    uart0_out_buff[58] = (uint8_t)( (roll_off_enc  >> 8) & 0xFF );
    uart0_out_buff[59] = (uint8_t)(  roll_off_enc        & 0xFF );
    uart0_out_buff[60] = (uint8_t)( (pitch_off_enc >> 8) & 0xFF );
    uart0_out_buff[61] = (uint8_t)(  pitch_off_enc       & 0xFF );

    // Bounded-out I2C transactions since boot, big-endian uint16, saturating.
    // A rising count with the link still alive is the fix working: the bus
    // glitched and the loop carried on instead of stopping forever.
    uart0_out_buff[62] = (uint8_t)( (i2c_fault_count >> 8) & 0xFF );
    uart0_out_buff[63] = (uint8_t)(  i2c_fault_count       & 0xFF );

    // DIAGNOSTIC BLOCK - instantaneous raw accel and the peak-hold since the
    // last packet, both in raw LSB, big-endian int16 / uint16.
    for(int a = 0; a < 3; a++){
        int16_t mean_a = accel_n ? (int16_t)(accel_sum[a] / (int32_t)accel_n) : accel[a];
        uart0_out_buff[64 + a*2]     = (uint8_t)(((uint16_t)mean_a >> 8) & 0xFF);
        uart0_out_buff[64 + a*2 + 1] = (uint8_t)((uint16_t)mean_a & 0xFF);
        accel_sum[a] = 0;
        uart0_out_buff[70 + a*2]     = (uint8_t)((accel_peak[a] >> 8) & 0xFF);
        uart0_out_buff[70 + a*2 + 1] = (uint8_t)(accel_peak[a] & 0xFF);
        accel_peak[a] = 0;   // reset the peak-hold for the next interval
    }
    accel_n = 0;

    // DIAGNOSTIC: |mag| in tenths of a uT, big-endian uint16.
    uint16_t mm = (uint16_t)(mag_mag_ut * 10.0f);
    uart0_out_buff[76] = (uint8_t)((mm >> 8) & 0xFF);
    uart0_out_buff[77] = (uint8_t)(mm & 0xFF);

    // ---- rail voltage ----
    // One conversion per packet (about 2 us) with a light IIR filter, so
    // the loop never waits on the ADC. ADC3 is VSYS/3 on a Pico.
    adc_select_input(3);
    {
        const float v_now = (float)adc_read() * 3.3f / 4095.0f * 3.0f;
        vsys_volts = (vsys_volts <= 0.1f) ? v_now
                                          : (0.92f * vsys_volts + 0.08f * v_now);
    }
    {
        int mv = (int)(vsys_volts * 1000.0f + 0.5f);
        if(mv < 0){ mv = 0; } else if(mv > 65535){ mv = 65535; }
        uart0_out_buff[78] = (uint8_t)((mv >> 8) & 0xFF);
        uart0_out_buff[79] = (uint8_t)(mv & 0xFF);
    }

    uart0_out_buff[80] = '*';
    //-------------------------//

}

static void phrase_and_set_Calibrated_Values(){
    uint8_t floatarr_index=0,start_index=0;
    for(int i=0; i<compass_Cal_buff_max_expected_len; i++){
        if(compassCalHoldBuff[i]==','){
            int tempChr_len = i-start_index;
            // +1 and an explicit NUL: atof() reads until it hits a
            // non-numeric byte, so without a terminator it ran off the end
            // of this buffer into whatever happened to be on the stack.
            // Usually harmless, but if those bytes were digits the parsed
            // calibration value came out silently wrong - on the numbers
            // that set the compass offsets and soft-iron matrix.
            char temp[tempChr_len + 1];
            for(uint8_t j=0; j<tempChr_len; j++){ temp[j] = compassCalHoldBuff[start_index+j];}
            temp[tempChr_len] = '\0';
            cal_arr[floatarr_index] = atof(temp);
            floatarr_index++;
            start_index = i+1;
            if(floatarr_index==9){break;}
        }
    }

    mag_cal[0][0] = cal_arr[0]; mag_cal[1][1] = cal_arr[1]; mag_cal[2][2] = cal_arr[2];
    mag_cal[0][1] = cal_arr[3]; mag_cal[1][0] = cal_arr[3];
    mag_cal[2][0] = cal_arr[4]; mag_cal[0][2] = cal_arr[4];
    mag_cal[1][2] = cal_arr[5]; mag_cal[2][1] = cal_arr[5];
    compass_offset_x = cal_arr[6];
    compass_offset_y = cal_arr[7];
    compass_offset_z = cal_arr[8];


}

int main() {
    
    sleep_ms(2000);
    Led_init();

    PWM_out_init();
    PWM_Write();

    // VSYS sense, for the rail voltage in the telemetry packet.
    adc_init();
    adc_gpio_init(29);

    UART1_setup(9600);
    GPS_init();
    DMA0_configure();
    // UART0_setup(921600) intentionally not called - the Pi5 link now runs
    // over USB CDC (see USB_CDC_* functions above), not the GP16/17 UART
    // wiring.

    gpio_put(PICO_DEFAULT_LED_PIN,1);
    USB_CDC_wait_for_calibration();
    phrase_and_set_Calibrated_Values();
    gpio_put(PICO_DEFAULT_LED_PIN,0);

    I2C_Init();
    mpu6050_init();
    qmc5883_init();
    bmp388_init();

    EKF_Init();
    Level_Cal_Load();

    while(1){
        timePrev = time;
        time = time_us_32();
        loop_time = (time - timePrev)/1000000.0f;
        // On the very first pass timePrev is still 0, so this measures
        // SECONDS SINCE BOOT rather than one loop period. The EKF's process
        // noise scales with loop_time squared (sigmaloop2b4 in EKF_Run), so
        // a multi-second dt can blow the filter up permanently - which is
        // exactly what happened when the boot-time gyro sweep added ~2.8 s
        // and pushed the first dt from ~5.4 s to ~8.2 s: the quaternion went
        // NaN and the attitude froze for good.
        // A flash write can inject a similar spike (Level_Cal_Save holds
        // interrupts off for tens of ms), so clamp anything absurd to the
        // nominal period rather than feeding it to the filter.
        if( loop_time <= 0.0f || loop_time > 0.05f ){
            loop_time = Main_Loop_Time_MICRO_SECONDS/1000000.0f;
        }

        IMU_Read();
        Baro_Read();
        EKF_Run();
        Level_Capture_Run();

        GPS_decode();

        GPS_Position_Hold();

        USB_CDC_Read();
        Is_uart0_receiving_and_Action();
        Led_set();

        Tx_Rx_Update_Variables();
        USB_CDC_Reply();

        PID_angular_rates_ctrl();

        Motor_Drive();

        while((time_us_32() - time) < Main_Loop_Time_MICRO_SECONDS);


    }
}