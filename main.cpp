#include <cmath>
#include <stdio.h>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h" // clk_sys
#include <RF24.h>        // RF24 radio object
#include <RF24Network.h> // RF24Network network object
#include "quadrature_encoder.pio.h" // Encoders PIO program

// Function prototype for 3rd DoF switch interruption handler
void gpio_callback(uint gpio, uint32_t events);

// low_level_sampling time. This is for motor control
float Ts = 25; // [ms]
// high_level_sampling time. This is for trayectory control
float Ts_h = 25; // [ms]

class SecondOrderButterworthFilter{
private:
    float fc; // Cut-off frequency
    double b[3] {0, 0, 0};
    double a[2] {0, 0};

public:
    SecondOrderButterworthFilter(float fc, float Ts) {
        float alpha = fc*2*M_PI*Ts/1000;
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];

        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];

        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;
    }

    double filter(double x0, double (&x)[3], double (&y)[3]){
        x[0] = x0;
        y[0] = a[0]*y[1] + a[1]*y[2] + b[0]*x[0] + b[1]*x[1] + b[2]*x[2];

        // Update values
        for (int i = 2; i>0; i--) {
            x[i] = x[i-1];
            y[i] = y[i-1];
        }

        return y[0];
    }
};

// Frequency of 2.125 works well, 2 works too
SecondOrderButterworthFilter butterworth_2nd = SecondOrderButterworthFilter(12, Ts); // [Hz] Declared before Motor class, because its used there


class PID{
    private:
        float e[3] {0, 0, 0};
        float c[2] {0, 0};

        float q0;
        float q1;
        float q2;

    public:

        PID(){} // Just to be able to add it as a member variable in the motor class
        PID(float kp, float td, float ti, float Ts) 
            : q0 {kp*td/Ts}, q1 {-kp - 2*kp*td/Ts}, q2 {kp*Ts/ti + kp + td*kp/Ts} {
        }

        float calculate_control(float sp, float pv){
            // Calculate error
            e[0] = sp-pv;

            // Calculate control law
            if (sp == 0){
                c[0] = 0;
            } else {
                c[0] = q2*e[0] + q1*e[1] + q0*e[2] + c[1];
                if (c[0] > 0 && c[0] < 50){
                    c[0] = 51;
                } else if (c[0] < 0 && c[0] > -50) {
                    c[0] = -51;
                }
            }
            
            // Shift registers
            for (int i {2}; i > 0; i--)
                e[i] = e[i-1];
            c[1] = c[0];

        return c[0];
    }

};


class PI_Controller{
private:
    float e[2] {0, 0};
    float c[2] {0, 0};

    float q0;
    float q1;

    int min_dc;

public:
    PI_Controller(){}
    PI_Controller(float K, float tau, float Ts, int _min_dc, float tss=0.7, float OS=0.01)
        : min_dc {_min_dc} {
        float xi = sqrt( ( pow(log(OS), 2) ) / ( pow(M_PI, 2) + pow(log(OS), 2) ) );
        float wn = 4/(xi*tss);
        float kp = (2*xi*wn*tau-1)/K;
        float ki = tau*pow(wn, 2)/K;

        q0 = -kp;
        q1 = kp+ki*Ts;
    }

    float calculate_control(float sp, float pv){
        // Calculate error
        e[0] = (sp-pv)*1;

        // Calculate control law
        if (sp == 0){
            c[0] = 0;
        } else {
            c[0] = q1*e[0] + q0*e[1] + c[1];

            if (sp != 0 && c[1] == 0) {
                if (c[0] < 0) {c[0] = -min_dc; }
                else {c[0] = min_dc; }
            }

            // Saturate the control signal to protect the motors for over voltage
            if (c[0] > 90) {c[0] = 90; }
            else if (c[0] < -90) {c[0] = -90; }
        }

        // Shift registers
        e[1] = e[0];
        c[1] = c[0];

        return c[0];
    }
};


// This controller should be used with relative coordinates, so that each control starts from x=0,y=0,th=0
class P_PositionOrientation_Controller{
private:
    // Controller parameters
    float kpPC, kpOC, kdPC, kdOC;

    // Linear speed value for robot
    float nominal_v;

    // Acceptable errors
    float threshold_pos, threshold_th;

    // Encapsulates the robot speeds in one data structure
    struct RobotSpeeds{
        float v, w;
    } control_signals {0.0, 0.0}; // v, w respectively

    // enum that helps with the state machine for the control of pose
    enum PosOrientControlStates : uint8_t {
        idle = 0, move_des_posi, orient_des_orie
    } curr_state = idle;

    struct ConditionsChangeStates {
        bool change_idle, at_des_pos, at_des_orie;
    } conditions {false, false, false};

    // float pv_zero[3] {0.0, 0.0, 0.0};
    // bool turn_sp_pv_relative {true};

public:
    P_PositionOrientation_Controller(){}
    P_PositionOrientation_Controller(float _kpPC, float _kpOC, float _kdPC, float _kdOC, float _nominal_v, float _threshold_pos, float _threshold_th)
        : kpPC {_kpPC}, kpOC {_kpOC}, kdPC {_kdPC}, kdOC {_kdOC}, nominal_v {_nominal_v}, threshold_pos {_threshold_pos}, threshold_th {_threshold_th} {   
    }

    void do_go_to_idle_helper(float sp[3], float pv[3]){
        curr_state = idle;
        conditions.change_idle = false;
    }

    // State machine director state
    void do_idle(){
        // Guard clause, only if conditions.change_idle is true it evaluates the other conditionals
        if (!conditions.change_idle){
            control_signals = {0.0, 0.0};
            return;
        }
        if (!conditions.at_des_pos){
            curr_state = move_des_posi;
            return;
        } else if (!conditions.at_des_orie){
            curr_state = orient_des_orie; // No return statement bc end of function, if this changes, remember to add return;
            return;
        }
    }

    void do_move_des_posi(float sp[3], float pv[3]){
        if (conditions.at_des_pos){ // This should ask && !conditions.at_des_orie, but another condition would be needed in order to go back to idle,
                                    // since conditions.at_des_orie is asked on the next state it is ommited. The cost of this: one more highlevel cycle
            do_go_to_idle_helper(sp, pv);
            return;
        }
        // Error
        float e_th = atan2(sp[1]-pv[1], sp[0]-pv[0]) - pv[2];

        // Experimental error so that the controller controls small divergences in angle but primarly control based on distance between actual and desired
        float e_l = sqrt(pow(sp[0]-pv[0], 2)+pow(sp[1]-pv[1], 2));

        pos_controller_calculate_control(e_th, e_l);
    }

    void do_orient_des_orie(float sp[3], float pv[3]){
        if (conditions.at_des_orie){
            do_go_to_idle_helper(sp, pv);
            return;
        }
        // Error
        float e_th = sp[2] - pv[2]; // Error between desired pose angle and current angle

        orient_controller_calculate_control(e_th);
    }

    void calculate_control(float sp[3], float pv[3], float (&c_speeds)[2]){

        float e_des_posi = sqrt(pow(sp[0]-pv[0], 2)+pow(sp[1]-pv[1], 2));
        float e_des_orie = sp[2]-pv[2];

        // This updates the conditions that determine which state the controller should be it
        conditions.at_des_pos = (e_des_posi < threshold_pos) ? true : false;
        conditions.at_des_orie = (fabs(e_des_orie) < threshold_th) ? true : false;

        // State machine that is in charge of running the controller once new_pose flag is set to true until it
        // turns back to false.
        // This implementation is needed bc we can't be evaluating a bunch of logic on every iteration in order to get the desired state
        switch(curr_state){
            case idle:
                do_idle();
                break;
            case move_des_posi:
                do_move_des_posi(sp, pv);
                break;
            case orient_des_orie:
                do_orient_des_orie(sp, pv);
                break;
        }

        c_speeds[0] = control_signals.v;
        c_speeds[1] = control_signals.w;

        // This errors' purpose is to be printed
        float e_th_PC = atan2(sp[1]-pv[1], sp[0]-pv[0]) - pv[2];
        float e_th_OC = sp[2] - pv[2];

        // To visualize each motor speed

        float wheel_D {31.4};
        float wheel_distance {117};

        float c_speeds_wheels[2];
        float A = (M_PI/60.0)*(wheel_D/2)*1000;
        float B = (2*M_PI/60.0)*(wheel_D/2)*(1/wheel_distance);

        c_speeds_wheels[0] = (1/2.0)*((1/A)*c_speeds[0] - (1/B)*c_speeds[1]); // Left wheel
        c_speeds_wheels[1] = -(1/2.0)*((1/A)*c_speeds[0] + (1/B)*c_speeds[1]); // Right wheel

        printf("%f %f %f %f %f %f %f %f %f %d %d %f %f %d\n", pv[0], pv[1], pv[2]*180/M_PI, e_th_PC, e_th_OC, c_speeds[0], c_speeds[1]*180/M_PI, c_speeds_wheels[0], c_speeds_wheels[1], conditions.at_des_pos, conditions.at_des_orie, e_des_posi, e_des_orie*180/M_PI, curr_state);
    }

    void pos_controller_calculate_control(float e_th, float e_l){
        // Control laws
        control_signals.v = 0.7*e_l; // This value has to be normalized, bc there are setpoints close to current position and others that aren't.
        control_signals.w = kpPC*e_th; // + kdPC*e_th_dot;
    }

    void orient_controller_calculate_control(float e_th){
        // Control laws
        control_signals.v = 0;
        control_signals.w = kpOC*e_th; // + kdOC*e_th_dot;
    }

    void update_state_exit_idle(){
        conditions.change_idle = true;
    }
};


class Motor{
private:
    // Motor PWM related variables
    uint sliceA1;
    uint sliceA2;
    uint channelA1;
    uint channelA2;
    uint32_t pwm_div = 0, pwm_top = 0;
    uint16_t pwm_wrap_point;
    uint32_t pwm_set_point;

    double pwm_dc_max {100};

    float dc; // Duty cicle [%]
    // Set point for speed
    double speed_sp {0};

    bool closed_loop;
    // Encoder related variables
    uint cw {1}; // 1: CW, 0 CCW

public:
    // Motor gpio
    uint pinA1;
    uint pinA2;

    // Encoder gpio
    uint pinC1;
    uint pinC2;

    // Encoder related variables
    int64_t enc_count {0};
    int64_t last_enc_count {0};

    // Unfiltered and filtered speed arrays, useful for filtering
    double speed_unf[3] {0ULL, 0ULL, 0ULL}; // Index 0 has the current value
    double speed_f[3] {0ULL, 0ULL, 0ULL}; // Index 0 has the current value
    float speed_corr_factor {0.0};

    // Flags for correct encoder count calculation - nonpio implementation
    bool ccw_fall {0}; // Old implementation
    bool cw_fall {0}; // Old implementation

    // Values for PIO for encoders - pio implementation
    uint sm;
    PIO pio;

    PI_Controller pi_controller;

    Motor(){} // This constructor is to be able to create the instances of Motor inside of differential robot and later on assign r-values to them
    // Constructor for motor instance
    Motor(uint _pinA1, uint _pinA2, float pwm_freq, PIO _pio, uint _pinC1, uint _pinC2, float _speed_corr_factor, bool _closed_loop, float K, float tau, float min_dc)
        : pinA1 {_pinA1}, pinA2 {_pinA2}, pio {_pio}, pinC1 {_pinC1}, pinC2 {_pinC2}, speed_corr_factor {_speed_corr_factor}, closed_loop {_closed_loop}{

        // -----PWM setup-----
        // set GPIO ports and get slices and channels
        pwm_set_gpio();
        // set the wrap point
        pwm_set_freq(pwm_freq);
        // pwm_set_wrap_point(pwm_freq);
        // set the set point for dutycicle of 0 y default
        pwm_set_dc();
        // enable the pwm slices
        pwm_set_enabled(sliceA1, true);
        pwm_set_enabled(sliceA2, true);

        // -----Encoder setup----- - pio implementation
        sm = pio_claim_unused_sm(pio, true);
        uint offset = pio_add_program(pio, &quadrature_encoder_program);
        quadrature_encoder_program_init(pio, sm, offset, pinC1, 0);

        // -----Controller setup-----
        pi_controller = PI_Controller(K, tau, Ts/1000, min_dc); // Remember to divide Ts by 1000
    }

    // Set the GPIO ports and get the corresponding slices and channels
    void pwm_set_gpio() {
        // enable GPIO ports for PWM
        gpio_set_function(pinA1, GPIO_FUNC_PWM);
        gpio_set_function(pinA2, GPIO_FUNC_PWM);

        // get the PWM slice number from the GPIO (there are 8)
        sliceA1 = pwm_gpio_to_slice_num(pinA1);
        sliceA2 = pwm_gpio_to_slice_num(pinA2);

        // Get the pwm channel from the GPIO (there is channel A and B)
        channelA1 = pwm_gpio_to_channel(pinA1);
        channelA2 = pwm_gpio_to_channel(pinA2);
    }

    // Set freq by setting wrap point, making sure the max resolution is achieved
    void pwm_set_freq(float pwm_freq) {
        // The code in this section is taken from the internet
        // Set the frequency, making "top" as large as possible for maximum resolution.
        pwm_div = (uint32_t)(16 * clock_get_hz(clk_sys) / (uint32_t)pwm_freq);
        pwm_top = 1;
        // 65534 is the max value in order to get PWM DC = 100%
        for (;;) {
            // Try a few small prime factors to get close to the desired frequency.
            if (pwm_div >= 16 * 5 && pwm_div % 5 == 0 && pwm_top * 5 <= 65534) {
                pwm_div /= 5;
                pwm_top *= 5;
            } else if (pwm_div >= 16 * 3 && pwm_div % 3 == 0 && pwm_top * 3 <= 65534) {
                pwm_div /= 3;
                pwm_top *= 3;
            } else if (pwm_div >= 16 * 2 &&pwm_top * 2 <= 65534) {
                pwm_div /= 2;
                pwm_top *= 2;
            } else {
                break;
            }
        }

        pwm_set_wrap(sliceA1, pwm_top); // Freq of 10kHz
        pwm_set_wrap(sliceA2, pwm_top);
    }

    // By default, duty cicle of 0
    void pwm_set_dc(int rotation=1, float _dc=0) {
        dc = _dc;
        // pwm_set_point = (uint16_t) pwm_wrap_point*dc/100.0;
        float scaler = pwm_dc_max/100; // 100 is the default value for DC, max_value is the max value 10 get a max of 6v at current supply voltage
        uint32_t duty = (uint32_t)(scaler*dc*(65534.0/100.0)); // Scale dc (0-100) to dc (0-65534)
        pwm_set_point = duty * (pwm_top + 1) / 65535; // Scale accoring to pwm_top that best suits the PWM's frequency

        if (rotation){
            cw = 1;
            pwm_set_chan_level(sliceA1, channelA1, pwm_set_point);
            pwm_set_chan_level(sliceA2, channelA2, 0);
        } else if (!rotation){
            cw = 0;
            pwm_set_chan_level(sliceA1, channelA1, 0);
            pwm_set_chan_level(sliceA2, channelA2, pwm_set_point);
        }
    }

    // Encoder count - pio implementation
    void update_enc_count() {
        enc_count = quadrature_encoder_get_count(pio, sm);
    }

    void calculate_speed(uint64_t delta_time) {
        int64_t temp_last_enc_count {enc_count}; // Snapshot of enc_count at function call
        int64_t delta_enc_count {temp_last_enc_count - last_enc_count};
        last_enc_count = temp_last_enc_count;

        double speed_unfiltered = (1/215.0)*((1e6)*(double)delta_enc_count/delta_time)*(1/(7.0*4))*60.0*speed_corr_factor;
        butterworth_2nd.filter(speed_unfiltered, speed_unf, speed_f); // Changes the vales of speed_unf and speed_f (not the best implementation, but whateves)
    }

    void calculate_control() {
        // Remember that the mean value of PWM output voltage varies with the battery voltage
        if (closed_loop) {
            float dc = pi_controller.calculate_control(speed_sp, speed_f[0]);

            // Deals with clockwise and counterclockwiser rotations
            if (dc<0)
                pwm_set_dc(0, -dc);
            else
                pwm_set_dc(1, dc);
        }
    }

    void set_speed(double speed) {speed_sp = speed; }
    void set_max_dc(double max_dc) {
        if (max_dc > 100) pwm_dc_max = 100;
        else pwm_dc_max = max_dc;
    }

    double get_max_dc(){return pwm_dc_max; }

    double get_speed(){return speed_sp; }

    float get_dc(){return dc; }
};


class MotorTwoPositions {
private:
    // -----Harware pins-----
    // Switches input pins
    uint8_t pin00deg;
    uint8_t pin90deg;
    // Output pins
    uint8_t pinA1;
    uint8_t pinA2;

    // -----PWM realted variables-----
    // Pin's PWM slices and channels
    uint8_t sliceA1;
    uint8_t sliceA2;
    uint8_t channelA1;
    uint8_t channelA2;
    // PWM freq related variales
    uint32_t pwm_div = 0, pwm_top = 0;
    uint16_t pwm_wrap_point;
    uint32_t pwm_set_point;
    // Duty cycle value
    float dc;

    // Position flags (these and the next two can be turned into one so that, but this way the code is more readable)
    bool at00deg {false};
    bool at90deg {false};

    // Two position control flags
    bool goto00deg {false};
    bool goto90deg {false};

public:
    MotorTwoPositions(uint8_t _pinA1, uint8_t _pinA2, float pwm_freq, uint8_t _pin00deg, uint8_t _pin90deg)
        : pinA1 {_pinA1}, pinA2 {_pinA2}, pin00deg {_pin00deg}, pin90deg {_pin90deg} {
        // -----PWM setup-----
        // set GPIO ports and get slices and channels
        pwm_set_gpio();
        // set the wrap point
        pwm_set_freq(pwm_freq);
        // pwm_set_wrap_point(pwm_freq);
        // set the set point for dutycicle of 0 y default
        pwm_set_dc();
        // enable the pwm slices
        pwm_set_enabled(sliceA1, true);
        pwm_set_enabled(sliceA2, true);

        // -----Switches setup-----
        switches_set_gpio(); // Set gpio and interruptions for switches

        // +++++++++++++++++++++++++++++++++++++implement homing sequence++++++++++++++++++++++++++++++++++++++++
    }

    // By default, motor has no velocity
    void pwm_set_dc(int rotation=1, float _dc=0) {
        dc = _dc;
   
        uint32_t duty = (uint32_t)(dc*(65534.0/100.0)); // Scale dc (0-100) to dc (0-65534)
        pwm_set_point = duty * (pwm_top + 1) / 65535; // Scale according to pwm_top that best suits the PWM's frequency

        if (rotation){
            pwm_set_chan_level(sliceA1, channelA1, pwm_set_point);
            pwm_set_chan_level(sliceA2, channelA2, 0);
            return;
        }
        pwm_set_chan_level(sliceA1, channelA1, 0);
        pwm_set_chan_level(sliceA2, channelA2, pwm_set_point);
    }

    // Changes the bools corresponding to the two motor positions
    void set_position(bool position=0) { // 0: default position i.e 00deg, 1: 90deg
        if (position) {
            goto00deg = false; // In case it was at 00deg be4(keeps position this way) turn that flag to false to avoid both flags being true(undesired behaviour)
            goto90deg = true; 
            return;
        }
        goto90deg = false;
        goto00deg = true;
    }

    // Calculates position, this function is meant to be called by the interruption callback
    void calcualte_position(bool position) { // 0: 00deg, 1: 90deg. Maybe use an enum for this in the future.
        if (position) {
            at00deg = false;
            at90deg = true;
            return;
        }
        at90deg = false;
        at00deg = true;
    }

    // Checks wether the DoF is at desired position or not
    void calculate_control(float dc=70) { //
        if (goto00deg && !at00deg) {
            pwm_set_dc(1, dc);
            return;
        } else if (goto90deg && !at90deg) {
            pwm_set_dc(0, dc);
            return;
        }
        pwm_set_dc(0, 0);
    }

    uint8_t get_pin00deg() {return pin00deg; }
    uint8_t get_pin90deg() {return pin90deg; }
    bool get_at00deg() {return at00deg; }
    bool get_at90deg() {return at90deg; }
    bool get_goto00deg() {return goto00deg; }
    bool get_goto90deg() {return goto90deg; }

private:
    // Set the GPIO ports and get the corresponding slices and channels
    void pwm_set_gpio() {
        // enable GPIO ports for PWM
        gpio_set_function(pinA1, GPIO_FUNC_PWM);
        gpio_set_function(pinA2, GPIO_FUNC_PWM);

        // get the PWM slice number from the GPIO (there are 8)
        sliceA1 = pwm_gpio_to_slice_num(pinA1);
        sliceA2 = pwm_gpio_to_slice_num(pinA2);

        // Get the pwm channel from the GPIO (there is channel A and B)
        channelA1 = pwm_gpio_to_channel(pinA1);
        channelA2 = pwm_gpio_to_channel(pinA2);
    }

    // Set freq by setting wrap point, making sure the max resolution is achieved
    void pwm_set_freq(float pwm_freq) {
        // The code in this section is taken from the internet
        // Set the frequency, making "top" as large as possible for maximum resolution.
        pwm_div = (uint32_t)(16 * clock_get_hz(clk_sys) / (uint32_t)pwm_freq);
        pwm_top = 1;
        // 65534 is the max value in order to get PWM DC = 100%
        for (;;) {
            // Try a few small prime factors to get close to the desired frequency.
            if (pwm_div >= 16 * 5 && pwm_div % 5 == 0 && pwm_top * 5 <= 65534) {
                pwm_div /= 5;
                pwm_top *= 5;
            } else if (pwm_div >= 16 * 3 && pwm_div % 3 == 0 && pwm_top * 3 <= 65534) {
                pwm_div /= 3;
                pwm_top *= 3;
            } else if (pwm_div >= 16 * 2 &&pwm_top * 2 <= 65534) {
                pwm_div /= 2;
                pwm_top *= 2;
            } else {
                break;
            }
        }

        pwm_set_wrap(sliceA1, pwm_top);
        pwm_set_wrap(sliceA2, pwm_top);
    }

    void switches_set_gpio() {
        // Enable gpios as inputs and enable pullup resistors
        gpio_init(pin00deg);
        gpio_set_dir(pin00deg, GPIO_IN);
        gpio_pull_up(pin00deg);

        gpio_init(pin90deg);
        gpio_set_dir(pin90deg, GPIO_IN);
        gpio_pull_up(pin90deg);

        // Only one _with_callback, bc every gpio interruption is handled by the same callback
        gpio_set_irq_enabled_with_callback(pin00deg, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
        gpio_set_irq_enabled(pin90deg, GPIO_IRQ_EDGE_FALL, true);
    }
};

// Since the ammount of instructions equals 29, there is no more space for more programs per pio block
// Motors are not members of DifferentialRobot in order to keep some modularity and not a tree structure

Motor motors[] {Motor(13, 12, 10000, pio0, 8, 9, 1.0169579, true, 1.971858685524465, 0.089975732191985, 59),
                Motor(18, 19, 10000, pio1, 20, 21, 1.0203832, true, 1.780015430108977, 0.105431603635594, 57)}; // 0: right wheel, 1: left_wheel


MotorTwoPositions third_dof {MotorTwoPositions(14, 15, 10000, 22, 5)};

class DifferentialRobot{
private:
    float wheel_D; // [m] Wheel diameter, both left and right sould be the same
    float wheel_distance;

    float speed_lwheel {0.0}; // Linear velocity
    float speed_rwheel {0.0}; // Linear velocity

    float v {0.0}; // Linear speed in robot's reference frame
    float w {0.0}; // Angular speed

    float pose_sp[3] = {0.0, 0.0, 0.0}; // Xd, Yd, thetad/psisd
    float pose_start_new_sp[3] = {0.0, 0.0, 0.0};

    P_PositionOrientation_Controller P_controller;

public:
    float pose[3] = {0.0, 0.0, 0.0}; // X, Y, theta/psis
    float pose_dot[3] = {0.0, 0.0, 0.0}; // Xdot, Ydot, thetadot/psisdot

    DifferentialRobot(float _wheel_D, float _wheel_distance)
        : wheel_D{_wheel_D/1000}, wheel_distance{_wheel_distance/1000} {
        P_controller = P_PositionOrientation_Controller(0.5, 0.8, 1, 0, 20, 1, 1*M_PI/180);
    }

    void calculate_position_velocity(int64_t delta_time, int64_t delta_enc_counts[2]){
        // Keep in mind that delta_time is in us

        // I'm taking the velocity reading from the motors object, i.e: the filtered speed
        speed_lwheel = motors[1].speed_f[0]; // [rpm]
        speed_rwheel = motors[0].speed_f[0]; // [rpm]

        // Robot's speeds calculation
        v = ((- speed_rwheel + speed_lwheel)/2)*(2.0*M_PI)*(1/60.0)*(wheel_D/2.0)*1000.0; // Wheel speed is in rpm, need it in mm/s
        w = ((wheel_D/2)*(- speed_rwheel - speed_lwheel)/(wheel_distance))*(2.0*M_PI)*(1/60.0); // Angular speed is in rpm, need it in rad/s

        // Positions (estimation based on velocity. Find a better way of doing this)
        pose[2] += ((w+pose_dot[2])/2)*(delta_time/(1.0e6)); // [rad] Uses the average value of k and k-1
        pose[0] += v*cos(pose[2])*(delta_time/(1.0e6)); // [mm]
        pose[1] += v*sin(pose[2])*(delta_time/(1.0e6)); // [mm]

        // Speeds
        pose_dot[0] = v*cos(pose[2]); // [mm/s]
        pose_dot[1] = v*sin(pose[2]); // [mm/s]
        pose_dot[2] = w; // [rad/s]

        // printf("%lf %lf %lf %lf %lf %f %f %f %f %f\n", v, w*180/M_PI, pose[0], pose[1], pose[2]*180/M_PI, pose_dot[0], pose_dot[1], pose_dot[2], speed_lwheel, speed_rwheel);
    }

    void calculate_control(){
        float c_speeds_robot[2];

        // Last roobt pose is set as current desired position plus new coordinate (one DoF at a time, and SPs for desired trayectory have to be fed to controller)
        float pose_sp_cummulative[3] {pose_sp[0]+pose_start_new_sp[0],
                              pose_sp[1]+pose_start_new_sp[1],
                              pose_sp[2]+pose_start_new_sp[2]};

        // float pose_sp_rel[3] {pose_sp[0]-pose_start_new_sp[0],
        //                       pose_sp[1]-pose_start_new_sp[1],
        //                       pose_sp[2]-pose_start_new_sp[2]};

        // float pose_rel[3] {pose[0]-pose_start_new_sp[0],
        //                    pose[1]-pose_start_new_sp[1],
        //                    pose[2]-pose_start_new_sp[2]};

        // Controller is fed relative coordinates with respect to last position b4 changing the desired pose
        // this simplifies the control problem, though accumulates error over time instead of correcting it
        // in order to fix the problem exposed above two factors could be improved: 1st, implement a more
        // precise method for estimating the robot's pose and 2nd, implement a more sofisticated control
        // method.
        P_controller.calculate_control(pose_sp_cummulative, pose, c_speeds_robot);

        float c_speeds_wheels[2];
        float A = (M_PI/60.0)*(wheel_D/2)*1000;
        float B = (2*M_PI/60.0)*(wheel_D/2)*(1/wheel_distance);

        c_speeds_wheels[0] = (1/2.0)*((1/A)*c_speeds_robot[0] - (1/B)*c_speeds_robot[1]); // Left wheel
        c_speeds_wheels[1] = -(1/2.0)*((1/A)*c_speeds_robot[0] + (1/B)*c_speeds_robot[1]); // Right wheel

        motors[0].set_speed(c_speeds_wheels[1]);
        motors[1].set_speed(c_speeds_wheels[0]);
    }

    double get_linear_speed(){return v; }
    double get_angular_speed(){return w; }

    void set_pose(float* _pose){
        pose_sp[0] = _pose[0];
        pose_sp[1] = _pose[1];
        pose_sp[2] = _pose[2];

        // Take snapshot of controller pose
        pose_start_new_sp[0] = pose[0];
        pose_start_new_sp[1] = pose[1];
        pose_start_new_sp[2] = pose[2];

        P_controller.update_state_exit_idle();
    }
};

DifferentialRobot DDV1 = DifferentialRobot(31.4, 117);

// Funtion handler for timer(low_level) callbacks
bool low_level_sampling(struct repeating_timer *t) {
    // Calculate change in time
    static uint64_t last_time {to_us_since_boot(get_absolute_time())};
    uint64_t temp_last_time {to_us_since_boot(get_absolute_time())}; // Variable that avoids calling to_us_since_boot(get_absolute_time()) twice
    uint64_t delta_time {temp_last_time - last_time}; // [us]
    last_time = temp_last_time;

    for (auto &motor: motors){
        motor.update_enc_count(); // Fetch encoder count per wheel from PIO block
        motor.calculate_speed(delta_time);
        motor.calculate_control();
    }
    third_dof.calculate_control();

    // printf("%d %d %d %d\n", third_dof.get_at00deg(), third_dof.get_at90deg(), third_dof.get_goto00deg(), third_dof.get_goto90deg());
    // printf("%lf %lf %lf %lf %lf %lf %f %ld\n", motors[1].speed_f[0], motors[1].get_speed(), motors[1].get_dc(), motors[0].speed_f[0], motors[0].get_speed(), motors[0].get_dc(), battery.get_voltage(), delta_time);
    return true;
}

// Funtion handler for timer(high_level) callbacks
bool high_level_sampling(struct repeating_timer *t){
    // Calculate change in time
    static uint64_t last_time_ {to_us_since_boot(get_absolute_time())};
    uint64_t current_time {to_us_since_boot(get_absolute_time())}; // Variable that avoids calling to_us_since_boot(get_absolute_time()) twice
    uint64_t delta_time {current_time - last_time_}; // [us]
    last_time_ = current_time;

    // Calculate change in encoder counts
    static int64_t last_enc_counts[2] {motors[0].enc_count, motors[1].enc_count}; // Don't like this way, because this function depends on whether motors array was created or not (global var)
    int64_t current_enc_counts[2] {motors[0].enc_count, motors[1].enc_count};
    int64_t delta_enc_counts[2] {current_enc_counts[0] - last_enc_counts[0], current_enc_counts[1] - last_enc_counts[1]};
    last_enc_counts[0] = delta_enc_counts[0];
    last_enc_counts[1] = delta_enc_counts[1];

    DDV1.calculate_position_velocity(delta_time, delta_enc_counts);
    DDV1.calculate_control();

    return true;
}

// In the variable events, the type of interrupt is passed to the funtion, I will not use it
// This way of handling interruptions forces spaguetti code, and I hate it, but there is no other way, thanks Raspberry Foundation
void gpio_callback(uint gpio, uint32_t events) {
    static uint32_t last_time = to_ms_since_boot(get_absolute_time());
    uint64_t temp_last_time {to_us_since_boot(get_absolute_time())}; // Variable that avoids calling to_us_since_boot(get_absolute_time()) twice
    
    if(!(temp_last_time-last_time>200)) {return; } // Guard clause to debounce switches

    bool position;
    if(gpio==third_dof.get_pin00deg()) {
        position = 0; // At 00deg
    }
    else if(gpio==third_dof.get_pin90deg()) {
        position = 1; // At 90deg
    }

    third_dof.calcualte_position(position); // Get position into third dof
    last_time = temp_last_time;
}

void test_motor_performance(int delay) {
    for (float speed_sp {55}; speed_sp <= 90; speed_sp+=2) { // Max 91
        motors[0].pwm_set_dc(0, speed_sp);
        motors[1].pwm_set_dc(1, speed_sp);
        busy_wait_ms(delay);
    }
}

void test_motor_controllers_performance(int delay) {
    for (float speed_sp {10.0}; speed_sp < 71; speed_sp+=2) { // Max 91
        motors[0].set_speed(-speed_sp); // Right wheel, negative symbol in order for it to go straight
        motors[1].set_speed(speed_sp); // Left wheel, positive symbol in order for it to go straight
        busy_wait_ms(delay);
    }
}

void test_third_dof(int delay) {
    third_dof.set_position(); // 0(00deg) by default: home position
    busy_wait_ms(delay);
    third_dof.set_position(1);
    busy_wait_ms(delay);
}

void test_pose_controller(float des_x, float des_y, float des_th_deg) {    
    float pose[] {des_x, des_y, (float)(des_th_deg*M_PI/180)};
    DDV1.set_pose(pose);
}

int main(){
    set_sys_clock_khz(250000, false); // Overclock the board (makes sure the low level code executes extactly on time)

    stdio_init_all(); // Starts usb or uart (this is decided in the CMakeLists.txt file)
    int delay {5000};

    // Motor control loop setup
    struct repeating_timer timer1;
    add_repeating_timer_ms(-Ts, low_level_sampling, NULL, &timer1);
    busy_wait_ms(500); // For some reason there has to be some time between each timer in order for the posiotion calculation to not integrate any error produced in the first cycles after the first time has been set

    // Trajectory control loop setup
    struct repeating_timer timer2;
    add_repeating_timer_ms(Ts_h, high_level_sampling, NULL, &timer2);
    busy_wait_ms(500);

    busy_wait_ms(5000); // Some time to be able to connect to pc

    test_pose_controller(100, 0, 0);
    busy_wait_ms(10000);

    test_pose_controller(0, 0, 90);
    busy_wait_ms(10000);

    test_pose_controller(0, 100, 0);
    busy_wait_ms(10000);

    while(true){    
        tight_loop_contents();
    }
}
