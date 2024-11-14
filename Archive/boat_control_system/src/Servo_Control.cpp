// servos.cpp

#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <thread>
#include <chrono>

class Servos {
public:
    // Constructor to initialize servos for rudder and sail
    Servos(int rudder_pin = 16, int sail_pin = 18) 
        : rudder_pin_(rudder_pin), sail_pin_(sail_pin) {
        // Initialize GPIO mode and PWM settings
        wiringPiSetupGpio();  // Use BCM GPIO numbering
        pinMode(rudder_pin_, OUTPUT);
        pinMode(sail_pin_, OUTPUT);

        // Initialize software PWM (50 Hz frequency)
        softPwmCreate(rudder_pin_, 0, 200);  // 200 to match 20 ms period (50 Hz)
        softPwmCreate(sail_pin_, 0, 200);
    }

    // Method to move the servo to a specific angle
    void move_servo(int pin, int angle) {
        int duty_cycle = 5 + (angle / 18);  // Convert angle (0-180) to duty cycle
        softPwmWrite(pin, duty_cycle);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Adjust timing as needed
        softPwmWrite(pin, 0);  // Turn off PWM signal after movement
    }

    // Method to set the rudder angle
    void set_rudder_angle(int angle) {
        if (angle >= 0 && angle <= 180) {
            move_servo(rudder_pin_, angle);
        } else {
            std::cerr << "Invalid rudder angle. Must be between 0 and 180.\n";
        }
    }

    // Method to set the sail angle
    void set_sail_angle(int angle) {
        if (angle >= 0 && angle <= 90) {  // Assuming sail angle is limited to 90 degrees
            move_servo(sail_pin_, angle);
        } else {
            std::cerr << "Invalid sail angle. Must be between 0 and 90.\n";
        }
    }

    // Cleanup method to stop PWM and reset GPIO
    void cleanup() {
        softPwmWrite(rudder_pin_, 0);
        softPwmWrite(sail_pin_, 0);
        digitalWrite(rudder_pin_, LOW);
        digitalWrite(sail_pin_, LOW);
    }

private:
    int rudder_pin_;
    int sail_pin_;
};