/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "PinNames.h"
#include <cstdint>
#include <cstdlib>


#include "mbed.h"
#include <Adafruit_PWMServoDriver.h>

#include "3DofWalkerLib.h"

// Blinking rate in milliseconds
#define BLINKING_RATE   500ms
#define SERVO_FREQ      50 // Analog servos run at ~50 Hz updates
#define UP              0
#define DOWN            1
#define MAX        130
#define MIN        60
#define MAXSPEED      25

int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    DigitalIn SW1(D2);
    SW1.mode(PullUp);

    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PB_7, PB_6, 0x80);


    int control = 0;
    bool direction = 0;
    uint16_t error_code = 0;

    led.write(0);
    pwm.begin();
    wait_us(100000);
    //pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    wait_us(100000);

    Servo J1 = Servo(0, &pwm, 5, 1.0f);
    Servo J2 = Servo(1, &pwm, -5, 1.0f);
    Servo J3 = Servo(2, &pwm, 155, -1.0f);
    //Servo J3 = Servo(2, &pwm, 90, -1.0f);

    Servo J4 = Servo(3, &pwm, 192, -1.0f);
    Servo J5 = Servo(4, &pwm, 180, -1.0f);
    Servo J6 = Servo(5, &pwm, 23, 1.0f);

    Servo J7 = Servo(6, &pwm, 0, 1.0f);
    Servo J8 = Servo(7, &pwm, 175, -1.0f);
    Servo J9 = Servo(8, &pwm, 25, 1.0f);

    Servo J10 = Servo(9, &pwm, 180, -1.0f);
    Servo J11 = Servo(10, &pwm, -5, 1.0f);
    Servo J12 = Servo(11, &pwm, 145, -1.0f);

    Scara scara1 = Scara(&J2,  &J3,  80, 80);
    Scara scara2 = Scara(&J5,  &J6,  80, 80);
    Scara scara3 = Scara(&J8,  &J9,  80, 80);
    Scara scara4 = Scara(&J11, &J12, 80, 80);


    Leg leg1 = Leg(&J1, &scara1);
    Leg leg2 = Leg(&J4, &scara2);
    Leg leg3 = Leg(&J7, &scara3);
    Leg leg4 = Leg(&J10, &scara4);

    vector3D home = {-90, 0, 60};

    //Path* pathArray = (Path*)malloc(4 * sizeof(Path));
//
    //pathArray[0] = Path(&leg1, home);
    //pathArray[1] = Path(&leg2, home);
    //pathArray[2] = Path(&leg3, home);
    //pathArray[3] = Path(&leg4, home);
    Path path1 = Path(&leg1, home);
    Path path2 = Path(&leg2, home);
    Path path3 = Path(&leg3, home);
    Path path4 = Path(&leg4, home);
//
    path1.setShitfZ(10.0f);
    path2.setShitfZ(10.0f);
    path3.setShitfZ(10.0f);
    path4.setShitfZ(10.0f);
//
    path1.setShitfX(35.0f);  // read legs negative towards front
    path2.setShitfX(35.0f);
    path3.setShitfX(0.0f);  // front legs negative towards front 
    path4.setShitfX(0.0f);

    path1.setShitfY(-30.0f);
    path2.setShitfY(-30.0f);
    path3.setShitfY(-30.0f);
    path4.setShitfY(-30.0f);
    
    /*
    while (true) {
        J1.move(90);
        J2.move(90);
        J3.move(0);
        J4.move(90);
        J5.move(90);
        J6.move(0);
        J7.move(90);
        J8.move(90);
        J9.move(0);
        J10.move(90);
        J11.move(90);
        J12.move(0);
    }
    */

    
    vector3D pose1;
    pose1.x = 0;
    pose1.y = -20;
    pose1.z = MIN;
    direction = UP;
    control = MIN;

    leg1.move(pose1);
    leg2.move(pose1);
    leg3.move(pose1);
    leg4.move(pose1);

    printf("Ready...\n");

    while(SW1.read() == 1) wait_us(200000);
    while(SW1.read() == 0) wait_us(200000);

    wait_us(2000000); //2 seconds wait

    while(control < MAX) {
        direction ? control-- : control++;

        wait_us(2000);

        pose1.z = control;

        leg1.move(pose1);
        leg2.move(pose1);
        leg3.move(pose1);
        leg4.move(pose1);
    }

    wait_us(2000000); //2 seconds wait

    while(control > MAX - 40) {
        control--;

        wait_us(2000);

        pose1.z = control;

        leg1.move(pose1);
        leg2.move(pose1);
        leg3.move(pose1);
        leg4.move(pose1);
    }
    
    wait_us(200000); //0.1 seconds wait

    //jump in running pose

    pose1.x = 0;
    pose1.y = -20;
    pose1.z = MAX;

    leg1.move(pose1);
    leg2.move(pose1);
    leg3.move(pose1);
    leg4.move(pose1);

    wait_us(200000); //0.2 seconds wait

    control = 0;

    direction = UP;
        path1.update(control+250);
        path2.update(control+750);
        path3.update(control+500);
        path4.update(control);

    wait_us(200000); //0.2 seconds wait

    int speed = 15;

    while (control < 8000) {

        path1.incrementalUpdate(speed);
        path2.incrementalUpdate(speed);
        path3.incrementalUpdate(speed);
        control = path4.incrementalUpdate(speed);

        wait_us(20000); //50 Hz

    }

    // final rest pose
    pose1.x = 0;
    pose1.y = -20;
    pose1.z = MIN;
    direction = UP;
    control = MIN;

    leg1.move(pose1);
    leg2.move(pose1);
    leg3.move(pose1);
    leg4.move(pose1);

    printf("DONE\n");

    while(true){

        led.write(direction);
        direction = 1 - direction;
        wait_us(100000);
    }
}
