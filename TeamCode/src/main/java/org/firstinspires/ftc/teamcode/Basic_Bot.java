/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Basic_Bot
{
    /* Public OpMode members. */
    protected DcMotor leftDrive = null;  // 7 DC Motors in use
    protected DcMotor rightDrive = null;
    protected DcMotor leftIntake = null;
    protected DcMotor rightIntake = null;
    protected DcMotor leftLift = null;
    protected DcMotor rightLift = null;
    protected DcMotor pushingWall = null;

    protected Servo   solarGrabber  = null; //  3 Servos in use
    protected Servo   solarLift     = null;
    protected Servo   rubberCoiler  = null;


    // Define servo positions
    protected final static double CONTINUOUS_SERVO_STOP = 0.5;     // These 3 servo pos are made for a continuous servo
    protected final static double CONTINUOUS_SERVO_CLOCKWISE = 0.8;
    protected final static double CONTINUOUS_SERVO_ANTI_CLOCKWISE = 0.2;

    // Define Intake speed
    protected final static double INTAKE_POWER =0.5;
    protected final static double INTAKE_OFF = 0.0;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Basic_Bot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftIntake = hwMap.get(DcMotor.class, "left_intake");
        rightIntake = hwMap.get(DcMotor.class, "right_intake");
        leftLift = hwMap.get(DcMotor.class, "left_tilt");
        rightLift = hwMap.get(DcMotor.class, "right_tilt");
        pushingWall = hwMap.get(DcMotor.class, "pushing_wall");

        // Define Motor Directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Positive input should drive the robot forward
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);   // Positive input should suck in the cubes :D
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);
        pushingWall.setPower(0);

        // Define and initialize ALL installed servos.
        solarGrabber = hwMap.get(Servo.class, "solar_grabber");
        rubberCoiler = hwMap.get(Servo.class, "rubber_coiler");
        solarLift    = hwMap.get(Servo.class, "solar_lift");

        // MAYBE init servo positions
        solarGrabber.setPosition(CONTINUOUS_SERVO_STOP);
        solarLift.setPosition(CONTINUOUS_SERVO_STOP);
        rubberCoiler.setPosition(CONTINUOUS_SERVO_STOP);

        //Change from tom
        //Just some test changes :)
        //More test changes :):)
    }
}
