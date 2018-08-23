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
    protected DcMotor leftLift = null;
    protected DcMotor rightLift = null;
    protected DcMotor rightBelt = null;
    protected DcMotor leftBelt = null;
    protected DcMotor windTurbine = null;

    protected Servo   solarGrabber  = null; //  3 Servos in use
    protected Servo   solarLift     = null;
    protected Servo   rubberCoiler  = null;
    //protected Servo   leftIntake    = null;
    //protected Servo   rightIntake   = null;
    protected Servo  rightPushWheel = null;
    protected Servo  leftPushWheel  = null;


    // Define servo positions
    protected final static double CONTINUOUS_SERVO_STOP = 0.5;     // These 3 servo pos are made for a continuous servo
    protected final static double CONTINUOUS_SERVO_CLOCKWISE = 0.8;
    protected final static double CONTINUOUS_SERVO_ANTI_CLOCKWISE = 0.2;

    // Define Intake speed
    protected final static double INTAKE_POWER =0.5;
    protected final static double INTAKE_FULL_POWER = 0.9;
    protected final static double INTAKE_OFF = 0.0;

    //Other
    protected boolean correctPositon;
    protected double position;

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
        leftLift = hwMap.get(DcMotor.class, "left_lift");
        rightLift = hwMap.get(DcMotor.class, "right_lift");
        rightBelt = hwMap.get(DcMotor.class, "right_belt");
        leftBelt = hwMap.get(DcMotor.class, "left_belt");
        windTurbine = hwMap.get(DcMotor.class, "wind_turbine");
        //pushingWall = hwMap.get(DcMotor.class, "pushing_wall");

        // Define Motor Directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Positive input should drive the robot forward
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftIntake.setDirection(DcMotor.Direction.FORWARD);   // Positive input should suck in the cubes :D
        //rightIntake.setDirection(DcMotor.Direction.REVERSE);

        //rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);
        rightBelt.setPower(0);
        leftBelt.setPower(0);
        windTurbine.setPower(0);
        //pushingWall.setPower(0);

        // Define and initialize ALL installed servos.
        solarGrabber = hwMap.get(Servo.class, "solar_grabber");
        //leftIntake = hwMap.get(Servo.class, "left_intake");
        //rightIntake = hwMap.get(Servo.class, "right_intake");
        solarLift    = hwMap.get(Servo.class, "solar_lift");
        rightPushWheel  = hwMap.get(Servo.class, "right_push");
        leftPushWheel  = hwMap.get(Servo.class, "left_push");

        // MAYBE init servo positions
        solarGrabber.setPosition(CONTINUOUS_SERVO_STOP);
        solarLift.setPosition(0);
        //rightIntake.setPosition(CONTINUOUS_SERVO_STOP);
        //leftIntake.setPosition(CONTINUOUS_SERVO_STOP);
        leftPushWheel.setPosition(CONTINUOUS_SERVO_STOP);
        rightPushWheel.setPosition(CONTINUOUS_SERVO_STOP);

        //rubberCoiler.setPosition(CONTINUOUS_SERVO_STOP);

    }
}
