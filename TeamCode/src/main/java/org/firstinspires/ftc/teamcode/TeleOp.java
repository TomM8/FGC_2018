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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Git Bash stuff:
 * First go to the repository
 * Then initialize git (git init)
 * Then add (git add .)
 * Then commit (git commit -m "")-
 * Then push to master (git push origin master) --> not necessary
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative Opmode")
//Disabled
public class TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Basic_Bot robo = new Basic_Bot();
    private double beltLeftLiftPower;
    private double beltRightliftPower;

    int toggle = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robo.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // region Driving code BELOW (single stick)     (gp1)

        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        double leftRubbPower = Range.clip(drive - turn, -0.6, 0.6);    // driving powers defined and set
        double rightRubbPower = Range.clip(drive + turn, -0.6, 0.6);

        double leftRubbSlowPower = Range.clip(drive - turn, -0.3, 0.3);    // driving powers defined and set (slower)
        double rightRubbSlowPower = Range.clip(drive + turn, -0.3, 0.3);

        //robo.leftDrive.setPower(leftRubbPower);
        //robo.rightDrive.setPower(rightRubbPower);

        if(!gamepad1.left_stick_button){
            robo.leftDrive.setPower(leftRubbPower);
            robo.rightDrive.setPower(rightRubbPower);
        }
        else if(gamepad1.left_stick_button){
            robo.leftDrive.setPower(leftRubbSlowPower);
            robo.rightDrive.setPower(rightRubbSlowPower);
        }
        else {
            robo.leftDrive.setPower(0);
            robo.rightDrive.setPower(0);
        }

        // endregion

        // region Lifting motors              (gp2)

        //double liftMotorsPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
        //double rightLiftPosition = robo.rightLift.getCurrentPosition();
        //double leftLiftPosition = robo.leftLift.getCurrentPosition();

        if(gamepad2.dpad_up){
            robo.rightLift.setPower(-1.0);
            robo.leftLift.setPower(1.0);
        }
        else if(gamepad2.dpad_down){
            robo.rightLift.setPower(0.7);
            robo.leftLift.setPower(-0.7);
        }
        else {
            robo.rightLift.setPower(0.0);
            robo.leftLift.setPower(0.0);
        }

        //endregion

        //region Solar servos             (gp1)

        //Servo arm

        double servoArmPosition = robo.solarLift.getPosition();

        if(gamepad1.b){
            robo.solarLift.setPosition(0.55);
        }
        else if(gamepad1.y){
            robo.solarLift.setPosition(0.0);
        }
        else if(gamepad1.x){
            robo.solarLift.setPosition(0.45);
        }
        else {
            robo.solarLift.setPosition(0.27);
        }

        /*if(servoArmPosition != 1.0){
            robo.solarLift.setPosition(servoArmPosition + 1);
            robo.correctPositon = true;
        }
        else {
            robo.solarLift.setPosition(servoArmPosition - 1);
            robo.correctPositon = false;
        }

        double servoArmPosition2 = robo.solarLift.getPosition();

        if(gamepad1.b && robo.correctPositon == true){
            robo.solarLift.setPosition(servoArmPosition2 + 1);
        }*/

        //Servo grabbers

        if(gamepad1.right_bumper){
            robo.solarGrabber.setPosition(robo.CONTINUOUS_SERVO_ANTI_CLOCKWISE);
        }
        else if(gamepad1.left_bumper){
            robo.solarGrabber.setPosition(robo.CONTINUOUS_SERVO_CLOCKWISE);
        }
        else {
            robo.solarGrabber.setPosition(robo.CONTINUOUS_SERVO_STOP);
        }

        //endregion

        //region Intake servos       (gp2)

        /*if(gamepad2.left_bumper){
            robo.rightIntake.setPosition(robo.CONTINUOUS_SERVO_ANTI_CLOCKWISE);
            robo.leftIntake.setPosition(robo.CONTINUOUS_SERVO_CLOCKWISE);
        }
        else if(gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0){
            robo.rightIntake.setPosition(robo.CONTINUOUS_SERVO_CLOCKWISE);
            robo.leftIntake.setPosition(robo.CONTINUOUS_SERVO_ANTI_CLOCKWISE);
        }
        else {
            robo.rightIntake.setPosition(robo.CONTINUOUS_SERVO_STOP);
            robo.leftIntake.setPosition(robo.CONTINUOUS_SERVO_STOP);
        }*/

        //endregion

        //region Intake belts       (gp2)

        //double beltLeftLiftPower = 0;
        //double beltRightliftPower = 0;


        if(gamepad2.right_bumper){
            robo.leftBelt.setPower(-robo.INTAKE_FULL_POWER);
            robo.rightBelt.setPower(robo.INTAKE_FULL_POWER);
        }
        /*else if(gamepad2.right_trigger < 0){
            beltLeftLiftPower = gamepad2.right_trigger;
            beltRightliftPower = -gamepad2.right_trigger;

            robo.rightBelt.setPower(beltRightliftPower);
            robo.leftBelt.setPower(beltLeftLiftPower);

        }*/
        else if(gamepad2.a){
            robo.leftBelt.setPower(robo.INTAKE_FULL_POWER);
            robo.rightBelt.setPower(-robo.INTAKE_FULL_POWER);
        }
        else {
            robo.leftBelt.setPower(robo.INTAKE_OFF);
            robo.rightBelt.setPower(robo.INTAKE_OFF);
        }

        // endregion

        //region Pusher wheels           (gp2)

        if(gamepad2.y){
            robo.rightPushWheel.setPosition(0.0);
            robo.leftPushWheel.setPosition(1.0);
        }
        else if(gamepad2.x){
            robo.rightPushWheel.setPosition(1.0);
            robo.leftPushWheel.setPosition(0.0);
        }
        else {
            robo.rightPushWheel.setPosition(robo.CONTINUOUS_SERVO_STOP);
            robo.leftPushWheel.setPosition(robo.CONTINUOUS_SERVO_STOP);
        }

        //endregion

        //region Wind turbine      (gp1)

        if(gamepad1.a){
            robo.windTurbine.setPower(1.0);
        }
        else {
            robo.windTurbine.setPower(0.0);
        }

        //endregion

        //region Telemetry

        telemetry.addData("solar_servo position: ", servoArmPosition);
        //telemetry.addData("right_lift position: ", rightLiftPosition);
        //telemetry.addData("left_lift position: ", leftLiftPosition);


        //endregion



    /*
     * Code to run ONCE after the driver hits STOP
     */
    }
    @Override
    public void stop() {
    }

    private boolean servoIsBusy(double currentPos, double targetPos) { // used to play commands ONCE servo is in place
        boolean reachedPosition;

        if(currentPos != targetPos){
            reachedPosition = false;
        } else{
            reachedPosition = true;
        }
        return reachedPosition;
    } // used to play commands ONCE servo is in place



    /*
     * Code below makes the robot move in a right-angled triangle
     */
    private void triangularMotion (double speed, double backingDistanceCm, double turnAngle) {} // Not done

    /*
     * Code below uses the triangular motion method and loops it in a way to create a horizontal motion
     */
    private void strafe (double speed, double backingDistanceCm, double turnAngle){} // not done
}
