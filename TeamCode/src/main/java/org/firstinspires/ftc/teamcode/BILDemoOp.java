/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="BIL: Demo Teleop", group="BIL")
public class BILDemoOp extends OpMode {

    BILRobotHardware robot = new BILRobotHardware(); // use the class created to define a Pushbot's hardware

    double frontRight;
    double frontLeft;
    double backRight;
    double backLeft;

    double throttleY;
    double throttleX;
    double turning;
    double liftSpeed;

    boolean directionRobot = true;
    boolean liftDeployed = false;
    double maxSpeed = 0.7;
    BILTeleOpJoystick bilTeleOpJoystick;
    /**
     * Constructor
     */
    public BILDemoOp() {
        bilTeleOpJoystick = new BILTeleOpJoystick();
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        //Initializes all robot hardware parts
        robot.init(hardwareMap);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        getJoystickInput();

        scaleJoystickInput();

        setMotorSpeeds();

        setPusher();

        getLiftDeployed();

        //telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("F/R", "direction:  " + String.format("%b",directionRobot ));
        telemetry.addData("FrontLeft Power", String.format("%.2f", frontLeft));
        telemetry.addData("BackLeft Power", String.format("%.2f", backLeft));
        telemetry.addData("FrontRight Power", String.format("%.2f", frontRight));
        telemetry.addData("BackRight Power", String.format("%.2f", backRight));
        telemetry.update();
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    protected void getJoystickInput() {
        // throttleY: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // throttleX: left_stick_x ranges from -1 to 1, where -1 is full left, and
        // 1 is full right
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        throttleY = -gamepad2.left_stick_y;
        throttleX = gamepad2.left_stick_x;
        turning = gamepad2.right_stick_x;

        if(Math.abs(gamepad1.left_stick_x) > 5 || Math.abs(gamepad1.left_stick_y) > 5 || Math.abs(gamepad1.right_stick_x) > 5 || gamepad1.y) {
            throttleY = -gamepad1.left_stick_y;
            throttleX = gamepad1.left_stick_x;
            turning = gamepad1.right_stick_x;
        }

        if (liftDeployed) {
            liftSpeed = gamepad1.left_stick_y;
        } else {
            liftSpeed = 0;
        }
    }

    protected void scaleJoystickInput() {
        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        throttleY = bilTeleOpJoystick.normalizeSpeed(throttleY, 2.0, maxSpeed);
        throttleX = bilTeleOpJoystick.normalizeSpeed(throttleX, 2.0, maxSpeed);
        turning = bilTeleOpJoystick.normalizeSpeed(turning, 2.0, maxSpeed);
        liftSpeed = bilTeleOpJoystick.normalizeSpeed(liftSpeed, 2.0, maxSpeed)/2;

        if(gamepad1.a){
            liftSpeed *= 2;
        }
    }

    protected void getMeccanumMotorSpeeds(double leftX, double leftY, double rightX) {
        frontRight = leftY - leftX - rightX;
        backRight = leftY + leftX - rightX;
        frontLeft = leftY + leftX + rightX;
        backLeft = leftY - leftX + rightX;

        frontRight = Range.clip(frontRight, -maxSpeed, maxSpeed);
        backRight = Range.clip(backRight, -maxSpeed, maxSpeed);
        frontLeft = Range.clip(frontLeft, -maxSpeed, maxSpeed);
        backLeft = Range.clip(backLeft, -maxSpeed, maxSpeed);
    }

    protected void setMotorSpeeds() {
        getMeccanumMotorSpeeds(throttleX, throttleY, turning);

        // write the values to the motors
        robot.motorFrontLeft.setPower(frontLeft);
        robot.motorBackLeft.setPower(backLeft);
        robot.motorFrontRight.setPower(frontRight);
        robot.motorBackRight.setPower(backRight);
        robot.motorLift.setPower(liftSpeed);
    }

    protected void setPusher() {
        double pusherPosition = robot.pusherMiddle;
        if(gamepad1.right_trigger > 0.5) {
            pusherPosition = robot.pusherRight;
        } else if(gamepad1.left_trigger > 0.5) {
            pusherPosition = robot.pusherLeft;
        }

        robot.pusher.setPosition(pusherPosition);
    }

    protected void getLiftDeployed() {
        if(!liftDeployed) {
            if(gamepad1.x && gamepad1.b) {
                liftDeployed = true;
                robot.liftHolder.setPosition(robot.liftHolderRelease);
            }
        }
    }
}