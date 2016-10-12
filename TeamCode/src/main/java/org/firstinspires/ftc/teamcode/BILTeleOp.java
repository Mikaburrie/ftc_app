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

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class BILTeleOp extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
	// TETRIX VALUES.

	DcMotor motorRight;
	DcMotor motorLeft;
	DcMotor motorClaw;
	Servo hookTurner;
	TouchSensor armInButton;
	TouchSensor armOutButton;
	UltrasonicSensor distanceSensor;

	boolean directionRobot = true;
	double hookPosition = 0.0;
	double low = 0.0;
	double middle = 0.5;
	double high = 0.9; // 1/12/15 See if the direction of servo movement can be changed
	float maxSpeed = 7/10; // 7/10 is a float, 0.7 is a double
	float maxArmSpeed = 1/2; // 1/2 is a float, 0.5 is a double
	int overdrive = 1;
	BILTeleOpJoystick bilTeleOpJoystick;
	/**
	 * Constructor
	 */
	public BILTeleOp() {
	bilTeleOpJoystick = new BILTeleOpJoystick();
	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		motorRight = hardwareMap.dcMotor.get("motor_2");
		motorLeft = hardwareMap.dcMotor.get("motor_1");

		motorClaw = hardwareMap.dcMotor.get("motor_3");
		//motorLeft.setDirection(DcMotor.Direction.REVERSE);

		hookTurner = hardwareMap.servo.get("servo_2");

		armInButton = hardwareMap.touchSensor.get("button_1");
		armOutButton = hardwareMap.touchSensor.get("button_2");

		distanceSensor = hardwareMap.ultrasonicSensor.get("ultrasonic_1");

		hookPosition = high;

		// assign the starting position of the wrist and claw

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

		// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.right_stick_x;
		float clawArmCommand = gamepad2.left_stick_y;


		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		throttle = (float)bilTeleOpJoystick.normalizeSpeed(throttle, 2.0, maxSpeed);
		direction =  (float)bilTeleOpJoystick.normalizeSpeed(direction, 2.0, maxSpeed);
		clawArmCommand = (float)bilTeleOpJoystick.normalizeSpeed(clawArmCommand, 2.0, maxSpeed);

		float right = throttle - direction;
		float left = throttle + direction;

		// clip the right/left values so that the values never exceed +/- whatever you set it to
		right = Range.clip(right, -maxSpeed, maxSpeed);
		left = Range.clip(left, -maxSpeed, maxSpeed);
		clawArmCommand = Range.clip(clawArmCommand, -maxArmSpeed, maxArmSpeed * overdrive);

		//checks for claw arm button and keeps string from snapping
		if(armInButton.isPressed() && -gamepad2.left_stick_y < 0){
			clawArmCommand = 0;
		}

		if(armOutButton.isPressed() && -gamepad2.left_stick_y > 0 ){
			clawArmCommand = 0;
		}

		// write the values to the motors
		motorRight.setPower(right);
		motorLeft.setPower(left);
		motorClaw.setPower(clawArmCommand);

		// update the position of the arm.
		if (gamepad2.dpad_left || gamepad2.dpad_right) {
			// if the A button is pushed on gamepad1, increment the position of
			// the arm servo.
			hookPosition = middle;
		}
		if (gamepad2.dpad_down) {
			// if the Y button is pushed on gamepad1, decrease the position of`
			// the arm servo.
			hookPosition = low;
		}
		if (gamepad2.dpad_up) {
			hookPosition = high;
		}

		directionRobot = gamepad1.right_trigger <= 0.90;

		if (gamepad2.right_trigger > 0.90) {
			overdrive = 2;
		}
		else {
			overdrive = 1;
		}

		if (!directionRobot) {
			motorRight.setDirection(DcMotor.Direction.REVERSE);
			motorLeft.setDirection(DcMotor.Direction.FORWARD);
		}
		else {
			motorRight.setDirection(DcMotor.Direction.FORWARD);
			motorLeft.setDirection(DcMotor.Direction.REVERSE);
		}
		// write position values to the wrist and claw servo
		hookTurner.setPosition(hookPosition);




		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("F/R", "direction:  " + String.format("%b",directionRobot ));
		telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
		telemetry.addData("Fully extend arm sensor", String.format("%b", armOutButton.isPressed()));
		telemetry.addData("Fully in arm sensor", String.format("%b", armInButton.isPressed()));
		telemetry.addData("Ultrasonic value(cm)", String.format("%.2f", (float)distanceSensor.getUltrasonicLevel()));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

}
