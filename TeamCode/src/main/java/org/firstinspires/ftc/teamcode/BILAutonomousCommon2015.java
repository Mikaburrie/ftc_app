package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Mika/Alex on 12/12/2015.
 */
public abstract class BILAutonomousCommon2015 extends OpMode {

    DcMotorController wheelController;
    DcMotor motorRight;
    DcMotor motorLeft;

    UltrasonicSensor distanceSensor;
    GyroSensor gyroSensor;
    LightSensor lightSensor;

    DcMotor claw;

    SpeedContainer speedContainer;

    int encoderTicks;
    protected double speed = 0.5;
    protected double gyroAdjustmentMultiplier = 0.01;
    static final double circum = (47 / 8) * Math.PI; //Diameter times pi equals circumference

    @Override
    public void init() {
        //motors and controllers
        wheelController = hardwareMap.dcMotorController.get("motor control");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        //sight sensors
        distanceSensor = hardwareMap.ultrasonicSensor.get("ultrasonic_1");
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        //gyro sensor and basic values
        gyroSensor = hardwareMap.gyroSensor.get("gyro_1");
        int xVal, yVal, zVal = 0;
        int heading = 0;
        //claw = hardwareMap.servo.get("servo_1");
        gyroSensor.calibrate();
        speedContainer = new SpeedContainer();

    }

    //
    // setForwardDriveDistance
    //

    /**
     * Set wheel encoders to run a certain distance
     */
    public void setForwardDriveDistance(double driveDistance, double circumference)

    {
        encoderTicks = (int) (1440 * (driveDistance * 12 / circumference)); //turns drive distance to inches. Divides by circumference by drive distance to get rotations. Multiply rotations by 1440 ticks to get motor ticks.


    } // setForwardDriveDistance

    /**
     * @return distance in centimeters
     */
    public double getDistance() {
        return distanceSensor.getUltrasonicLevel();
    }

    public boolean haveDriveEncodersReached() {
        boolean haveDriveEncodersReached = false;
        logTelemetry();
        haveDriveEncodersReached = this.haveDriveEncodersReached(encoderTicks, encoderTicks);
        telemetry.addData("have drive encoders reached", haveDriveEncodersReached);
        telemetry.addData("encoder ticks", encoderTicks);
        return haveDriveEncodersReached;
    }

    protected void logTelemetry() {
        telemetry.addData("Light Sensor", lightSensor.toString());
        telemetry.addData("Gyro Heading", String.format("%03d", gyroSensor.getHeading()));
        telemetry.addData("left motor", motorLeft.getCurrentPosition());
        telemetry.addData("right motor", motorRight.getCurrentPosition());
        telemetry.addData("RunMode: ", motorLeft.getMode().toString());
    }

    //--------------------------------------------------------------------------
    //
    // haveDriveEncodersReached
    //

    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean haveDriveEncodersReached(double pLeftCount, double pRightCount)

    {
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (hasDriveEncoderReached(motorRight, pLeftCount) && hasDriveEncoderReached(motorLeft, pRightCount)) {
            l_return = true;
        }

        return l_return;

    } // haveDriveEncodersReached

    //--------------------------------------------------------------------------

    boolean hasDriveEncoderReached(DcMotor motorType, double p_count)

    {
        boolean lReturn = false;

        if (motorType != null) {
            //
            // Has the encoder reached the specified values?
            //
            telemetry.addData("motor left current position", motorType.getCurrentPosition());
            if (Math.abs(motorType.getCurrentPosition()) > p_count) {
                lReturn = true;
            }
        }

        return lReturn;

    } // hasLeftDriveEncoderReached

    //--------------------------------------------------------------------------


    boolean isLeftDriveEncoderZero()

    {
        boolean lReturn = false;

        if (motorLeft != null) {
            //
            // Has the encoder reached the specified values?
            //
            telemetry.addData("motor left current position", motorLeft.getCurrentPosition());
            if (Math.abs(motorLeft.getCurrentPosition()) < 0.1) {
                lReturn = true;
            }
        }

        return lReturn;

    }

    /**
     * Checks if drive encoder is zero,
     * telemetry string could be "motor right current position"
     * or "motor left current position"
     * @param motorType
     * @param telemetryString
     * @return
     */
    boolean isDriveEncoderZero(DcMotor motorType, String telemetryString)

    {
        boolean lReturn = false;

        if (motorType != null) {
            //
            // Has the encoder reached the specified values?
            //
            telemetry.addData(telemetryString, motorRight.getCurrentPosition());
            if (Math.abs(motorType.getCurrentPosition()) < 0.1) {
                lReturn = true;
            }
        }

        return lReturn;

    }

   /* boolean areDriveEncodersZero()

    {
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (isRightDriveEncoderZero() && isLeftDriveEncoderZero()) {
            l_return = true;
        }

        return l_return;

    }*/
    //--------------------------------------------------------------------------

    protected void driveForward(SpeedContainer speedContainer) {
        motorRight.setPower(speedContainer.getRightSpeed());
        motorLeft.setPower(speedContainer.getLeftSpeed());
    }

    protected void turnLeft(SpeedContainer speedContainer) {
        motorRight.setPower(0.0);
        motorLeft.setPower(speedContainer.getLeftSpeed());
    }

    protected void driveBackward(SpeedContainer speedContainer) {
        motorRight.setPower(-speedContainer.getRightSpeed());
        motorLeft.setPower(-speedContainer.getLeftSpeed());
    }

    protected void stopDriving() {
        motorRight.setPower(0.0);
        motorLeft.setPower(0.0);
    }

    protected void turnRight(SpeedContainer speedContainer) {
        motorRight.setPower(speedContainer.getRightSpeed());
        motorLeft.setPower(0.0);
    }

    protected void driveUpRamp(double speed) {
        motorRight.setPower(0.35);
        motorLeft.setPower(0.35);

    }

    //--------------------------------------------------------------------------

    //
    // resetDriveEncoders
    //

    /**
     * Reset both drive wheel encoders.
     */
    public void resetDriveEncoders()

    {
        resetDriveEncoder(motorRight);
        resetDriveEncoder(motorLeft);

    } // resetDriveEncoders

    //--------------------------------------------------------------------------


    /**
     * Reset the right drive wheel encoder.
     */
    public void resetDriveEncoder(DcMotor motorType)

    {

        if (motorType != null) {
            DcMotor.RunMode runMode = this.wheelController.getMotorMode(motorType.getPortNumber());
            motorType.setMode(runMode.RESET_ENCODERS);
        }

    } // resetRightDriveEncoder


    public void resetMotors() {

    }
    //--------------------------------------------------------------------------
    //
    // runUsingEncoders
    //

    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void runUsingEncoders()

    {
        runUsingDriveEncoder(motorRight);
        runUsingDriveEncoder(motorLeft);
    } // runUsingEncoders
    //--------------------------------------------------------------------------
    //
    // runUsingLeftDriveEncoder
    //

    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void runUsingDriveEncoder(DcMotor motorType)

    {
        if (motorType != null) {
            DcMotor.RunMode runMode = this.wheelController.getMotorMode(motorType.getPortNumber());
            motorType.setMode(runMode.RUN_USING_ENCODERS);
        }

    } //
    //--------------------------------------------------------------------------
    //


    //--------------------------------------------------------------------------
    //
    // runWithoutDriveEncoders
    //

    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void runWithoutDriveEncoders()

    {
        runWithoutDriveEncoder(motorRight);
        runWithoutDriveEncoder(motorLeft);

    } // runWithoutDriveEncoders
    //--------------------------------------------------------------------------
    //
    // runWithoutLeftDriveEncoder
    //

    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void runWithoutDriveEncoder(DcMotor motorType)

    {
        if (motorType != null) {
            DcMotor.RunMode runMode = this.wheelController.getMotorMode(motorType.getPortNumber());
            if (motorType.getMode() == runMode.RESET_ENCODERS) {

                motorType.setMode(runMode.RUN_WITHOUT_ENCODERS);
            }
        }

    } //
    //

    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */

    //calculates an adjusts the speed of the robot so it can readjust the gyro path
    protected void calculateSpeedBasedOnGyro()
    {
        int heading = gyroSensor.getHeading();
        if (heading > 180) //going left
        {
            heading = heading - 360;
        }

        double leftSpeed = speed - heading * gyroAdjustmentMultiplier;
        double rightSpeed = speed + heading * gyroAdjustmentMultiplier;
        leftSpeed = enforceSpeedLimitation(leftSpeed);
        rightSpeed = enforceSpeedLimitation(rightSpeed);

        speedContainer.setLeftSpeed(leftSpeed);
        speedContainer.setRightSpeed(rightSpeed);
    }

    //Protects the speed from going over 1.0 or going less than -1.0
    protected double enforceSpeedLimitation(double initialSpeed)
    {
        if (initialSpeed >= 0)
        {
            initialSpeed = Math.min(initialSpeed, 0.99);
        }
        else
        {
            initialSpeed = Math.max(initialSpeed, -0.99);
        }
        return initialSpeed;
    }
}