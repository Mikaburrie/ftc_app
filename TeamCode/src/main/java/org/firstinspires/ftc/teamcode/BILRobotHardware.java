package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mikab_000 on 10/20/2016.
 */
public class BILRobotHardware {
    /* Public OpMode members. */
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public LightSensor lightSensor;
    public GyroSensor gyroSensor;
    public ColorSensor colorSensor;
    public Servo pusher;
    public double pusherLeft = 0.16;
    public double pusherMiddle = 0.41;
    public double pusherRight = 0.66;
    public final static int ticksPerRotation = 1440;
    public final static double wheelCircumference = (4 * Math.PI)/12; //circumference in feet
    public final static int driveTimeScalar = 3;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public BILRobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Define and Initialize Motors
        //Left_Front, Left_Back, Right_Front, Right_Back
        motorFrontRight = hwMap.dcMotor.get("Right_Front");
        motorBackRight = hwMap.dcMotor.get("Right_Back");
        motorFrontLeft = hwMap.dcMotor.get("Left_Front");
        motorBackLeft = hwMap.dcMotor.get("Left_Back");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Servos
        pusher = hwMap.servo.get("pusher");

        //Initialize sensors
        lightSensor = hwMap.lightSensor.get("lightSensor");
        colorSensor = hwMap.colorSensor.get("colorSensor");

        //Initialize gyro and calibrate
        gyroSensor = hwMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();

        // Set all motors to zero power
        setAllDriveMotors(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param mode The run mode to set for all motors.
     */
    public void setAllMotorModes(DcMotor.RunMode mode) {
        motorFrontRight.setMode(mode);
        motorBackRight.setMode(mode);
        motorFrontLeft.setMode(mode);
        motorBackLeft.setMode(mode);
    }

    /**
     *
     * @param frontLeft Power for front left wheel.
     * @param backLeft Power for back left wheel.
     * @param frontRight Power for front right wheel.
     * @param backRight Power for back right wheel.
     */
    public void setDriveMotors(double frontLeft, double backLeft, double frontRight, double backRight) {
        motorFrontLeft.setPower(frontLeft);
        motorBackLeft.setPower(backLeft);
        motorFrontRight.setPower(frontRight);
        motorBackRight.setPower(backRight);
    }

    /**
     * @param power The power to set for all drive motors.
     */
    public void setAllDriveMotors(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    /**
     * @param power The speed to drive at.
     * @param distance How far the robot should travel (in feet).
     */
    public void driveDistance(double power, double distance) {
        //reset encoders just to be safe
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //convert input distance in feet to motor ticks
        int ticks = (int)Math.round((distance/wheelCircumference) * ticksPerRotation);

        //set the target positions for all motors
        motorFrontLeft.setTargetPosition(ticks);
        motorBackLeft.setTargetPosition(ticks);
        motorFrontRight.setTargetPosition(ticks);
        motorBackRight.setTargetPosition(ticks);

        //tells motors to run until position is reached
        setAllMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        //starts the motors
        setAllDriveMotors(power);

        //waits for motors to finish moving
        period.reset();
        while(getAllMotorsBusy()) {
            try {
                //if robot has been driving longer then we think necessary we will automatically stop and move on
                if(period.milliseconds() > ticks/power/driveTimeScalar) {
                    break;
                }
                Thread.sleep(1);
            }catch(InterruptedException e) {

            }
        }

        //set all motors to 0
        setAllDriveMotors(0);

        //resets encoder values and changes mode back to default
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param power The power for the motors.
     * @param degrees The degrees to turn.
     */
    public void turnDegrees(double power, double degrees) {
        //set to run using encoders just in case
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        //gets the current heading to refer to
        int startHeading = gyroSensor.getHeading();
        if((startHeading + degrees) >= 360){
            startHeading -= 360;
        }else if(startHeading + degrees < 0) {
            startHeading += 360;
        }

        //if it is more efficient to turn left
        if(degrees > 180) {
            setDriveMotors(-power, -power, power, power);
        } else {
            setDriveMotors(power, power, -power, -power);
        }

        //if we still need to turn
        while(Math.abs(Math.abs(startHeading - gyroSensor.getHeading()) - degrees) > 5) {
            try {
                Thread.sleep(1);
            } catch(InterruptedException e) {

            }
        }

        //stop all the motors
        setAllDriveMotors(0);
    }

    /**
     * @return If one or more motors are busy return true, otherwise false.
     */
    public boolean getAllMotorsBusy() {
        return (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy());
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
