package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

/**
 * Created by Mika/Alex on 12/12/2015.
 */
public abstract class BILAutonomousCommon extends LinearOpMode {
    BILRobotHardware robot = new BILRobotHardware();

    public final static int ticksPerRotation = 1440;
    public final static double wheelCircumference = (4 * Math.PI)/12; //circumference in feet
    public final static int driveTimeScalar = 3;
    public final double lineColorThreshold = 0.1;
    private ElapsedTime period = new ElapsedTime();

    /**
     * @param mode The run mode to set for all motors.
     */
    public void setAllMotorModes(DcMotor.RunMode mode) {
        robot.motorFrontRight.setMode(mode);
        robot.motorBackRight.setMode(mode);
        robot.motorFrontLeft.setMode(mode);
        robot.motorBackLeft.setMode(mode);
    }

    /**
     *
     * @param frontLeft Power for front left wheel.
     * @param backLeft Power for back left wheel.
     * @param frontRight Power for front right wheel.
     * @param backRight Power for back right wheel.
     */
    public void setDriveMotors(double frontLeft, double backLeft, double frontRight, double backRight) {
        robot.motorFrontLeft.setPower(frontLeft);
        robot.motorBackLeft.setPower(backLeft);
        robot.motorFrontRight.setPower(frontRight);
        robot.motorBackRight.setPower(backRight);
    }

    /**
     * @param power The power to set for all drive motors.
     */
    public void setAllDriveMotors(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
    }

    /**
     * @param power The speed to drive at.
     * @param distance How far the robot should travel (in feet).
     */
    public void driveDistance(double power, double distance) {
        //reset encoders just to be safe
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //convert input distance in feet to motor ticks
        int ticks = (int)Math.round(Math.abs(distance/wheelCircumference) * ticksPerRotation);

        //set the target positions for all motors
        robot.motorFrontLeft.setTargetPosition(ticks);
        robot.motorBackLeft.setTargetPosition(ticks);
        robot.motorFrontRight.setTargetPosition(ticks);
        robot.motorBackRight.setTargetPosition(ticks);

        //tells motors to run until position is reached
        setAllMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        //starts the motors
        setAllDriveMotors(power);

        //waits for motors to finish moving
        period.reset();
        while(getAllMotorsBusy()) {
            try {
                //if robot has been driving longer then we think necessary we will automatically stop and move on
                if(period.milliseconds() > Math.abs(ticks/power/driveTimeScalar)) {
                    break;
                }
                idle();
            }catch(InterruptedException e) {
                //do nothing
            }
        }

        //set all motors to 0
        setAllDriveMotors(0);

        //resets encoder values and changes mode back to default
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByTime(double power, int milliseconds) {
        period.reset();
        setAllDriveMotors(power);
        while(period.milliseconds() < milliseconds) {
            try {
                idle();
            }catch (InterruptedException e){
                //do nothing
            }
        }
    }

    /**
     * @param power The power for the motors.
     * @param degrees The degrees to turn.
     */
    public void turnDegrees(double power, double degrees) {
        //set to run using encoders just in case
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        //gets the current heading to refer to
        int startHeading = robot.gyroSensor.getHeading();
        if((startHeading + degrees) >= 360){
            startHeading -= 360;
        }else if(startHeading + degrees < 0) {
            startHeading += 360;
        }

        //if it is more efficient to turn left
        if(degrees > 180 || degrees < 0) {
            setDriveMotors(-power, -power, power, power);
        } else {
            setDriveMotors(power, power, -power, -power);
        }

        //if we still need to turn
        while(Math.abs(Math.abs(startHeading - robot.gyroSensor.getHeading()) - Math.abs(degrees)) > 5) {
            try {
                idle();
            } catch(InterruptedException e) {
                //do nothing
            }
        }

        //stop all the motors
        setAllDriveMotors(0);
    }

    public void driveUntilLineOrDistance(double power, double distance, double floorColor) {
        robot.lightSensor.enableLed(true);
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int)Math.round((distance/wheelCircumference) * ticksPerRotation);

        robot.motorFrontLeft.setTargetPosition(ticks);
        robot.motorBackLeft.setTargetPosition(ticks);
        robot.motorFrontRight.setTargetPosition(ticks);
        robot.motorBackRight.setTargetPosition(ticks);

        setAllMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        setAllDriveMotors(power);

        //waits for motors to finish moving
        period.reset();
        while(getAllMotorsBusy() && robot.lightSensor.getLightDetected() < floorColor + lineColorThreshold) {
            try {
                //if robot has been driving longer then we think necessary we will automatically stop and move on
                if(period.milliseconds() > ticks/power/driveTimeScalar) {
                    break;
                }
                idle();
            }catch(InterruptedException e) {
                //do nothing
            }
        }

        //set all motors to 0
        setAllDriveMotors(0);

        //resets encoder values and changes mode back to default
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @return If one or more motors are busy return true, otherwise false.
     */
    public boolean getAllMotorsBusy() {
        return (robot.motorFrontLeft.isBusy() || robot.motorBackLeft.isBusy() || robot.motorFrontRight.isBusy() || robot.motorBackRight.isBusy());
    }


    public void driveToTarget(VectorF translation){
        double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
        double zTrans = (double)translation.get(2);

        double degreesToTurn = Math.toDegrees(Math.atan2(zTrans, xTrans)) + 90; //horizontal phone

        double leftSpeed = Range.clip((40 + degreesToTurn * 2) / 100, -Math.abs(zTrans) / 2000, Math.abs(zTrans) / 2000);
        double rightSpeed = Range.clip((40 - degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
        setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
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