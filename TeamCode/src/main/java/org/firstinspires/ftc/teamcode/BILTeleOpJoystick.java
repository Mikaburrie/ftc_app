package org.firstinspires.ftc.teamcode;

/**
 * Created by Lorenzo on 12/15/2015.
 * Added deadBand
 * Adjusts control of the robot via the gamepad joystick
 */
public class    BILTeleOpJoystick {
    /*
         * This is a reworked version of the method scaleInput.
         * This version replaces arrays with an exponential function,
         * thereby simplifying the acceleration speed and making it more customizable.
         */

    double scaleInput(double dVal, double expo) {

        //output
        double scaleD = Math.pow (dVal, expo);

        //check for === signs
        if((dVal < 0)!= (scaleD < 0))
                scaleD = -scaleD;

        //return
        return scaleD;
    }

    double deadFix(double scaleD, double deadband, double multiplier) {

        double fixedD = 0;

        if (scaleD < 0) {
            fixedD = (scaleD + deadband) * multiplier;
        }
        else if (scaleD > 0) {
            fixedD = (scaleD - deadband) * multiplier;
        }
        return  fixedD;
    }

    double scaleToSpeed(double scaleD, double speedLimit) {
        return scaleD * speedLimit;
    }

    public double normalizeSpeed(double dVal, double expo, double maxSpeed)
    {
        double scaleD = scaleInput(dVal, expo);
    //    double fixedD = deadFix(scaleD, .05, 1.05);
        double fixedD = scaleToSpeed(scaleD, maxSpeed);
        return fixedD;
    }

}