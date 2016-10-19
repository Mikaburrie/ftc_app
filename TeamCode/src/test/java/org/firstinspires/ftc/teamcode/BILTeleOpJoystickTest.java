package org.firstinspires.ftc.teamcode;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by Administrator on 12/17/2015.
 */
public class BILTeleOpJoystickTest {

    private BILTeleOpJoystick bilTeleOpJoystick;

    @Before
    public void setUp() throws Exception {
        bilTeleOpJoystick = new BILTeleOpJoystick ();
    }

    @After
    public void tearDown() throws Exception {

    }

    @Test
    public void testScaleInput() throws Exception {

        // Set up
        double [] joystickInputVal = {-1, -.7 ,-.5, -.3, 0, .3, .5, .7, 1, -1, -.7 ,-.5, -.3, 0, .3, .5, .7, 1};
        double [] expo = {2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3};
        double [] expectedValue = {-1, -.49, -.25, -.09, 0, .09, .25, .49, 1, -1, -.342, -.125, -.026, 0, .026, .125, .342, 1};

        for (int idx = 0; idx < joystickInputVal.length; idx ++) {
            // Run the test
            double scaleD = bilTeleOpJoystick.scaleInput(joystickInputVal [idx], expo [idx]);

            // Verify
            Assert.assertEquals("Unexpected returned scale value for index: "+idx, expectedValue [idx], scaleD , 0.01);
        }
    }

    @Test
    public void testDeadFix () {

        // Set up
        double [] scaleDval = {-1, -.49, -.25, -.09, 0, .09, .25, .49, 1};
        double [] expectedValue = {-.9975, -.462, -.21, -.042, 0, .042, .21, .462, 1};

        for (int idx = 0; idx < scaleDval.length; idx ++) {
            //Run test
            double fixedD = bilTeleOpJoystick.deadFix (scaleDval [idx], .05, 1.05);

            //Verify
            Assert.assertEquals("Unexpected returned scale value for index: "+idx, expectedValue [idx], fixedD , 0.01);
        }
  }

}