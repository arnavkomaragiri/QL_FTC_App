package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/**
 * Created by keerthisekar on 10/9/16.
 */
public class FTCUtils {
    final static int NEVEREST40 = 1120;
    final static int NEVEREST60 = 1680;
    public void Encoder_upmotors( int distance)
    {

        System.out.println("up motors move");
    }

    public void sleep( long xxMilliSeconds){
        double x = System.nanoTime() + 1E6 * xxMilliSeconds;

        if (x != -1 && x > System.nanoTime()) {
            return;
        } else {
            x = -1;
        }

    }
    public int getNewCounts (double idistance, int ENCODER_CPR, double GEAR_RATIO){


        final int WHEEL_DIAMETER = 4; //Diameter in inches
        //final static int DISTANCE = 36; //Distance in inches

        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        final double ROTATIONS = idistance / CIRCUMFERENCE;
        final double COUNTS = ENCODER_CPR * ROTATIONS / GEAR_RATIO;
        return (int) COUNTS;
    }

    public void colorsensor () {
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;
        //Get help for logic to set up parameter?

        if ((bCurrState == true) && (bCurrState != bPrevState))  {

            // button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn;
            //colorSensor.enableLed(bLedOn);

        }

    }

    public void push_button (DcMotor up_left,DcMotor up_right) {

        up_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        up_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int upCOUNTS = getNewCounts(1,1120,1);

        up_left.setTargetPosition(upCOUNTS);
        up_right.setTargetPosition(upCOUNTS); //NEED TO ADD MOTOR SPEED
        up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_left.setPower(0.5);
        up_right.setPower(0.5);

    }
    public void move_to_button (DcMotor up_left,DcMotor up_right, DcMotor side_left, DcMotor side_right){
        up_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        side_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        side_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        up_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        side_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        side_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int upCOUNTS = getNewCounts(3,1120,1); //move BACK
        int sideCOUNTS = getNewCounts(6,11120,1); //move to other

        up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        side_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        side_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        side_right.setTargetPosition(sideCOUNTS);
        side_left.setTargetPosition(sideCOUNTS);
        up_left.setTargetPosition(upCOUNTS);
        up_right.setTargetPosition(upCOUNTS);

        if(up_left.getCurrentPosition() >= upCOUNTS){
            up_left.setPower(0);
            up_right.setPower(0);
            side_left.setPower(.5);
            side_right.setPower(.5);
            if (side_left.getCurrentPosition() >= sideCOUNTS){
                side_left.setPower(0);
                side_right.setPower(0);
            }
        } else {
            up_left.setPower(-.5);
            up_right.setPower(-.5);
        }

    }
}
