package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

//@TeleOp(name="sixbot_tele_no_relic", group="Teleop")
//@Disabled
public class sixbot_tele_no_relic extends OpMode
{
    double pos = 0.0;
    double pitch = 0.0;
    double speed = 0.8; //CHANGED FROM 0.5 TO 0.7 11/10/17
    double leftpos = 0.0;
    double rightpos = 0.0;

    boolean running = false;
    boolean freeze = false;
    boolean process = false;
    boolean returning = false;
    boolean trainingwheels = false;

    FTCUtils myUtils = new FTCUtils() ;

    DcMotor up_left;
    DcMotor up_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor slide1;
    DcMotor slide2;

    Servo grab_left;
    Servo grab_right;
    Servo grab_top;
    Servo stick_X;
    Servo stick_Y;
    Servo relic;

    OpticalDistanceSensor odr;

    ModernRoboticsTouchSensor touch;

    ColorSensor colorsensor;

    final String QTAG= "QL8719";

    ElapsedTime mReleaseStateTime = new ElapsedTime();

    release_state releaseState;
    //OVER HERE GENIUS
    //@Override
    private enum release_state{
        STATE_BOTTOM,
        STATE_TOP
    }
    public void init() {
        up_left = hardwareMap.dcMotor.get("up_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        up_right = hardwareMap.dcMotor.get("up_right");
        slide1 = hardwareMap.dcMotor.get("slide1"); //O0I
        slide2 = hardwareMap.dcMotor.get("slide2");
        grab_left = hardwareMap.servo.get("grab_left");
        grab_right = hardwareMap.servo.get("grab_right");
        grab_top = hardwareMap.servo.get("grab_top");
        stick_X = hardwareMap.servo.get("stick_y"); // code change to match auto config
        stick_Y = hardwareMap.servo.get("stick_x"); // code change to match auto config
        relic = hardwareMap.servo.get("relic");
        colorsensor = hardwareMap.colorSensor.get("colorsensor");
        odr = hardwareMap.opticalDistanceSensor.get("odr");

        touch = hardwareMap.get(ModernRoboticsTouchSensor.class, "touch");

        back_right.setDirection(DcMotor.Direction.REVERSE);
        up_right.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.REVERSE);


        grab_left.setPosition(0.1);
        grab_right.setPosition(0.9);
        grab_top.setPosition(0.35);
        stick_X.setPosition(1.0);
        stick_Y.setPosition(0.9);
        relic.setPosition(0.0);
        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        colorsensor.enableLed(false);
    }



    @Override
    public void start() {
        //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stick_Y.setPosition(0.9);
        logMessage("Start", "Setting stickY to .9");
    }
    //@Override
    public void loop() {
        double leftpos = 0.0;
        double rightpos = 0.0;

        /*int upcounts = 0;
        int height = 0;
        int tmp = 0;
        boolean retrieve = false;*/
            double rightpower = Range.clip(gamepad1.right_stick_y, -1, 1);
            double leftpower = Range.clip(gamepad1.left_stick_y, -1, 1);//TEST AND SEE WHATS GOES ON
            double slidepower1 = Range.clip(gamepad2.left_stick_y, -1, 1);
            double slidepower2 = Range.clip(gamepad2.right_stick_y, -1, 1);

            leftpos = grab_left.getPosition();
            rightpos = grab_right.getPosition();

            up_left.setPower(leftpower * speed); //CHANGED SPEED
            back_left.setPower(leftpower * speed);
            up_right.setPower(rightpower * speed);
            back_right.setPower(rightpower * speed);
            //slide1.setPower(slidepower1);
            slide2.setPower(slidepower2);

        /*if (gamepad2.b){
            slide1.setPower(0.5);
            telemetry.addData("Pressed!",0);
        }*/
            if (gamepad1.dpad_up){
                if (!trainingwheels){
                    trainingwheels = true;
                    speed = 0.4;
                }
                else{
                    trainingwheels = false;
                    speed = 0.8;
                }
            }
            if (gamepad2.a) { //servo positioning
                grab_left.setPosition(0.75);
                grab_right.setPosition(0.35);
                grab_top.setPosition(1.0);
            }

            //x = open the plate halfway from full and closed -KEERTHI
            if (gamepad2.x) {
                grab_left.setPosition(0.4);
                grab_right.setPosition(0.6);// 0.45
                grab_top.setPosition(0.35);
            }

            if (gamepad2.y) {
                grab_left.setPosition(0.1);
                grab_right.setPosition(0.9);
                grab_top.setPosition(0.35);
            }
            /*if (gamepad2.b) {
                grab_left.setPosition(0.45);
                grab_right.setPosition(0.55);
                grab_top.setPosition(0.35);
            }*/
            if (gamepad2.dpad_up) {
                pos = relic.getPosition();
                pos = pos + 0.01;
                if (pos >= 1.0) {
                    pos = 1.0;
                }
                relic.setPosition(pos);
            }
            if (gamepad2.dpad_down) {
                pos = relic.getPosition();
                pos = pos - 0.01;
                if (pos <= 0.0) {
                    pos = 0.0;
                }
                relic.setPosition(pos);
            }
            if (relic.getPosition() != pos){
                pos = relic.getPosition();
            }
            if (gamepad2.dpad_left){
                process = true;
            }
            if (gamepad2.b) {
                stick_Y.setPosition(0.9);
                running = true;
                logMessage("Task:","Starting");
                newreleaseState(release_state.STATE_BOTTOM);
            }
            if (running){
                switch(releaseState){
                    case STATE_BOTTOM:
                        grab_left.setPosition(0.5);
                        grab_right.setPosition(0.5);
                        logMessage("Task:","In Progress");
                        if (mReleaseStateTime.time() >= 0.5){
                            newreleaseState(release_state.STATE_TOP);
                        }
                        break;
                    case STATE_TOP:
                        grab_top.setPosition(0.35);
                        if (mReleaseStateTime.time() >= 0.5){
                            running = false;
                            logMessage("Task:","Done");
                        }
                        break;
                }
            }

        /*if (gamepad2.a){
            pos = pos + 0.01;
            stick_X.setPosition(pos);
        }
        if (gamepad2.b){
            if (pos >= 0.01){
                pos = pos - 0.01;
                stick_X.setPosition(pos);
            }
        }
        if (gamepad2.y){
            pitch = pitch + 0.01;
            stick_Y.setPosition(pitch);
        }
        if (gamepad2.a){
            if (pitch >= 0.01){
                pitch = pitch - 0.01;
                stick_Y.setPosition(pitch);
            }
        }
        */
        if (touch.getValue() == 0.0){
            telemetry.addData("Bad Grab:", Double.toString(touch.getValue()));
        }
        else{
            telemetry.addData("Good Grab:", Double.toString(touch.getValue()));
        }
        logMessage("Light Detected:", Double.toString(odr.getLightDetected()));
        logMessage("Raw Light Detected:", Double.toString(odr.getRawLightDetected()));
        telemetry.addData("Right Stick",gamepad1.right_stick_y);
        telemetry.addData("Slides",gamepad2.left_stick_y);
        telemetry.addData("Left Stick",gamepad1.left_stick_y);
        telemetry.addData("Grab Left",leftpos);
        telemetry.addData("Grab Right",rightpos);
        telemetry.addData("Right X", gamepad1.right_stick_x);
        telemetry.addData("Left X", gamepad1.left_stick_x);
        //telemetry.addData("Counts",slide1.getCurrentPosition());
        telemetry.addData("Pitch",pitch);
        telemetry.addData("Pos",pos);
        logMessage("Grab_Left:", Double.toString(leftpos));
        logMessage("Grab_Right:", Double.toString(rightpos));
    }

    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii(QTAG, getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }
    private void newreleaseState (sixbot_tele_no_relic.release_state newreleasestate){
        mReleaseStateTime.reset();
        releaseState = newreleasestate;

    }

}
/*
    todo:
        **slide coming down
        **2nd slide
        **power the right motor
        **speed value
 */