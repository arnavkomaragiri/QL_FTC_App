package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="six_bot", group="Teleop")  // @Autonomous(...) is the other common choice
@Disabled
public class sixbot_tele extends OpMode
{
    double pos = 0.0;
    double pitch = 0.0;
    double speed = 0.5;
    boolean trigger = false;
    FTCUtils myUtils = new FTCUtils() ;
    DcMotor up_left;
    DcMotor up_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor slide1;
    DcMotor slide2;

    Servo grab_left;
    Servo grab_right;
    Servo stick_X;
    Servo stick_Y;
    Servo relic;

    ColorSensor colorsensor;
    //OVER HERE GENIUS
    @Override
    public void init() {
        up_left = hardwareMap.dcMotor.get("up_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        up_right = hardwareMap.dcMotor.get("up_right");
        slide1 = hardwareMap.dcMotor.get("slide1");
        slide2 = hardwareMap.dcMotor.get("slide2");
        grab_left = hardwareMap.servo.get("grab_left");
        grab_right = hardwareMap.servo.get("grab_right");
        stick_X = hardwareMap.servo.get("stick_x");
        stick_Y = hardwareMap.servo.get("stick_y");
        relic = hardwareMap.servo.get("relic");
        colorsensor = hardwareMap.colorSensor.get("colorsensor");
        back_right.setDirection(DcMotor.Direction.REVERSE);
        up_right.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.REVERSE);

        grab_left.setPosition(0.1);
        grab_right.setPosition(0.9);
        stick_X.setPosition(1.0);

        stick_Y.setPosition(1.0);
        relic.setPosition(0.0);

        colorsensor.enableLed(false);
        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void start() {
        //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    @Override
    public void loop() {
        double leftpos = 0.0;
        double rightpos = 0.0;
        /*int upcounts = 0;
        int height = 0;
        int tmp = 0;
        boolean retrieve = false;*/
        double rightpower = Range.clip(gamepad1.right_stick_y,-1,1);
        double leftpower = Range.clip(gamepad1.left_stick_y,-1,1);//TEST AND SEE WHATS GOES ON
        double slidepower1 = Range.clip(gamepad2.right_stick_y,-1,1);
        double slidepower2 = Range.clip(gamepad2.left_stick_y,-1,1);

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        //leftMotor.setPower(-gamepad1.left_stick_y);
        //rightMotor.setPower(-gamepad1.right_stick_y);
        //leftpos = grab_left.getPosition();
        //rightpos = grab_right.getPosition();

        up_left.setPower(leftpower * speed);
        back_left.setPower(leftpower * speed);
        up_right.setPower(rightpower * speed);
        back_right.setPower(rightpower * speed);
        slide1.setPower(slidepower1);
        slide2.setPower(slidepower2);
        /*if (gamepad2.b){
            slide1.setPower(0.5);
            telemetry.addData("Pressed!",0);
        }*/
        if (gamepad2.a){ //servo positioning
            grab_left.setPosition(0.75);
            grab_right.setPosition(0.25);
        }
        if (gamepad2.y){
            grab_left.setPosition(0.1);
            grab_right.setPosition(0.9);
        }
        if (gamepad2.x){
            grab_left.setPosition(0.4);
            grab_right.setPosition(0.6);
        }
        if (gamepad1.dpad_up){
            relic.setPosition(0.5);
        }
        if (gamepad1.dpad_down){
            relic.setPosition(0.0);
        }
        if (gamepad1.a){
            stick_Y.setPosition(0.9);
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
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
}
/*
    todo:
        **slide coming down
        **2nd slide
        **power the right motor
        **speed value
        * remove sandbagging
 */