package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;

//@TeleOp(name = "mecanum_tele", group = "Proto:")
public class mecanum_tele extends OpMode{
    Mecanum_Drive drivetrain;

    Servo grab_left;
    Servo grab_right;
    Servo grab_top;

    DcMotor slide;

    double speed = 0.8;
    boolean running = false;
    boolean training_wheels = false;

    private enum release_state{
        STATE_BOTTOM,
        STATE_TOP
    }

    ElapsedTime mReleaseStateTime = new ElapsedTime();
    release_state releaseState;

    public void init(){
        slide = hardwareMap.get(DcMotor.class, "slide");

        drivetrain = new Mecanum_Drive(hardwareMap);

        grab_left = hardwareMap.servo.get("grab_left");
        grab_right = hardwareMap.servo.get("grab_right");
        grab_top = hardwareMap.servo.get("grab_top");

        grab_left.setPosition(0.1);
        grab_right.setPosition(0.9);
        grab_top.setPosition(0.35);
    }
    public void loop(){
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double rightX = gamepad1.right_stick_x;

        drivetrain.drive(r * speed, robotAngle, rightX, gamepad1.right_stick_y);
        slide.setPower(gamepad2.right_stick_y);

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
        if (gamepad2.b) {
            running = true;
            newreleaseState(release_state.STATE_BOTTOM);
        }
        if (running){
            switch(releaseState){
                case STATE_BOTTOM:
                    grab_left.setPosition(0.5);
                    grab_right.setPosition(0.5);
                    if (mReleaseStateTime.time() >= 0.5){
                        newreleaseState(release_state.STATE_TOP);
                    }
                    break;
                case STATE_TOP:
                    grab_top.setPosition(0.35);
                    if (mReleaseStateTime.time() >= 0.5){
                        running = false;
                    }
                    break;
            }
        }
    }
    private void newreleaseState (release_state newreleasestate){
        mReleaseStateTime.reset();
        releaseState = newreleasestate;
    }
}
