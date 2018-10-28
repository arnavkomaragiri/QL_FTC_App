package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@TeleOp(name="four_bot_tele", group="Teleop")
public class fourbot_tele extends OpMode{
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
    Hanger hanger;
    Arm arm;
    Servo box_left;
    Servo box_right;
    boolean flip = false;
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime precision = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    DcMotorEx motor;

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");
        
        hanger = new Hanger(hardwareMap);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        box_left = hardwareMap.get(Servo.class, "box_left");
        box_right = hardwareMap.get(Servo.class, "box_right");
        box_left.setPosition(0.85);
        box_right.setPosition(0.15);

        arm = new Arm(sweeper, arm1);

        drive = new Mecanum_Drive(motors, hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"));
        //drive.enable();
    }
    public void loop(){
        drive.drive(gamepad1);

        hanger.operate(gamepad2);
        arm.move(gamepad2);

        if (precision.time() > 0.5) {
            pose = drive.track();
            precision.reset();
        }
        if (gamepad2.left_bumper && cooldown.time() > 0.25){
            if (!flip){
                box_left.setPosition(0.0);
                box_right.setPosition(1.0);
                flip = true;
            }
            else {
                box_left.setPosition(0.85);
                box_right.setPosition(0.15);
                flip = false;
            }
            cooldown.reset();
        }

        telemetry.addData("Arm Pos: ", arm.getArm().getCurrentPosition());
        telemetry.addData("Angle:", Math.toDegrees(drive.getRobotHeading()));
        telemetry.addData("Up Left: ", Double.toString(motors[0].getPower()));
        telemetry.addData("Up Right: ", Double.toString(motors[1].getPower()));
        telemetry.addData("Back Left: ", Double.toString(motors[2].getPower()));
        telemetry.addData("Back Right: ", Double.toString(motors[3].getPower()));
        telemetry.addData("X: ", pose.x());
        telemetry.addData("Y: ", pose.y());
        telemetry.addData("Heading: ", pose.heading());
        telemetry.addData("Time: ", precision.time());
        telemetry.addData("Min: ", drive.getMinimumDistance());
        for (int i = 0; i < 4; i++){
            telemetry.addData("Pos: ", Double.toString((Math.PI * drive.getMotors()[i].getCurrentPosition()) / 140));
        }
    }
    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii("8719", getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }

    public void processBitmap(Bitmap bmp){
        for (int i = 0; i < bmp.getHeight(); i++){
            for (int j = 0; j < bmp.getWidth(); j++){
                int pixel = bmp.getPixel(j, i);

                int red = Color.red(pixel);
                int green = Color.green(pixel);
                int blue = Color.blue(pixel);
                float hsv[] = new float[3];

                Color.RGBToHSV(red, green, blue, hsv);

                if (hsv[0] < 65 && hsv[0] > 20){ //todo: tune hsv thresholds
                    hsv[0] = 50;
                    pixel = Color.HSVToColor(hsv);
                    bmp.setPixel(j, i, pixel);
                }
            }
        }
    }
}

