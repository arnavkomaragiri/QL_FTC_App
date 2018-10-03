package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@TeleOp(name="four_bot", group="Teleop")
public class fourbot_tele extends OpMode{
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm;
    DcMotor sweeper;
    Mecanum_Drive drive;

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");

        arm = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        drive = new Mecanum_Drive(motors);
    }
    public void loop(){
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        drive.drive(r, robotAngle, rightX);

        //*Arm Motion
        arm.setPower(gamepad2.right_stick_y * 0.6);
        if (gamepad2.dpad_up){
            arm.setPower(0.6);
        }
        else if (gamepad2.dpad_down){
            arm.setPower(-0.6);
        }
        else{
            arm.setPower(0.0);
        }
        sweeper.setPower(-gamepad2.right_trigger);

        telemetry.addData("Angle:", ((180 * robotAngle) / Math.PI));
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

