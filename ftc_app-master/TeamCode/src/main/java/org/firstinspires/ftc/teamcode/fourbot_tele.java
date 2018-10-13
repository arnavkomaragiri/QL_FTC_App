package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@TeleOp(name="four_bot", group="Teleop")
public class fourbot_tele extends OpMode{
    DcMotor motors[] = new DcMotor[4];
    Mecanum_Drive drive;
    Arm arm;

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");

        arm = new Arm(hardwareMap.get(DcMotor.class, "sweeper"), hardwareMap.get(DcMotor.class, "arm"));

        drive = new Mecanum_Drive(motors, hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"));
    }
    public void loop(){
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        drive.drive(gamepad1);

        //*Arm Motion
        arm.move(gamepad2);

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

