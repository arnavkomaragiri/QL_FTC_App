package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@TeleOp(name="four_bot", group="Teleop")
public class fourbot_tele extends OpMode{
    DcMotor up_left;
    DcMotor up_right;
    DcMotor back_left;
    DcMotor back_right;
    ColorSensor sensor;
    public void init(){
        up_left = hardwareMap.dcMotor.get("up_left");
        up_right = hardwareMap.dcMotor.get("up_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");

        up_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop(){
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        up_left.setPower(v1);
        up_right.setPower(v2);
        back_left.setPower(v3);
        back_right.setPower(v4);

        telemetry.addData("Up_Left:", v1);
        telemetry.addData("Up_Right:", v2);
        telemetry.addData("Back_Left:", v3);
        telemetry.addData("Back_Right:", v4);
        telemetry.addData("Angle:", ((180 * robotAngle) / Math.PI));
    }


}

