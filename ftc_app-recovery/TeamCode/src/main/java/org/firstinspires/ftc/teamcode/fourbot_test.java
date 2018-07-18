package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "fourbot_test", group = "Test")
public class fourbot_test extends OpMode{

    DcMotor up_left;
    DcMotor up_right;
    DcMotor back_left;
    DcMotor back_right;

    public void init(){
        up_left = hardwareMap.get(DcMotor.class, "up_left");
        up_right = hardwareMap.get(DcMotor.class, "up_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        up_left.setDirection(DcMotor.Direction.REVERSE);
        up_right.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){
        up_left.setPower(1.0);
        up_right.setPower(1.0);
        back_left.setPower(1.0);
        back_right.setPower(1.0);
    }
}
