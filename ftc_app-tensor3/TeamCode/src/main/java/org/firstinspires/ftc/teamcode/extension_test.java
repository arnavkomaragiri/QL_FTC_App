package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Extension_Test", group = "Test")
public class extension_test extends OpMode {
    DcMotor extend;

    public void init(){
        extend = hardwareMap.get(DcMotor.class, "extend");
    }

    public void loop(){
        extend.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

    }
}
