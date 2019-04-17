package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Filter_Tester", group = "Hardware Test")
public class Filter_Tester extends OpMode {
    Servo filter;

    public void init(){
        filter = hardwareMap.get(Servo.class, "filter");
        filter.setPosition(0.0);
    }

    public void loop(){
        filter.setPosition(filter.getPosition() + (gamepad1.dpad_up ? 0.001 : (gamepad1.dpad_down ? -0.001 : 0.0)));
        telemetry.addData("Pos: ", filter.getPosition());
    }
}
