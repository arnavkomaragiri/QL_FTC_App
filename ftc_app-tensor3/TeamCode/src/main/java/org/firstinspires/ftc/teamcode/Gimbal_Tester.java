package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrapper.Gimbel;

@TeleOp(name = "Gimbal Tester", group = "Concept")
public class Gimbal_Tester extends OpMode {
    Gimbel g;

    public void init(){
        g = new Gimbel(hardwareMap);
        g.GoTo(0, 0);
    }

    public void loop(){
        g.move(gamepad1.right_stick_x / 100, gamepad1.right_stick_y / 100);
        telemetry.addData("X: ", g.getPos().x());
        telemetry.addData("Y: ", g.getPos().y());
    }
}
