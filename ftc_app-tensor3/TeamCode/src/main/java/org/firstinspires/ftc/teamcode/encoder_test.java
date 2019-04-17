package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Encoder Test", group = "Test: ")
public class encoder_test extends OpMode {
    AnalogInput encoder;
    CRServo intake;

    public void init(){
        encoder = hardwareMap.get(AnalogInput.class, "x_tracker");
        intake = hardwareMap.get(CRServo.class, "intake_right");
    }

    public void loop(){
        intake.setPower(1.0);
        //intake.setPower(0.5 + Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -0.5, 0.5));
        telemetry.addData("Voltage: ", encoder.getVoltage());
    }
}
