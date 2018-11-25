package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.wrapper.sensors.QL_ColorSensor;

//@TeleOp(name = "Multi-Color Test", group = "Concept")

public class test_color extends OpMode{
    ColorSensor color1;
    ColorSensor color2;
    QL_ColorSensor sensor1;
    QL_ColorSensor sensor2;

    public void init(){
        color1 = hardwareMap.get(ColorSensor.class, "colorsensor");
        color2 = hardwareMap.get(ColorSensor.class, "color2");

        sensor1 = new QL_ColorSensor(color1);
        sensor2 = new QL_ColorSensor(color2, I2cAddr.create8bit(0x42));
    }
    public void loop(){
        telemetry.addData("Color 1 R:", color1.red());
        telemetry.addData("Color 1 G:", color1.green());
        telemetry.addData("Color 1 B:", color1.blue());
        telemetry.addData("Color 2 R:", color2.red());
        telemetry.addData("Color 2 G:", color2.green());
        telemetry.addData("Color 2 B:", color2.blue());
    }
}
