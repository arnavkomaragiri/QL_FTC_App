package org.firstinspires.ftc.teamcode.wrapper.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class QL_ColorSensor {

    ColorSensor sensor;

    public QL_ColorSensor(ColorSensor origin){
        sensor = origin;
    }

    public QL_ColorSensor(ColorSensor origin, I2cAddr addr){
        sensor.setI2cAddress(addr);
        sensor = origin;
    }

    public void setAddr(I2cAddr addr){
        sensor.setI2cAddress(addr);
    }

    public I2cAddr getAddr(){
        return sensor.getI2cAddress();
    }

    public void setSensor(ColorSensor origin){
        sensor = origin;
    }

    public ColorSensor getSensor(){
        return sensor;
    }

    public int red() { return sensor.red(); }
    public int green() { return sensor.green(); }
    public int blue() { return sensor.blue(); }
    public int alpha() { return sensor.alpha(); }

}
