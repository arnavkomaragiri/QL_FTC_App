package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrapper.sensors.QL_Encoder;

public class Tracker_Wheel {
    QL_Encoder odometer;
    Servo tension;
    boolean engaged = false;
    boolean inverted = false;
    boolean specific = false;

    public Tracker_Wheel(HardwareMap h){
        odometer = new QL_Encoder(h);
        tension = h.get(Servo.class, "tension");
        tension.setPosition(1.0);
    }

    public Tracker_Wheel(HardwareMap h, String name){
        odometer = new QL_Encoder(h.get(AnalogInput.class, name));
        tension = h.get(Servo.class, "x_tension");
        tension.setPosition(1.0);
        specific = true;
    }

    public void setRatio(double r){
        odometer.setRatio(r);
    }

    public void setTension(Servo s){
        tension = s;
    }

    public void setInverted(boolean inverted){
        this.inverted = inverted;
    }

    public Servo getTension(){
        return tension;
    }

    public void setOdometer(QL_Encoder e){
        odometer = e;
    }

    public QL_Encoder getOdometer(){
        return odometer;
    }

    public void reset(){
        odometer.reset();
    }

    public boolean isEngaged(){
        return engaged;
    }

    public void engage(){
        tension.setPosition((inverted ? (specific ? 0.826 : 1.0) : 0.0));
        engaged = true;
    }

    public void disengage(){
        tension.setPosition((inverted ? 0.0 : 1.0));
        engaged = false;
    }

    public double getDistance(){
        return odometer.getDistance();
    }
}
