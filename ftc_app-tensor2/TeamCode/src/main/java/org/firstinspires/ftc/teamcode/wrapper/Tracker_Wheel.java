package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrapper.sensors.QL_Encoder;

public class Tracker_Wheel {
    QL_Encoder odometer;
    Servo tension;
    boolean engaged = false;

    public Tracker_Wheel(HardwareMap h){
        odometer = new QL_Encoder(h);
        tension = h.get(Servo.class, "tension");
        tension.setPosition(1.0);
    }

    public void setTension(Servo s){
        tension = s;
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
        tension.setPosition(0.0);
        engaged = true;
    }

    public void disengage(){
        tension.setPosition(1.0);
        engaged = false;
    }

    public double getDistance(){
        return odometer.getDistance();
    }
}
