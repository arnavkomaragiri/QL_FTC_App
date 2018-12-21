package org.firstinspires.ftc.teamcode.wrapper.motors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class QL_Servo { //possible limit switch integration extension?
    Servo s;
    double pos = 0.0;

    public QL_Servo(HardwareMap h, String name){
        s = h.get(Servo.class, name);
    }

    public QL_Servo(Servo s){
        this.s = s;
    }

    public void setPosition(double pos){
        if (this.pos != pos){
            s.setPosition(pos);
            this.pos = pos;
        }
    }

    public double getPosition(){
        return s.getPosition();
    }

    public void setServo(Servo s){
        this.s = s;
    }

    public Servo getServo(){
        return this.s;
    }

    public String toString(){
        String output = "";
        output += s.toString() + "\n";
        output += "Pos: " + Double.toString(pos) + "\n";
        return output;
    }
}
