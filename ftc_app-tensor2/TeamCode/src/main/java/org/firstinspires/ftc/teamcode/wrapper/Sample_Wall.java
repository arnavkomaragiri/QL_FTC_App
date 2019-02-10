package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sample_Wall {
    Servo wall;

    public Sample_Wall(Servo wall){
        this.wall = wall;
        init();
    }

    public Sample_Wall(HardwareMap h){
        this.wall = h.get(Servo.class, "ds");
        init();
    }

    public Sample_Wall(HardwareMap h, String name){
        this.wall = h.get(Servo.class, name);
        init();
    }

    public void init(){
        this.wall.setPosition(0.0);
    }

    public Servo getWall() {
        return wall;
    }

    public void setWall(Servo wall) {
        this.wall = wall;
    }

    public void engage(){
        this.wall.setPosition(0.6);
    }

    public void disengage(){
        this.wall.setPosition(0.0);
    }

    public String toString() {
        return this.wall.toString();
    }
}
