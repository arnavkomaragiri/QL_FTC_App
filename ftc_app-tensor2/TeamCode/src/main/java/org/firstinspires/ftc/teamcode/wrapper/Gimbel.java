package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.Vector2d;

public class Gimbel{
    Servo servoX;
    Servo servoY;

    public Gimbel(Servo servoX, Servo servoY){
        this.servoX = servoX;
        this.servoY = servoY;
    }

    public Gimbel(HardwareMap h){
        servoX = h.get(Servo.class, "x");
        servoY = h.get(Servo.class, "y");
    }

    public void setServoX(Servo servoX) {
        this.servoX = servoX;
    }

    public void setServoY(Servo servoY) {
        this.servoY = servoY;
    }

    public Servo getServoX(){
        return this.servoX;
    }

    public Servo getServoY(){
        return this.servoY;
    }

    public void GoTo(double x, double y){
        double x_pos = 0.5 + x;
        x_pos = Range.clip(x_pos, 0.0, 1.0);
        double y_pos = 0.5 + y;
        y_pos = Range.clip(y_pos, 0.0, 1.0);
        servoX.setPosition(x_pos);
        servoY.setPosition(y_pos);
    }

    public void move(double x, double y){
        double x_pos = servoX.getPosition() + x;
        double y_pos = servoY.getPosition() + y;

        x_pos = Range.clip(x_pos, 0, 1);
        y_pos = Range.clip(y_pos, 0, 1);

        servoX.setPosition(x_pos);
        servoY.setPosition(y_pos);
    }

    public Vector2d getPos(){
        double x_pos = servoX.getPosition();
        double y_pos = servoY.getPosition();

        return new Vector2d(x_pos - 0.5, y_pos - 0.5);
    }
}
