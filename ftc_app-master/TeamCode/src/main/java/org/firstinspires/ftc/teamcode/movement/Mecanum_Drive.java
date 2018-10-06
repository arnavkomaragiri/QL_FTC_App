package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.control.Pid;

import java.util.Arrays;
import java.util.Collections;

public class Mecanum_Drive {
    private final double ticksPerRevolution = 1000;  // Get for your motor and gearing.
    private double prevTime;  // The last time loop() was called.
    private int prevLeftEncoderPosition;   // Encoder tick at last call to loop().
    private int prevRightEncoderPosition;  // Encoder tick at last call to loop().

    private final double drivePidKp = 1;     // Tuning variable for PID.
    private final double drivePidTi = 1.0;   // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = 105;  // Limit to max speed.
    private final double driveOutMax = 1.0;  // Motor output limited to 100%.

    DcMotor motors[];

    private Pid leftDrive = null;
    private Pid rightDrive = null;

    public Mecanum_Drive(DcMotor names[]){
        motors = names.clone();
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotors(DcMotor names[]){
        motors = names.clone();
    }

    public DcMotor[] getMotors(){
        return motors.clone();
    }

    public String toString(){
        String output = "";
        output += Double.toString(motors[0].getPower());
        output += " " + Double.toString(motors[1].getPower());
        output += " " + Double.toString(motors[2].getPower());
        output += " " + Double.toString(motors[3].getPower());

        return output;
    }

    public void drive(double r, double angle, double rightX){
        double robotAngle = angle;

        Double[] v = new Double[4];
        double max = 0.0;
        double scale = 1.0;

        v[0] = Math.cos(robotAngle) + rightX;
        v[1] = Math.sin(robotAngle) - rightX;
        v[2] = Math.sin(robotAngle) + rightX;
        v[3] = Math.cos(robotAngle) - rightX;

        max = Collections.max(Arrays.asList(v));
        scale = Math.abs(r / max);

        for (int i = 0; i < 4; i++){
            //v[i] *= scale;
            motors[i].setPower(v[i]);
        }
    }
    
    public void drive(Gamepad g){
        double r = Math.hypot(g.left_stick_x, g.left_stick_y);
        double robotAngle = Math.atan2(g.left_stick_y, g.left_stick_x) - Math.PI / 4;
        double rightX = g.right_stick_x;

        Double[] v = new Double[4];
        double max = 0.0;
        double scale = 1.0;

        v[0] = Math.cos(robotAngle) + rightX;
        v[1] = Math.sin(robotAngle) - rightX;
        v[2] = Math.sin(robotAngle) + rightX;
        v[3] = Math.cos(robotAngle) - rightX;

        max = Collections.max(Arrays.asList(v));
        scale = Math.abs(r / max);

        for (int i = 0; i < 4; i++){
            //v[i] *= scale;
            motors[i].setPower(v[i]);
        }
    }
}
