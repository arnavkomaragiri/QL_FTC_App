package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.Pid;
import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.control.Vector2d;

import java.util.Arrays;
import java.util.Collections;

public class Mecanum_Drive{
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
    ModernRoboticsI2cGyro gyro;

    Pose2d p = new Pose2d(0, 0, 0);
    double[] prevpos = new double[4];
    double robotHeading = 0.0;

    private Pid leftDrive = null;
    private Pid rightDrive = null;

    public Mecanum_Drive(DcMotor names[], ModernRoboticsI2cGyro gyro){
        motors = names.clone();
        this.gyro = gyro;
        this.gyro.calibrate();
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

    public void drive(Gamepad gamepad1) {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        robotHeading = angle;
        double robotAngle = angle - Math.PI / 4;
        double max = -1.0;

        Double[] v = {0.0, 0.0, 0.0, 0.0};

        v[0] = (r * Math.cos(robotAngle)) + rightX;
        v[1] = (r * Math.sin(robotAngle)) - rightX;
        v[2] = (r * Math.sin(robotAngle)) + rightX;
        v[3] = (r * Math.cos(robotAngle)) - rightX;

        for (int i = 0; i < 4; i++) {
            if (v[i] > max) {
                max = v[i];
            }
        }
        if (max <= 0) {
            max = 1;
        }
        //System.out.println();
        double scale = ((r == 0) ? Math.hypot(rightX, rightY) : r) / ((max == 0) ? ((r == 0) ? Math.hypot(rightX, rightY) : r) : max);
        //System.out.println(scale);
        //System.out.println();

        for (int i = 0; i < 4; i++) {
            v[i] *= scale;
            //System.out.println(v[i]);
            v[i] = Range.clip(v[i], -1.0, 1.0);
            motors[i].setPower(v[i]);
        }
    }

    public void drive(double r, double angle, double rightX, double rightY) {
        robotHeading = angle;
        double robotAngle = angle - Math.PI / 4;
        double max = -1.0;

        Double[] v = {0.0, 0.0, 0.0, 0.0};

        v[0] = (r * Math.cos(robotAngle)) + rightX;
        v[1] = (r * Math.sin(robotAngle)) - rightX;
        v[2] = (r * Math.sin(robotAngle)) + rightX;
        v[3] = (r * Math.cos(robotAngle)) - rightX;

        for (int i = 0; i < 4; i++) {
            if (v[i] > max) {
                max = v[i];
            }
        }
        if (max <= 0) {
            max = 1;
        }
        //System.out.println();
        double scale = ((r == 0) ? Math.hypot(rightX, rightY) : r) / ((max == 0) ? ((r == 0) ? Math.hypot(rightX, rightY) : r) : max);
        //System.out.println(scale);
        //System.out.println();

        for (int i = 0; i < 4; i++) {
            v[i] *= scale;
            //System.out.println(v[i]);
            v[i] = Range.clip(v[i], -1.0, 1.0);
            motors[i].setPower(v[i]);
        }
    }

    public Pose2d track(){
        Vector2d[] v = new Vector2d[4];

        double heading = gyro.getHeading(); //todo: format heading
        double z = Math.toRadians(inverse(gyro.getIntegratedZValue()));
        heading = Math.toRadians(invert(heading)); //fix: mount gyro sideways bc I'm lazy LUL

        double x = 0.0, y = 0.0;

        for (int i = 0; i < 4; i++){
            double distance = (motors[i].getCurrentPosition() - prevpos[i]) / 560;
            distance *= 4 * Math.PI;
            v[i] = new Vector2d(distance, (i == 1 || i == 2) ? distance * -1 : distance);
            prevpos[i] = motors[i].getCurrentPosition();
        }

        Vector2d left = v[0].normalize(v[2]); //normalize by averaging x and y coordinates
        Vector2d right = v[1].normalize(v[3]); //normalize legs first, then normalize two legs together to get overall transposition
        Vector2d center = left.normalize(right); //todo: insert rotational logic

        double startx = center.norm() * Math.cos(heading + robotHeading);
        double starty = center.norm() * Math.sin(heading + robotHeading);

        p = new Pose2d(p.x() + starty, p.y() + startx, heading);
        return p;
    }
    public double invert(double heading){
        return (360 - heading) % 360;
    }
    public double flip(double heading){
        return Math.abs((heading - 180) % 360);
    }
    public double inverse(double heading){
        return heading * -1;
    }

    public double getRobotHeading(){
        return this.robotHeading;
    }

    public double getMinimumDistance(){
        double min = 9999;
        for (int i = 0; i < 4; i++){
            double distance = motors[i].getCurrentPosition() - prevpos[i];
            if (Math.abs(distance) < min){
                min = distance;
            }
        }
        return min;
    }

    public void enable(){
        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
