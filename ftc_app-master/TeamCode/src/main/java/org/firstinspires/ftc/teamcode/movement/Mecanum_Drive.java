package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.control.Pid;
import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.control.Vector2d;

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

    Pose2d p = new Pose2d(0, 0, 0);//getting ready to use new pose class
    double[] prevpos = new double[4];//initializing the previous position in an array of doubles.
    double robotHeading = 0.0;//initializing the robot heading.

    DcMotor motors[];//initializing my array of motors.
    ModernRoboticsI2cGyro gyro;

    //Reversing the motors because of the fact that we have mecanum wheels.
    public Mecanum_Drive(DcMotor names[], ModernRoboticsI2cGyro gyro){
        motors = names.clone();
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        this.gyro = gyro;
    }

    public void setMotors(DcMotor names[]){
        motors = names.clone();
    }//todo: Explain this!

    public DcMotor[] getMotors(){
        return motors.clone();
    }

//Displaying values of motor speeds.
    public String toString(){
        String output = "";
        output += Double.toString(motors[0].getPower());
        output += " " + Double.toString(motors[1].getPower());
        output += " " + Double.toString(motors[2].getPower());
        output += " " + Double.toString(motors[3].getPower());

        return output;
    }

    public void drive(double r, double angle, double rightX){
        robotHeading = angle;
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
        robotHeading = Math.atan2(g.left_stick_y, g.left_stick_x);
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
    public void track(){
        Vector2d[] v = new Vector2d[4];

        double heading = gyro.getHeading(); //todo: format heading
        double z = Math.toRadians(inverse(gyro.getIntegratedZValue()));
        heading = Math.toRadians(invert(heading)); //fix: mount gyro sideways bc I'm lazy LUL

        double x = 0.0, y = 0.0;

        for (int i = 0; i < 4; i++){
            double distance = motors[i].getCurrentPosition() - prevpos[i];
            v[i] = new Vector2d((i == 1 || i == 2) ? distance * -1 : distance, distance);
            prevpos[i] = motors[i].getCurrentPosition();
        }

        Vector2d left = v[0].normalize(v[2]); //normalize by averaging x and y coordinates
        Vector2d right = v[1].normalize(v[3]); //normalize legs first, then normalize two legs together to get overall transposition
        Vector2d center = left.normalize(right); //todo: insert rotational logic

        double startx = center.norm() * Math.cos(heading + robotHeading);
        double starty = center.norm() * Math.sin(heading + robotHeading);

        p = new Pose2d(p.x() + startx, p.y() + starty, heading);
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
}
