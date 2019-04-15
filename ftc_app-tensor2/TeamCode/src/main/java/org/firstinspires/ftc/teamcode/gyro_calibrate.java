package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.movement.Runner_Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Box;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Sample_Wall;
import org.firstinspires.ftc.teamcode.wrapper.Slide;
import org.firstinspires.ftc.teamcode.wrapper.Tracker_Wheel;
import org.firstinspires.ftc.teamcode.wrapper.sensors.QL_Encoder;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous(name = "PID Gyro Calibrate", group = "Roadrunner")
public class gyro_calibrate extends OpMode {
    Runner_Mecanum_Drive drive;
    FtcDashboard dash = FtcDashboard.getInstance();
    Trajectory t;

    Hanger hanger;
    Arm arm;
    Box box;
    Servo backboard;
    Servo marker;
    Slide slide;
    Sample_Wall wall;
    boolean mode = false;
    boolean previous = false;
    ElapsedTime precision = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    Gimbel g;
    double pos = 0.42;
    Tracker_Wheel wheel;
    QL_Encoder encoder;
    Servo temp_tensioner;
    boolean fState = false;
    DcMotor arm1;
    DcMotor sweeper;

    public void init(){
        dash.updateConfig();
        //drive.setHardwareMap(hardwareMap);
        drive = new Runner_Mecanum_Drive();
        drive.setDrive(hardwareMap);
        drive.engage();

        hanger = new Hanger(hardwareMap, true);

        //temp_tensioner = hardwareMap.get(Servo.class, "x_tension");
        //temp_tensioner.setPosition(1.0);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        wall = new Sample_Wall(hardwareMap);

        box = new Box(hardwareMap);
        box.compress();
        backboard =  hardwareMap.get(Servo.class, "back");
        marker = hardwareMap.get(Servo.class, "tm");
        backboard.setPosition(0.50);
        marker.setPosition(0.3);
        slide = new Slide(hardwareMap);

        arm = new Arm(sweeper, arm1, backboard, hardwareMap.voltageSensor.get("Motor Controller 2"), hardwareMap.get(CRServo.class, "intake_left"), hardwareMap.get(CRServo.class, "intake_right"));
        g = new Gimbel(hardwareMap);
        g.GoTo(0.5, 0);
    }

    public void start(){
        drive.reset();
        t = drive.trajectoryBuilder()
                .turnTo(Math.toRadians(180))
                .build();
        drive.followTrajectory(t);
    }

    public void loop(){
        ArrayList<Double> headings = new ArrayList<Double>();
        ArrayList<Double> yval = new ArrayList<Double>();
        ArrayList<Double> xval = new ArrayList<Double>();
        double m1 = 0.0, b1 = 0.0, r21 = 0.0, m2 = 0.0, b2 = 0.0, r22 = 0.0;
        if (drive.isFollowingTrajectory() && !drive.getDrive().getGyro().isCalibrating()){
            drive.getDrive().getPositioner().setPose(new Pose2d(0.0, 0.0, 0.0));
            drive.updateFollower();
            headings.add(drive.getExternalHeading());
            yval.add(drive.getDrive().getPositioner().getY().getDistance());
            xval.add(drive.getDrive().getPositioner().getX().getDistance());
            telemetry.addData("Error: ", drive.getFollowingError());
        }
        else{
            ArrayList<Double> data1 = linear_regression(headings, xval);
            ArrayList<Double> data2 = linear_regression(headings, yval);
            m1 = data1.get(0);
            b1 = data1.get(1);
            r21 = data1.get(2);
            m2 = data2.get(0);
            b2 = data2.get(1);
            r22 = data2.get(2);
            drive.drive(gamepad1);
            if (gamepad1.x){
                dash.updateConfig();
                drive = new Runner_Mecanum_Drive();
                drive.setDrive(hardwareMap);
                drive.disengage();
                drive.followTrajectory(t);
            }
        }
        telemetry.addData("X relationship: ", "Y = " + Double.toString(m1) + "x + " + Double.toString(b1) + " : " + Double.toString(r21));
        telemetry.addData("Y relationship: ", "Y = " + Double.toString(m2) + "x + " + Double.toString(b2) + " : " + Double.toString(r22));
        telemetry.addData("Error: ", drive.getFollowingError());
        telemetry.addData("Heading: ", Math.toDegrees(drive.getExternalHeading()));
    }

    public ArrayList<Double> linear_regression(ArrayList<Double> headings, ArrayList<Double> data){
        double sumx = 0.0, sumy = 0.0, sumx2 = 0.0;
        int n = data.size();
        for (int i = 0; i < n; i++){
            sumx += headings.get(i);
            sumx2 += Math.pow(headings.get(i), 2);
            sumy += data.get(i);
        }
        double xbar = sumx / n;
        double ybar = sumy / n;

        double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
        for (int i = 0; i < n; i++){
            xxbar += (headings.get(i) - xbar) * (headings.get(i) - xbar);
            yybar += (data.get(i) - ybar) * (data.get(i) - ybar);
            xybar += (headings.get(i) - xbar) * (data.get(i) - ybar);
        }
        double slope = xybar / xxbar;
        double intercept = ybar - slope * xbar;
        ArrayList<Double> out = new ArrayList<Double>();
        out.add(slope);
        out.add(intercept);

        double rss = 0.0;
        double ssr = 0.0;
        for (int i = 0; i < n; i++){
            double fit = slope * headings.get(i) + intercept;
            rss += Math.pow(fit - data.get(i), 2);
            ssr += Math.pow(fit - ybar, 2);
        }
        int degreesOfFreedom = n - 2;
        double r2 = ssr / yybar;
        out.add(r2);
        return out;
    }
}
