package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name = "PID Gyro Tester", group = "Roadrunner")
public class gyro_tele extends OpMode {
    Runner_Mecanum_Drive drive;
    //FtcDashboard dash = FtcDashboard.getInstance();
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
        //dash.updateConfig();
        //drive.setHardwareMap(hardwareMap);
        drive = new Runner_Mecanum_Drive();
        drive.setDrive(hardwareMap);
        drive.disengage();

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
        t = drive.trajectoryBuilder()
                .turnTo(Math.toRadians(180))
                .build();
        drive.followTrajectory(t);
    }

    public void loop(){
        if (drive.isFollowingTrajectory() && !drive.getDrive().getGyro().isCalibrating()){
            drive.getDrive().getPositioner().setPose(new Pose2d(0.0, 0.0, 0.0));
            drive.updateFollower();
            telemetry.addData("Error: ", drive.getFollowingError());
        }
        else{
            drive.drive(gamepad1);
            if (gamepad1.x){
                //dash.updateConfig();
                drive = new Runner_Mecanum_Drive();
                drive.setDrive(hardwareMap);
                drive.disengage();
                drive.followTrajectory(t);
            }
        }
        telemetry.addData("Error: ", drive.getFollowingError());
        telemetry.addData("Heading: ", Math.toDegrees(drive.getExternalHeading()));
    }
}
