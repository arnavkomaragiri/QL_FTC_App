package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Tracker_Wheel;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@TeleOp(name="reset", group="Teleop")
public class fourbot_reset extends OpMode{
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
    Hanger hanger;
    Arm arm;
    Servo box_left;
    Servo box_right;
    Servo backboard;
    Servo marker;
    boolean flip = false;
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime precision = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    Gimbel g;
    Tracker_Wheel wheel;

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");
        wheel = new Tracker_Wheel(hardwareMap);
        
        hanger = new Hanger(hardwareMap);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        box_left = hardwareMap.get(Servo.class, "box_left");
        box_right = hardwareMap.get(Servo.class, "box_right");
        backboard =  hardwareMap.get(Servo.class, "back");
        marker = hardwareMap.get(Servo.class, "tm");
        box_left.setPosition(0.9);
        box_right.setPosition(0.1);
        backboard.setPosition(0.50);
        marker.setPosition(0.3);

        arm = new Arm(sweeper, arm1, backboard);
        g = new Gimbel(hardwareMap);
        g.GoTo(0.5, 0);

        drive = new Mecanum_Drive(motors, hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"), wheel);
        //drive.enable();
    }

    public void start(){
        telemetry.addData("LEGGO BOYS MAX 2 USUALLY 3: ", "idk what to put here");
        marker.setPosition(0.8);
    }
    public void loop(){
        arm.reset();

        telemetry.addData("Odometer: ", drive.getOdoDistance());
        telemetry.addData("Delta Distance: ", drive.getG_distance());
        telemetry.addData("Arm Pos: ", arm.getArm().getCurrentPosition());
        telemetry.addData("Wheel Pos: ", arm.getSweeper().getCurrentPosition());
        telemetry.addData("Slide Pos: ", hanger.getExtend().getCurrentPosition());
        telemetry.addData("Backboard Pos: ", arm.getBack().getPosition());
        telemetry.addData("Angle:", Math.toDegrees(drive.getRobotHeading()));
        telemetry.addData("Up Left: ", Double.toString(motors[0].getPower()));
        telemetry.addData("Up Right: ", Double.toString(motors[1].getPower()));
        telemetry.addData("Back Left: ", Double.toString(motors[2].getPower()));
        telemetry.addData("Back Right: ", Double.toString(motors[3].getPower()));
        telemetry.addData("X: ", pose.x());
        telemetry.addData("Y: ", pose.y());
        telemetry.addData("Heading: ", Math.toDegrees(pose.heading()));
        telemetry.addData("Time: ", precision.time());
        telemetry.addData("Min: ", drive.getMinimumDistance());
        for (int i = 0; i < 4; i++){
            telemetry.addData("Pos: ", Double.toString((Math.PI * drive.getMotors()[i].getCurrentPosition()) / 140));
        }
    }
    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii("8719", getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }
}

