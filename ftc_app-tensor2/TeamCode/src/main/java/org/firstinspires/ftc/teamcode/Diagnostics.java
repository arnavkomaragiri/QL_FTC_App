package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.control.DStarLite;
import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Box;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Sample_Wall;
import org.firstinspires.ftc.teamcode.wrapper.Tracker_Wheel;
import org.firstinspires.ftc.teamcode.wrapper.sensors.QL_Encoder;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@Autonomous(name="Diagnostics", group="Diagnostics")
public class Diagnostics extends OpMode{
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
    Hanger hanger;
    Arm arm;
    Box box;
    Servo backboard;
    Servo marker;
    Sample_Wall wall;
    boolean mode = false;
    ElapsedTime precision = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    Gimbel g;
    double pos = 0.42;
    Tracker_Wheel wheel;

    private State mDiagnosticState = State.STATE_INTAKE;
    private ElapsedTime mStateTime = new ElapsedTime();

    private void newState(State s){
        mDiagnosticState = s;
        mStateTime.reset();
    }

    private enum State{
        STATE_INTAKE,
        STATE_INTAKE2,
        STATE_EXTEND,
        STATE_FLIP,
        STATE_FLIP2,
        STATE_DRIVE,
        STATE_STOP
    }

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");
        wheel = new Tracker_Wheel(hardwareMap);

        hanger = new Hanger(hardwareMap, true);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        wall = new Sample_Wall(hardwareMap);

        box = new Box(hardwareMap);
        backboard =  hardwareMap.get(Servo.class, "back");
        marker = hardwareMap.get(Servo.class, "tm");
        backboard.setPosition(0.50);
        marker.setPosition(0.3);

        arm = new Arm(sweeper, arm1, backboard, hardwareMap.voltageSensor.get("Motor Controller 2"));
        g = new Gimbel(hardwareMap);
        g.GoTo(0.5, 0);

        drive = new Mecanum_Drive(motors, hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"), wheel);
        //drive.enable();
    }

    public void start(){
        telemetry.addData("LEGGO BOYS MAX 2 USUALLY 3: ", "idk what to put here");
        marker.setPosition(1.0);
    }
    public void loop(){
        switch (mDiagnosticState){
            case STATE_INTAKE:
                arm.diagnose();
                if (mStateTime.time() >= 1.0){
                    newState(State.STATE_INTAKE2);
                }
                break;
            case STATE_INTAKE2:
                arm.diagnose2();
                if (mStateTime.time() >= 1.0) {
                    newState(State.STATE_EXTEND);
                }
                break;
            case STATE_EXTEND:
                if (hanger.diagnose()){
                    newState(State.STATE_FLIP);
                }
                break;
            case STATE_FLIP:
                box.flip(true);
                if (mStateTime.time() >= 0.9){
                    box.init();
                    newState(State.STATE_FLIP2);
                }
                break;
            case STATE_FLIP2:
                if (mStateTime.time() >= 0.9){
                    box.flip(false);
                    if (mStateTime.time() >= 1.8){
                        box.init();
                        newState(State.STATE_DRIVE);
                    }
                }
                break;
            case STATE_DRIVE:
                if (drive.diagnose()){
                    newState(State.STATE_STOP);
                }
                break;
        }

        telemetry.addData("Voltage: ", arm.getVoltage());
        telemetry.addData("Mode: ", (mode ? "Field Centric" : "Robot Centric"));
        telemetry.addData("Odometer: ", drive.getOdoDistance());
        telemetry.addData("Delta Distance: ", drive.getG_distance());
        telemetry.addData("Arm Pos: ", arm.getArmPos());
        telemetry.addData("Wheel Pos: ", arm.getSweeper().getCurrentPosition());
        telemetry.addData("Slide Pos: ", hanger.getExtend().getCurrentPosition());
        telemetry.addData("Extend Pos: ", hanger.getHang().getCurrentPosition());
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

