package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
import org.firstinspires.ftc.teamcode.movement.Two_Axis_Localizer;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Box;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Sample_Wall;
import org.firstinspires.ftc.teamcode.wrapper.Slide;
import org.firstinspires.ftc.teamcode.wrapper.Tracker_Wheel;
import org.firstinspires.ftc.teamcode.wrapper.sensors.QL_Encoder;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@TeleOp(name="four_bot_tele", group="Teleop")
public class fourbot_tele extends OpMode{
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
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
    Two_Axis_Localizer localizer;
    boolean fState = false;

    long previous_time = System.currentTimeMillis();

    double rot = 0.0;

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");
        wheel = new Tracker_Wheel(hardwareMap);
        encoder = new QL_Encoder(hardwareMap.get(AnalogInput.class, "x_tracker"));
        encoder.setRatio(1.0);
        
        hanger = new Hanger(hardwareMap, true);

        localizer = new Two_Axis_Localizer(hardwareMap);

        temp_tensioner = hardwareMap.get(Servo.class, "x_tension");
        temp_tensioner.setPosition(1.0);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        wall = new Sample_Wall(hardwareMap);

        box = new Box(hardwareMap);
        backboard =  hardwareMap.get(Servo.class, "back");
        marker = hardwareMap.get(Servo.class, "tm");
        backboard.setPosition(0.50);
        marker.setPosition(0.3);
        slide = new Slide(hardwareMap);

        arm = new Arm(sweeper, arm1, backboard, hardwareMap.voltageSensor.get("Motor Controller 2"), hardwareMap.get(CRServo.class, "intake_left"), hardwareMap.get(CRServo.class, "intake_right"));
        g = new Gimbel(hardwareMap);
        g.GoTo(0.5, 0);

        drive = new Mecanum_Drive(motors, hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"), wheel);
        //drive.enable();
        localizer.disengage();
    }

    public void start(){
        telemetry.addData("LEGGO BOYS MAX 2 USUALLY 3: ", "idk what to put here");
        marker.setPosition(1.0);
        temp_tensioner.setPosition(0.0);
    }
    public void loop(){
        new Runnable(){
            public void run(){
                if (!mode) {
                    drive.drive(gamepad1);
                }
                else{
                    drive.drive(gamepad1);
                }
            }
        }.run();

        if (false){
            if (mode){
                mode = false;
            }
            else{
                mode = true;
                drive.calibrate();
            }
        }
        else if (gamepad1.b){
            telemetry.addData("Arm Successfully Recalibrated: "
                    , arm.getArmPos());
            arm.recalibrate();
        }

        //hanger.operate(gamepad2, gamepad1);
        new Runnable(){
            public void run(){
                arm.move(gamepad2);
            }
        }.run();
        //box.operate(gamepad2, arm.getBState());
        new Runnable(){
            public void run(){
                slide.operate(gamepad2, arm.getBState(), fState, telemetry, gamepad1);
            }
        }.run();

        if (gamepad1.right_bumper){
            marker.setPosition(0.3);
        }
        if (gamepad1.dpad_up){
            localizer.disengage();
        }
        else if (gamepad1.dpad_down){
            localizer.engage();
        }

        if (gamepad1.dpad_left){
            wall.engage();
        }
        else if (gamepad1.dpad_right){
            wall.disengage();
        }

        if (precision.time() >= 0.125) {
            pose = drive.k_track();
            precision.reset();
        }
        if (gamepad1.b){
            drive.motor_reset();
        }

        if (slide.getState() == Slide.State.STATE_CONTRACT){
            fState = false;
        }
        /*if (gamepad1.dpad_left){
            marker.setPosition(0.3);
        }
        else if (gamepad1.dpad_right){
            marker.setPosition(0.8);
        }*/

        telemetry.addData("Mode: ", !fState ? (arm.getBState() ? "SILVER" : "GOLD") : "MIX");
        telemetry.addData("Cycle Time: ", System.currentTimeMillis() - previous_time);
        previous_time = System.currentTimeMillis();
        telemetry.addData("Filter Pos: ", slide.getBox().getFilter().getPosition());
        telemetry.addData("X Pos: ", encoder.getDistance());
        telemetry.addData("Voltage: ", arm.getVoltage());
        telemetry.addData("Mode: ", (mode ? "Field Centric" : "Robot Centric"));
        telemetry.addData("Odometer: ", drive.getOdoDistance());
        telemetry.addData("Delta Distance: ", drive.getG_distance());
        telemetry.addData("Arm Pos: ", arm.getArmPos());
        telemetry.addData("Sweeper Pos: ", arm.getSweeper().getCurrentPosition());
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
            telemetry.addData("Pos: ", Double.toString((3 * drive.getMotors()[i].getCurrentPosition())));
        }
    }

    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii("8719", getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }

    private boolean isPress(boolean input){
        boolean result = false;
        if (!previous && input){
            result = true;
        }
        else{
            result = false;
        }
        previous = input;
        return result;
    }
}

