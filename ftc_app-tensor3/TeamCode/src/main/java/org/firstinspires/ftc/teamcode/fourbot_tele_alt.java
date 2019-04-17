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
import org.firstinspires.ftc.teamcode.wrapper.Box;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Slide;
import org.firstinspires.ftc.teamcode.wrapper.Tracker_Wheel;

/**
 * Created by arnav on 10/22/2017.
 */
//@Disabled
@TeleOp(name="ALTERNATE_four_bot", group="Alternate")
public class fourbot_tele_alt extends OpMode{
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
    Hanger hanger;
    Arm arm;
    Box box;
    Servo backboard;
    Slide slide;
    //Servo marker;
    boolean flip = false;
    boolean mode = false;
    boolean flip2 = false;
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime cooldown2 = new ElapsedTime();
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
        
        hanger = new Hanger(hardwareMap, true);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        backboard =  hardwareMap.get(Servo.class, "back");
        //marker = hardwareMap.get(Servo.class, "tm");
        slide = new Slide(hardwareMap);
        slide.setAlt(true);

        backboard.setPosition(0.50);
        box = new Box(hardwareMap);
        box.setAlt(true);
        //marker.setPosition(0.3);

        arm = new Arm(sweeper, arm1, backboard, hardwareMap.voltageSensor.get("Motor Controller 2"));
        g = new Gimbel(hardwareMap);
        g.GoTo(0.5, 0);

        drive = new Mecanum_Drive(motors, hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"), wheel);
        //drive.enable();
    }

    public void start(){
        telemetry.addData("LEGGO BOYS MAX 2 USUALLY 3: ", "idk what to put here");
        //marker.setPosition(1.0);
    }
    public void loop(){
        if (!drive.getGyro().isCalibrating()) {
            if (!mode) {
                drive.f_drive(gamepad1);
            } else {
                drive.f_drive(gamepad1);
            }
        }

        if (false){
            if (mode){
                mode = false;
            }
            else{
                mode = true;
                //drive.calibrate();
            }
        }
        else if (gamepad1.b){
            telemetry.addData("Arm Successfully Recalibrated: "
                    , arm.getArmPos());
            arm.recalibrate();
        }

        //hanger.operate(gamepad2, gamepad1);
        arm.move(gamepad2);
        //box.operate(gamepad2, arm.getBState());
        slide.operate(gamepad2, arm.getBState(), telemetry, gamepad1);

        if (gamepad1.dpad_up){
            wheel.disengage();
        }
        else if (gamepad1.dpad_down){
            wheel.engage();
        }

        if (precision.time() >= 0.125) {
            pose = drive.k_track();
            precision.reset();
        }
        if (gamepad1.b){
            drive.motor_reset();
        }
        /*if (gamepad1.dpad_left){
            marker.setPosition(0.3);
        }
        else if (gamepad1.dpad_right){
            marker.setPosition(0.8);
        }*/

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
            telemetry.addData("Pos: ", Double.toString((3 * drive.getMotors()[i].getCurrentPosition())));
        }
    }
    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii("8719", getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }
}

