package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
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
    Servo box_left;
    Servo box_right;
    Servo backboard;
    //Servo marker;
    boolean flip = false;
    boolean mode = false;
    boolean flip2 = false;
    boolean first = true;
    boolean[] previous = {false, false, false};
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime cooldown2 = new ElapsedTime();
    ElapsedTime precision = new ElapsedTime();
    ElapsedTime flip_time = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    Gimbel g;
    double pos = 0.42;
    Tracker_Wheel wheel;

    private enum State{
        STATE_LAUNCH,
        STATE_BANK,
        STATE_END
    }

    private State mBoxState = State.STATE_END;

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");
        wheel = new Tracker_Wheel(hardwareMap);
        
        hanger = new Hanger(hardwareMap, true);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        box_left = hardwareMap.get(Servo.class, "box_left");
        box_right = hardwareMap.get(Servo.class, "box_right");
        backboard =  hardwareMap.get(Servo.class, "back");
        //marker = hardwareMap.get(Servo.class, "tm");
        box_left.setPosition(0.8);
        box_right.setPosition(0.2);
        backboard.setPosition(0.50);
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
        if (!mode) {
            drive.drive(gamepad1);
        }
        else{
            drive.drive(gamepad1);
        }

        if (gamepad1.a){
            if (mode){
                mode = false;
            }
            else{
                mode = true;
            }
        }
        else if (gamepad1.b){
            telemetry.addData("Arm Successfully Recalibrated: "
                    , arm.getArmPos());
            arm.recalibrate();
        }

        hanger.operate(gamepad2);
        arm.move(gamepad2);

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

        if (isPress(gamepad2.left_bumper)){// && cooldown.time() > 0.25){
            if (!flip){
                if (arm.getBState()) {
                    box_left.setPosition(0.0);
                    box_right.setPosition(1.0);
                    newState(State.STATE_END);
                }
                else{
                    box_left.setPosition(0.025);
                    box_right.setPosition(0.975);
                }
                flip = true;
            }
            else {
                box_left.setPosition(0.8);
                box_right.setPosition(0.2);
                newState(State.STATE_END);
                flip = false;
            }
            cooldown.reset();
        }
        if (isPress(gamepad2.right_bumper,1)){// && cooldown2.time() > 0.25){
            if (!flip2) {
                box_left.setPosition(0.6);
                box_right.setPosition(0.4);
                newState(State.STATE_END);
                flip2 = true;
            }
            else{
                box_left.setPosition(0.8);
                box_right.setPosition(0.2);
                newState(State.STATE_END);
                flip2 = false;
            }
            cooldown2.reset();
        }

        switch (mBoxState){
            case STATE_LAUNCH:
                if (first) {
                    box_left.setPosition(0.05);
                    box_right.setPosition(0.95);
                    first = false;
                }
                if (flip_time.time() >= 1.0){
                    newState(State.STATE_BANK);
                    first = true;
                }
                break;
            case STATE_BANK:
                if (first) {
                    box_left.setPosition(0.0);
                    box_right.setPosition(1.0);
                    first = false;
                }
                if (flip_time.time() >= 0.5){
                    newState(State.STATE_END);
                    first = true;
                }
                break;
        }

        telemetry.addData("Voltage: ", arm.getVoltage());
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

    private void newState(State s){
        mBoxState = s;
        flip_time.reset();
    }
    private boolean isPress(boolean input){
        boolean result = false;
        if (!previous[0] && input){
            result = true;
        }
        else{
            result = false;
        }
        previous[0] = input;
        return result;
    }
    private boolean isPress(boolean input, int mode){
        boolean result = false;
        if (mode == 1) {
            if (!previous[mode] && input) {
                result = true;
            } else if (previous[mode] && !input){
                result = false;
            }
            previous[mode] = input;
        }
        else if (mode == 0){
            if (!previous[0] && input){
                result = true;
            }
            else{
                result = false;
            }
            previous[0] = input;
        }
        return result;
    }
    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii("8719", getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }
}

