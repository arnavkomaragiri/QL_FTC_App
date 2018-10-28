package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Sentry;

public class QL_Auto_R1 extends OpMode {
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
    Hanger hanger;
    Arm arm;
    Servo box_left;
    Servo box_right;
    boolean flip = false;
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime precision = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    Sentry s;
    int pos = -1;

    State mRobotState = State.STATE_START;
    ElapsedTime mStateTime = new ElapsedTime();

    public enum State{
        STATE_START,
        STATE_DROP,
        STATE_SCAN,
        STATE_SAMPLE,
        STATE_TRAVEL,
        STATE_CLAIM,
        STATE_RECENTER,
        STATE_ENTER,
        STATE_COLLECT,
        STATE_EXIT,
        STATE_RETURN,
        STATE_SCORE,
        STATE_PARK,
        STATE_STOP
    }

    public void init(){
        motors[0] = hardwareMap.dcMotor.get("up_left");
        motors[1] = hardwareMap.dcMotor.get("up_right");
        motors[2] = hardwareMap.dcMotor.get("back_left");
        motors[3] = hardwareMap.dcMotor.get("back_right");

        hanger = new Hanger(hardwareMap);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        box_left = hardwareMap.get(Servo.class, "box_left");
        box_right = hardwareMap.get(Servo.class, "box_right");
        box_left.setPosition(0.85);
        box_right.setPosition(0.15);

        arm = new Arm(sweeper, arm1);
        s = new Sentry(hardwareMap);


        drive = new Mecanum_Drive(motors, hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"));
    }

    public void start(){
        newState(State.STATE_START);
        s.start();
    }

    public void loop(){
        switch (mRobotState){
            case STATE_START:
                telemetry.addData("Entering Auto", "1");
                newState(State.STATE_SCAN);
                break;
            case STATE_DROP:
                //insert drop code once hanging is complete
                break;
            case STATE_SCAN:
                if (s.lockOn()){
                    pos = s.getPosition(telemetry);
                    telemetry.addData("Moving to State Sample", pos);
                    newState(State.STATE_SAMPLE);
                }
                else{
                    telemetry.addData("Unable to find sample", pos);
                    if (mStateTime.time() >= 4.0){
                        telemetry.addData("Unable to find sample, moving to marker", pos);
                        newState(State.STATE_STOP); //replace with state claim
                    }
                }
                break;
        }
    }

    public void newState(State s){
        mRobotState = s;
        mStateTime.reset();
    }
}
