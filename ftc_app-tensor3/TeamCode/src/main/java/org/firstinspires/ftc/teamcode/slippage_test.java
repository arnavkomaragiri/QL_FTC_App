package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.movement.DriveConstants;
import org.firstinspires.ftc.teamcode.movement.Runner_Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Box;

@Autonomous(name = "Slippage Tester: ", group = "Odometry: ")
public class slippage_test extends OpMode {
    Runner_Mecanum_Drive drive;
    Box b;
    double power = 0.3;
    double coefficient = DriveConstants.slippage_y;

    enum State{
        STATE_TEST,
        STATE_END
    }

    State driveState = State.STATE_END;
    public void init(){
        b = new Box(hardwareMap);
        b.compress();
        drive = new Runner_Mecanum_Drive();
        drive.setDrive(hardwareMap);
        drive.engage();
    }
    public void start(){
        drive.reset();
    }
    public void loop(){
        double dist = drive.getDrive().getPositioner().getY().getDistance();
        if (gamepad1.a){
            drive.reset();
            newState(State.STATE_TEST);
        }
        switch (driveState){
            case STATE_TEST:
                if (dist >= 42){
                    drive.stop();
                    drive.reset();
                    newState(State.STATE_END);
                }
                else{
                    drive.drive(power, Math.PI / 2, 0.0, 0.0);
                }
                break;
            case STATE_END:
                if (gamepad1.dpad_up){
                    power += 0.001;
                }
                else if (gamepad1.dpad_down){
                    power -= 0.001;
                }
                break;
        }
        telemetry.addData("Pos: ", dist);
        telemetry.addData("Power: ", power);
    }

    public void newState(State s){
        driveState = s;
    }
}
