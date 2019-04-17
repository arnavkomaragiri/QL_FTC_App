package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.movement.Runner_Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Box;

@Autonomous(name = "Straight Right Test", group = "Roadrunner")
public class straight_right_test extends OpMode {
    Runner_Mecanum_Drive drive;
    Box box;
    //FtcDashboard dash = FtcDashboard.getInstance();
    Trajectory t;
    long previous = 0;

    public void init(){
        //dash.updateConfig();
        box = new Box(hardwareMap);
        box.compress();
        drive = new Runner_Mecanum_Drive();
        drive.setDrive(hardwareMap);
        drive.engage();
    }

    public void start(){
        drive.reset();
        t = drive.trajectoryBuilder()
                .strafeRight(48)
                .build();
        drive.followTrajectory(t);
    }

    public void loop(){
        if (drive.isFollowingTrajectory() && !drive.getDrive().getGyro().isCalibrating()){
            drive.updateFollower();
            telemetry.addData("Error: ", drive.getFollowingError());
        }
        else{
            drive.drive(gamepad1);
            if (gamepad1.x){
                //dash.updateConfig();
                drive = new Runner_Mecanum_Drive();
                drive.engage();
                drive.reset();
                drive.followTrajectory(t);
            }
        }

        /*Pose2d currentPose = drive.getEstimatedPose();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        fieldOverlay.setStrokeWidth(4);
        fieldOverlay.setStroke("green");
        DashboardUtil.drawSampledTrajectory(fieldOverlay, t);

        fieldOverlay.setFill("blue");
        fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

        dash.sendTelemetryPacket(packet);*/

        telemetry.addData("Error: ", drive.getFollowingError());
        telemetry.addData("Wheel Pos: ", drive.getPoseEstimate());
        telemetry.addData("Cycle Time: ", System.currentTimeMillis() - previous);
        telemetry.addData("Pos: ", drive.getEstimatedPose());
        telemetry.addData("Heading: ", Math.toDegrees(drive.getExternalHeading()));
        previous = System.currentTimeMillis();
    }
}
