package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.DashboardUtil;
import org.firstinspires.ftc.teamcode.movement.Runner_Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Box;

@Autonomous(name = "TM Spline Test", group = "Roadrunner")
public class spline_test extends OpMode {
    Runner_Mecanum_Drive drive;
    Box b;
    FtcDashboard dash = FtcDashboard.getInstance();
    Trajectory t;

    public void init(){
        b = new Box(hardwareMap);
        b.compress();
        dash.updateConfig();
        drive = new Runner_Mecanum_Drive();
        drive.setDrive(hardwareMap);
        drive.engage();
    }

    public void start(){
        drive.reset();
        t = drive.trajectoryBuilder()
                .splineTo(new Pose2d(48, -48, 0))
                .build();
        drive.followTrajectory(t);
    }

    public void loop(){
        if (drive.isFollowingTrajectory()) {
            drive.updateFollower();
        }
        else{
            drive.drive(0.0, 0.0, 0.0, 0.0);
        }

        /*Pose2d currentPose = drive.getPoseEstimate();

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

        telemetry.addData("Pos: ", drive.getEstimatedPose());
        telemetry.addData("Heading: ", drive.getExternalHeading());
        telemetry.addData("Error: ", drive.getFollowingError());
    }
}
