package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.Statistics;
import org.firstinspires.ftc.teamcode.movement.DriveConstants;
import org.firstinspires.ftc.teamcode.movement.Runner_Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Box;

import java.util.ArrayList;

@Autonomous(name = "Odometry Tester", group = "Odometry")
public class odo_test extends OpMode {
    Runner_Mecanum_Drive drive;
    Box box;
    long previous = 0;
    Statistics wx = new Statistics();
    Statistics wy = new Statistics();
    Statistics tx = new Statistics();
    Statistics ty = new Statistics();
    ArrayList<Double> headings = new ArrayList<Double>();
    ArrayList<Double> xVal = new ArrayList<Double>();
    ArrayList<Double> yVal = new ArrayList<Double>();

    public void init(){
        drive = new Runner_Mecanum_Drive();
        drive.setSmoothing(DriveConstants.smoothing);
        box = new Box(hardwareMap);
        box.compress();
        drive.setDrive(hardwareMap);
        drive.engage();
    }

    public void start(){
        drive.reset();
    }

    public void loop(){
        drive.drive(gamepad1);
        drive.updatePoseEstimate();
        Pose2d w_estimate = drive.getPoseEstimate();
        telemetry.addData("Wheelbase: ", w_estimate);
        Pose2d t_estimate = drive.getEstimatedPose();

        wx.addValue(w_estimate.getX());
        wy.addValue(w_estimate.getY());
        tx.addValue(t_estimate.getX());
        ty.addValue(t_estimate.getY());

        headings.add(drive.getExternalHeading());
        xVal.add(drive.getDrive().getPositioner().getX().getDistance());
        yVal.add(drive.getDrive().getPositioner().getY().getDistance());

        ArrayList<Double> data1 = linear_regression(headings, xVal);
        ArrayList<Double> data2 = linear_regression(headings, yVal);


        double rot = ((drive.getDrive().getMotors()[3].getCurrentPosition() - drive.getDrive().getMotors()[0].getCurrentPosition()) + (drive.getDrive().getMotors()[1].getCurrentPosition() - drive.getDrive().getMotors()[2].getCurrentPosition())) / 4;
        rot *= (3 / 1120);
        rot *= 2 * Math.PI;
        rot /= (9 * Math.sqrt(2));


        telemetry.addData("X Data: ", data1);
        telemetry.addData("Y Data: ", data2);

        telemetry.addData("Positions: ", drive.getWheelPositions());

        telemetry.addData("Kalman Pose: ", drive.k_track());
        telemetry.addData("Averaged Pose: ", drive.a_track());
        telemetry.addData("Low Pass Pose: ", drive.low_track());
        telemetry.addData("Cycle Time: ", System.currentTimeMillis() - previous);
        telemetry.addData("Pos: ", t_estimate);
        telemetry.addData("Heading: ", Math.toDegrees(drive.getExternalHeading()));
        telemetry.addData("W Rotation: ", Math.toDegrees(rot) % 360);
        telemetry.addData("Rotation: ", (drive.getDrive().getPositioner().getRot() / (2 * Math.hypot(12.5, 4) * Math.PI)) * 360);
        telemetry.addData("Distance: ", drive.getDrive().getPositioner().getRot());
        telemetry.addData("W Sigma X: ", wx.calculateStandardDeviation());
        telemetry.addData("W Sigma Y: ", wy.calculateStandardDeviation());
        telemetry.addData("T Sigma X: ", tx.calculateStandardDeviation());
        telemetry.addData("T Sigma Y: ", ty.calculateStandardDeviation());
        previous = System.currentTimeMillis();
    }

    public ArrayList<Double> linear_regression(ArrayList<Double> headings, ArrayList<Double> data){
        double sumx = 0.0, sumy = 0.0, sumx2 = 0.0;
        int n = data.size();
        for (int i = 0; i < n; i++){
            sumx += headings.get(i);
            sumx2 += Math.pow(headings.get(i), 2);
            sumy += data.get(i);
        }
        double xbar = sumx / n;
        double ybar = sumy / n;

        double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
        for (int i = 0; i < n; i++){
            xxbar += (headings.get(i) - xbar) * (headings.get(i) - xbar);
            yybar += (data.get(i) - ybar) * (data.get(i) - ybar);
            xybar += (headings.get(i) - xbar) * (data.get(i) - ybar);
        }
        double slope = xybar / xxbar;
        double intercept = ybar - slope * xbar;
        ArrayList<Double> out = new ArrayList<Double>();
        out.add(slope);
        out.add(intercept);

        double rss = 0.0;
        double ssr = 0.0;
        for (int i = 0; i < n; i++){
            double fit = slope * headings.get(i) + intercept;
            rss += Math.pow(fit - data.get(i), 2);
            ssr += Math.pow(fit - ybar, 2);
        }
        int degreesOfFreedom = n - 2;
        double r2 = ssr / yybar;
        out.add(r2);
        return out;
    }
}
