package org.firstinspires.ftc.teamcode.movement;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.control.Statistics;
import org.firstinspires.ftc.teamcode.control.Vector2d;
import org.firstinspires.ftc.teamcode.wrapper.Tracker_Wheel;

public class Two_Axis_Localizer {
    private Tracker_Wheel y;
    private Tracker_Wheel x;
    private ModernRoboticsI2cGyro gyro;

    private double rot = 0.0;
    private double previous = 0.0;

    private double previous_x = 0.0;
    private double previous_y = 0.0;

    private Pose2d pos = new Pose2d(0.0, 0.0, 0.0);

    protected static double k_sigma_x = 2; //todo: tune these values
    protected static double k_sigma_y = 2;
    protected static double w_sigma_x = 0.5;
    protected static double w_sigma_y = 0.5;

    private Statistics k_std_x = new Statistics();
    private Statistics k_std_y = new Statistics();
    private Statistics w_std_x = new Statistics();
    private Statistics w_std_y = new Statistics();

    protected static double kalman_gain_x = (Math.pow(w_sigma_x, 2)) / (Math.pow(k_sigma_x, 2) + Math.pow(w_sigma_x, 2));
    protected static double kalman_gain_y = (Math.pow(w_sigma_y, 2)) / (Math.pow(k_sigma_y, 2) + Math.pow(w_sigma_y, 2));

    public double R = Math.hypot(12.5, 4.5);

    private enum Position{
        R1,
        R2,
        B1,
        B2
    }

    public Two_Axis_Localizer(HardwareMap h){
        y = new Tracker_Wheel(h);
        x = new Tracker_Wheel(h, "x_tracker");
        x.setRatio(-2.0);
        x.setInverted(true);
        gyro = (ModernRoboticsI2cGyro)h.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    public Pose2d getPose(){
        return this.pos;
    }

    public void setPose(Pose2d p){
        pos = p;
    }

    public Tracker_Wheel getX(){
        return x;
    }

    public Tracker_Wheel getY(){
        return y;
    }

    public void setPose(double x, double y, double heading){
        pos = new Pose2d(x, y, heading);
    }

    public Pose2d track(){
        double rotation = getRot() * (Math.PI / 180);
        double dx = (x.getDistance() - compute_rot_x(rotation)) - previous_x;
        dx *= DriveConstants.slippage_x;
        double dy = (y.getDistance() - compute_rot_y(rotation)) - previous_y;
        dy *= DriveConstants.slippage_y;
        Vector2d offset = new Vector2d(dx, dy);
        offset.rotated(getHeading());

        pos = new Pose2d(pos.x() + offset.x(), pos.y() + offset.y(), gyro.getHeading());
        previous_x = x.getDistance() - compute_rot_x(rotation);
        previous_y = y.getDistance() - compute_rot_y(rotation);
        return pos;
    }

    public double getHeading(){
        return (0.996864 * ((0.985697 * ((0.969219 * this.gyro.getHeading())) + 0.486599) - 0.662445)) + 0.182906;
    }

    public double compute_rot_x(double heading){
        return 16.35 * heading - 0.05;
    }

    public double compute_rot_x_deg(){
        return ((0.2853613327 * getRot()) - 0.05);
    }

    public double compute_rot_y(double heading){
        return -2.83 * heading + 0.01;
    }

    public double compute_rot_y_deg(){
        return ((-0.0493928178 * getRot()) + 0.01);
    }

    public Pose2d k_track(Vector2d move){
        Pose2d p1 = new Pose2d(pos.pos().added(move), this.gyro.getHeading());
        Pose2d p2 = track();
        Vector2d error = p2.added(p1.multiplied(-1)).pos();
        Pose2d p3 = new Pose2d(p1.x() + (kalman_gain_x * error.x()), p1.y() + (kalman_gain_y * error.y()), this.gyro.getHeading());
        if (p3.dist(p2) < Math.hypot(w_sigma_x, w_sigma_y)){
            pos = p3;
        }
        else{
            pos = p2;
        }
        return pos;
    }

    public Pose2d getFieldCentricPose(Position f){
        Pose2d p;
        switch (f){
            case R1:
                p = new Pose2d(pos.pos().rotated(135), (this.gyro.getHeading() + 135) % 360);
                break;
            case R2:
                p = new Pose2d(pos.pos().rotated(225), (this.gyro.getHeading() + 225) % 360);
                break;
            case B1:
                p = new Pose2d(pos.pos().rotated(315), (this.gyro.getHeading() + 315) % 360);
                break;
            case B2:
                p = new Pose2d(pos.pos().rotated(45), (this.gyro.getHeading() + 45) % 360);
                break;
            default:
                p = pos;
                break;
        }
        return p;
    }

    public void disengage(){
        x.disengage();
        y.disengage();
    }

    public void engage(){
        x.engage();
        y.engage();
    }

    public void reset(){
        x.reset();
        y.reset();
        pos = new Pose2d(0.0, 0.0, 0.0);
    }

    public double getRot(){
        double heading = getHeading();
        if ((heading - previous) < -270){
            rot += (360 + (heading - previous)) / 360;
        } else if((heading - previous) > 270){
            rot -= (360 + (previous - heading)) / 360;
        }
        else{
            rot += (heading - previous) / 360;
        }
        previous = heading;

        return rot * 360;
    }

}
