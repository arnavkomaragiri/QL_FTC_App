package org.firstinspires.ftc.teamcode.movement;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.control.QL_Vector2d;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class Runner_Mecanum_Drive extends MecanumDrive {
    TrajectoryFollower follower;
    Tracker_Mecanum_Drive drive;

    PIDCoefficients HEADING_PID;
    PIDCoefficients TRANSLATIONAL_PID;

    DriveConstraints BASE_CONSTRAINTS;

    protected static HardwareMap hardwareMap;

    private List<Double> x_val = new ArrayList<Double>();
    private List<Double> y_val = new ArrayList<Double>();

    private double TOP_SPEED = 67.02;

    private double previousTime = 0.0;

    private double smoothing = DriveConstants.smoothing;
    private Pose2d previous = new Pose2d();
    private Vector2d velocity = new Vector2d();
    private long previousMS = System.currentTimeMillis();


    public final double K = (17 + 17) / 4;

    protected static double k_sigma_x = 0.33; //todo: tune these values
    protected static double k_sigma_y = 0.42;
    protected static double w_sigma_x = 0.22;
    protected static double w_sigma_y = 0.6;

    protected static double kalman_gain_x = (Math.pow(w_sigma_x, 2)) / (Math.pow(k_sigma_x, 2) + Math.pow(w_sigma_x, 2));
    protected static double kalman_gain_y = (Math.pow(w_sigma_y, 2)) / (Math.pow(k_sigma_y, 2) + Math.pow(w_sigma_y, 2));

    public void setHardwareMap(HardwareMap h){
        hardwareMap = h;
    }

    public Runner_Mecanum_Drive(){
        super(DriveConstants.TRACK_WIDTH);
        //drive = new Tracker_Mecanum_Drive(hardwareMap);
        TRANSLATIONAL_PID = DriveConstants.TRANSLATIONAL_PID;
        HEADING_PID = DriveConstants.HEADING_PID;
        BASE_CONSTRAINTS = DriveConstants.BASE_CONSTRAINTS;
        follower = new MecanumPIDVAFollower(this, TRANSLATIONAL_PID, HEADING_PID, DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
    }

    public void setDrive(HardwareMap h){
        drive = new Tracker_Mecanum_Drive(h);
    }

    public void setDrive(Tracker_Mecanum_Drive drive){
        this.drive = drive;
    }

    public void engage(){
        drive.engage();
    }

    public void disengage(){
        drive.disengage();
    }

    public Tracker_Mecanum_Drive getDrive(){
        return this.drive;
    }

    public void drive(Gamepad g){
        drive.drive(g);
    }

    public void drive(double r, double angle, double rightX, double rightY){
        drive.drive(r, angle, rightX, rightY);
    }

    public void f_drive(Gamepad g){
        drive.f_drive(g);
    }

    public void reset(){
        drive.getPositioner().reset();
        previous = new Pose2d();
    }

    @Override
    public double getExternalHeading(){
        return Math.toRadians((0.996864 * ((0.985697 * ((0.969219 * drive.getGyro().getHeading())) + 0.486599) - 0.662445)) + 0.182906);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        /*for (DcMotor motor : drive.getMotors()) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.getCurrentPosition()));
        }*/
        wheelPositions.add(DriveConstants.encoderTicksToInches(drive.getMotors()[0].getCurrentPosition()));
        wheelPositions.add(DriveConstants.encoderTicksToInches(drive.getMotors()[3].getCurrentPosition()));
        wheelPositions.add(DriveConstants.encoderTicksToInches(drive.getMotors()[1].getCurrentPosition()));
        wheelPositions.add(DriveConstants.encoderTicksToInches(drive.getMotors()[2].getCurrentPosition()));
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drive.setPowers(v, v1, v2, v3);
    }

    public Pose2d getEstimatedPose(){
        /*double deltaTime = (System.nanoTime() - previousTime);
        QL_Vector2d v = new QL_Vector2d(Math.cos(drive.getRobotHeading()), Math.sin(drive.getRobotHeading()));
        v.multiplied(drive.getRobotPower() * TOP_SPEED * deltaTime);
        previousTime = (System.nanoTime());a*/
        //return new Pose2d(drive.getPositioner().track().x(), drive.getPositioner().track().y(), getExternalHeading());
        Pose2d estimate = k_track();
        //return new Pose2d(-estimate.getY(), estimate.getX(), getExternalHeading());
        return new Pose2d(drive.getPositioner().track().y(), -drive.getPositioner().track().x(), getExternalHeading());
    }

    public boolean goTo(Pose2d p, double power, double r){
        Pose2d pos = getEstimatedPose();
        double angle = Math.atan2(p.getY() - pos.getX(), p.getX() - pos.getY()) - pos.getHeading();
        if (p.pos().distanceTo(pos.pos()) < r){
            drive(0.0, 0.0, 0.0, 0.0);
            return true;
        }
        else {
            drive(power, angle, 0.0, 0.0);
            return false;
        }
    }

    public Pose2d k_track(){
        long deltaTime = System.nanoTime() - previousMS;
        Pose2d p1 = previous.plus(new Pose2d(velocity.times(deltaTime), 0));
        updatePoseEstimate();
        p1 = getPoseEstimate();
        p1 = new Pose2d(p1.getY(), p1.getX(), getExternalHeading());
        Vector2d wheel_offset = p1.pos().minus(previous.pos());
        Pose2d p2 = new Pose2d(drive.getPositioner().track().x(), drive.getPositioner().track().y(), getExternalHeading());
        Vector2d tracker_offset = p2.pos().minus(previous.pos());
        Pose2d pos;

        Vector2d error = tracker_offset.minus(wheel_offset);
        Pose2d t_offset = new Pose2d(wheel_offset.getX() + (kalman_gain_x * error.getX()), wheel_offset.getY() + (kalman_gain_y * error.getY()), 0);
        Pose2d p3 = previous.plus(t_offset);
        if (Math.hypot(p3.getX() - p2.getX(), p3.getY() - p2.getY()) < Math.hypot(w_sigma_x, w_sigma_y)){
            pos = new Pose2d(p3.getX(), p3.getY(), getExternalHeading());
        }
        else{
            pos = new Pose2d(p2.getX(), p2.getY(), getExternalHeading());
            setPoseEstimate(pos);
        }
        velocity = (p2.pos().minus(previous.pos())).div(deltaTime);
        previous = pos;
        previousMS = System.nanoTime();
        return pos;
    }

    public Pose2d a_track(){
        Pose2d p1 = new Pose2d(drive.getPositioner().track().x(), drive.getPositioner().track().y(), getExternalHeading());
        x_val.add(p1.getX());
        y_val.add(p1.getY());
        if (x_val.size() > DriveConstants.sample_size){
            for (int i = 0; i < x_val.size() - DriveConstants.sample_size; i++) {
                x_val.remove(0);
            }
        }
        if (y_val.size() > 5){
            for (int i = 0; i < y_val.size() - DriveConstants.sample_size; i++) {
                y_val.remove(0);
            }
        }
        Pose2d output = new Pose2d(average(x_val), average(y_val), getExternalHeading());
        //drive.getPositioner().setPose(output.getX(), output.getY(), output.getHeading());
        return output;
    }

    public void setSmoothing(double smoothing){
        this.smoothing = smoothing;
    }

    public Pose2d low_track(){
        Pose2d p1 = new Pose2d(drive.getPositioner().track().y(), drive.getPositioner().track().x(), getExternalHeading());
        long deltaTime = System.nanoTime() - previousMS;
        Vector2d out = previous.pos().plus(p1.pos().minus(previous.pos()).times(smoothing / deltaTime));
        previous = new Pose2d(out, getExternalHeading());
        previousMS = System.nanoTime();
        return previous;
    }

    private double average(List<Double> data){
        double sum = 0.0;
        for (int i = 0; i < data.size(); i++){
            sum += data.get(i);
        }
        return sum / data.size();
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getEstimatedPose(), BASE_CONSTRAINTS);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void updateFollower(){
        //updatePoseEstimate();
        follower.update(getEstimatedPose());
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }
    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

    public Pose2d getPoseDelta(double[] rot) {
        if (rot.length != 4) {
            throw new IllegalArgumentException("length must be four");
        }
        double RADIUS = 2;
        double x = RADIUS * (rot[0] + rot[1] - rot[2] - rot[3]) / 4;
        double y = RADIUS * (rot[0] - rot[1] + rot[2] - rot[3]) / 4;
        double h = RADIUS * (-rot[0] - rot[1] - rot[2] - rot[3]) / (4 * K);
        return new Pose2d(x, y, h);
    }
}
