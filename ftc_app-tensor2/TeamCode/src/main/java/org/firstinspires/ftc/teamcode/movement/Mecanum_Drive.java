package org.firstinspires.ftc.teamcode.movement;

import android.content.Context;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.control.Vector2d;
import org.firstinspires.ftc.teamcode.wrapper.Tracker_Wheel;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

public class Mecanum_Drive{
    private DcMotor motors[] = new DcMotor[4];
    private ModernRoboticsI2cGyro gyro;
    private Tracker_Wheel odometer;
    private ElapsedTime cooldown = new ElapsedTime();
    private double modifier = 1.0;

    private Pose2d p = new Pose2d(0, 0, 0);
    private Pose2d memo = new Pose2d(0, 0, 0);
    private double[] prevpos = new double[5];
    private double robotHeading = 0.0;
    private double f_heading = 0.0;
    private double primary = 1.0;
    private double primaryAngle = 0.0;
    private double avgHeading = 0.0;
    private long sum = 0;
    private long count = 1;
    double g_distance = 0.0;

    private boolean first = true;

    public Mecanum_Drive(DcMotor names[], ModernRoboticsI2cGyro gyro, Tracker_Wheel odometer){
        motors = names.clone();
        this.gyro = gyro;
        this.gyro.calibrate();
        this.odometer = odometer;
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
    }

    public Mecanum_Drive(HardwareMap h){
        this.gyro = (ModernRoboticsI2cGyro)h.gyroSensor.get("gyro");
        //this.gyro.calibrate();
        this.odometer = new Tracker_Wheel(h);
        motors[0] = h.get(DcMotor.class, "up_left");
        motors[1] = h.get(DcMotor.class, "up_right");
        motors[2] = h.get(DcMotor.class, "back_left");
        motors[3] = h.get(DcMotor.class, "back_right");
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotors(DcMotor names[]){
        motors = names.clone();
    }

    public DcMotor[] getMotors(){
        return motors.clone();
    }

    public String toString(){
        String output = "";
        output += Double.toString(motors[0].getPower());
        output += " " + Double.toString(motors[1].getPower());
        output += " " + Double.toString(motors[2].getPower());
        output += " " + Double.toString(motors[3].getPower());

        return output;
    }

    public void calibrate(){
        odometer.reset();
        this.gyro.calibrate();
    }

    public void setPos(Pose2d p){
        this.p = p;
    }

    public Pose2d getPos(){
        return p;
    }

    public void setModifier(double d){
        this.modifier = d;
    }

    public double getModifier(){
        return this.modifier;
    }

    public void loadModifier(Context c){
        modifier = Double.parseDouble(readFromFile(c));
    }

    private String readFromFile(Context context) {
        String ret = "";

        try {
            InputStream inputStream = context.openFileInput("modifier.txt");

            if ( inputStream != null ) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString = "";
                StringBuilder stringBuilder = new StringBuilder();

                while ( (receiveString = bufferedReader.readLine()) != null ) {
                    stringBuilder.append(receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        }
        catch (FileNotFoundException e) {
            RobotLog.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            RobotLog.e("login activity", "Can not read file: " + e.toString());
        }

        return ret;
    }

    public void drive(Gamepad gamepad1) {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        robotHeading = angle;
        double robotAngle = angle - Math.PI / 4;
        double max = 0.0;

        Double[] v = {0.0, 0.0, 0.0, 0.0};

        v[0] = (r * Math.cos(robotAngle)) + rightX;
        v[1] = (r * Math.sin(robotAngle)) - rightX;
        v[2] = (r * Math.sin(robotAngle)) + rightX;
        v[3] = (r * Math.cos(robotAngle)) - rightX;

        for (int i = 0; i < 4; i++) {
            if (Math.abs(v[i]) > max) {
                max = Math.abs(v[i]);
            }
        }
        if (max == 0) {
            max = 1;
        }
        //System.out.println();
        double scale = ((r == 0) ? Math.hypot(rightX, rightY) : r) / ((max == 0) ? ((r == 0) ? Math.hypot(rightX, rightY) : r) : max);
        //System.out.println(scale);
        //System.out.println();

        for (int i = 0; i < 4; i++) {
            v[i] *= scale;
            //System.out.println(v[i]);
            v[i] = Range.clip(v[i], -1.0, 1.0);
            motors[i].setPower(v[i]);
        }
    }

    public void f_drive(Gamepad gamepad1){
        f_heading = this.gyro.getHeading() - 45;
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.toRadians(f_heading);
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        robotHeading = angle;
        double robotAngle = angle - Math.PI / 4;
        double max = 0.0;

        Double[] v = {0.0, 0.0, 0.0, 0.0};

        v[0] = (r * Math.cos(robotAngle)) + rightX;
        v[1] = (r * Math.sin(robotAngle)) - rightX;
        v[2] = (r * Math.sin(robotAngle)) + rightX;
        v[3] = (r * Math.cos(robotAngle)) - rightX;

        for (int i = 0; i < 4; i++) {
            if (Math.abs(v[i]) > max) {
                max = Math.abs(v[i]);
            }
        }
        if (max == 0) {
            max = 1;
        }
        //System.out.println();
        double scale = ((r == 0) ? Math.hypot(rightX, rightY) : r) / ((max == 0) ? ((r == 0) ? Math.hypot(rightX, rightY) : r) : max);
        //System.out.println(scale);
        //System.out.println();

        for (int i = 0; i < 4; i++) {
            v[i] *= scale;
            //System.out.println(v[i]);
            v[i] = Range.clip(v[i], -1.0, 1.0);
            motors[i].setPower(v[i]);
        }
    }

    public boolean turn(double heading, Telemetry t){
        double e = heading - this.gyro.getIntegratedZValue();
        boolean result = false;
        t.addData("E: ", e);
        if (e >= 5){
            t.addData("Direction Forward: ", e);
            drive(0.0, 0.0, -0.5, 0.0);
        }
        else if (e <= -5){
            t.addData("Direction Backward: ", e);
            drive(0.0, 0.0, 0.4,0.0);
        }
        else{
            drive(0.0, 0.0, 0.0, 0.0);
            motor_reset();
            odometer.reset();
            result = true;
        }
        return result;
    }

    public boolean turn(double heading, double r, Telemetry t){
        double e = heading - this.gyro.getIntegratedZValue();
        boolean result = false;
        t.addData("E: ", e);
        if (e >= r){
            t.addData("Direction Forward: ", e);
            drive(0.0, 0.0, -0.3, 0.0);
        }
        else if (e <= -r){
            t.addData("Direction Backward: ", e);
            drive(0.0, 0.0, 0.3,0.0);
        }
        else{
            drive(0.0, 0.0, 0.0, 0.0);
            motor_reset();
            odometer.reset();
            result = true;
        }
        return result;
    }
    public boolean turn(double heading, double r, double speed, Telemetry t){
        double e = heading - this.gyro.getIntegratedZValue();
        boolean result = false;
        t.addData("E: ", e);
        if (e >= r){
            t.addData("Direction Forward: ", e);
            drive(0.0, 0.0, -speed, 0.0);
        }
        else if (e <= -r){
            t.addData("Direction Backward: ", e);
            drive(0.0, 0.0, speed,0.0);
        }
        else{
            drive(0.0, 0.0, 0.0, 0.0);
            motor_reset();
            odometer.reset();
            result = true;
        }
        return result;
    }

    public void drive(double r, double angle, double rightX, double rightY) {
        robotHeading = angle;
        double robotAngle = angle - Math.PI / 4;
        double max = -1.0;

        Double[] v = {0.0, 0.0, 0.0, 0.0};

        v[0] = (r * Math.cos(robotAngle)) + rightX;
        v[1] = (r * Math.sin(robotAngle)) - rightX;
        v[2] = (r * Math.sin(robotAngle)) + rightX;
        v[3] = (r * Math.cos(robotAngle)) - rightX;

        for (int i = 0; i < 4; i++) {
            if (v[i] > max) {
                max = Math.abs(v[i]);
            }
        }
        if (max == 0) {
            max = 1;
        }
        //System.out.println();
        double scale = ((r == 0) ? Math.hypot(rightX, rightY) : r) / ((max == 0) ? ((r == 0) ? Math.hypot(rightX, rightY) : r) : max);
        scale = Math.abs(scale);
        //System.out.println(scale);
        //System.out.println();

        for (int i = 0; i < 4; i++) {
            v[i] *= scale;
            //System.out.println(v[i]);
            v[i] = Range.clip(v[i], -1.0, 1.0);
            motors[i].setPower(v[i]);
        }
    }

    public Pose2d track(){
        Vector2d[] v = new Vector2d[4];

        double heading = gyro.getHeading(); //todo: format heading
        double z = Math.toRadians(inverse(gyro.getIntegratedZValue()));
        double robotAngle = 0.0;
        heading = Math.toRadians(heading); //fix: mount gyro sideways bc I'm lazy LUL

        double x = 0.0, y = 0.0;

        for (int i = 0; i < 4; i++){
            double pos = motors[i].getCurrentPosition() * 0.63782834439 * modifier;
            double distance = (pos - prevpos[i]) / 1120;
            distance *= 3;
            distance *= 4 * Math.PI;
            v[i] = new Vector2d(distance, (i == 1 || i == 2) ? distance * -1 : distance);
            prevpos[i] = motors[i].getCurrentPosition() * 0.63782834439 * modifier;
        }

        Vector2d left = v[0].normalize(v[2]); //normalize by averaging x and y coordinates
        Vector2d right = v[1].normalize(v[3]); //normalize legs first, then normalize two legs together to get overall transposition
        Vector2d center = left.normalize(right); //todo: insert rotational logic
        double distance = odometer.getDistance() - prevpos[4];
        g_distance = distance;
        if (Math.abs(distance) >= 0.5){ //todo: tune this
            prevpos[4] = odometer.getDistance();
            Vector2d v2 = new Vector2d(p.x() - memo.x(), p.y() - memo.y());
            if (v2.y() != 0.0 && v2.norm() >= 0.5){
                avgHeading = avgHeading + (Math.PI / 2);
                avgHeading = avgHeading % (2 * Math.PI);
                sum = 0;
                count = 1;
                double dist = distance * Math.sin(avgHeading);
                double scale = dist / v2.y();
                v2.multiplied(scale);
                p = new Pose2d(memo.x() + v2.x(), memo.y() + v2.y(), 0);
            }
            memo = p;
        }
        else{
            sum += this.gyro.getHeading();
            avgHeading = sum / count;
            count++;
            avgHeading = Math.toRadians(avgHeading);
        }

        robotAngle = (Math.atan2(center.y(), center.x()));// + robotHeading) / 2;
        //robotAngle = robotHeading; //todo: test if this even works

        double startx = center.norm() * Math.cos(heading + robotAngle);
        double starty = center.norm() * Math.sin(heading + robotAngle);

        p = new Pose2d(p.x() + startx, p.y() + starty, heading);
        return p;
    }

    public double getG_distance(){
        return g_distance;
    }

    public Pose2d track(Telemetry telemetry) {
        Vector2d[] v = new Vector2d[4];

        double heading = Math.toRadians(gyro.getHeading()); //todo: format heading
        double z = Math.toRadians(inverse(gyro.getIntegratedZValue()));
        double robotAngle = 0.0;
        //heading = Math.toRadians(heading); //fix: mount gyro sideways bc I'm lazy LUL

        double x = 0.0, y = 0.0;

        for (int i = 0; i < 4; i++) {
            double pos = motors[i].getCurrentPosition() * 0.63782834439;
            double distance = (pos - prevpos[i]) / 1120;
            distance *= 3;
            distance *= 4 * Math.PI;
            v[i] = new Vector2d(distance, (i == 1 || i == 2) ? distance * -1 : distance);
            prevpos[i] = motors[i].getCurrentPosition() * 0.63782834439;
        }

        Vector2d left = v[0].normalize(v[2]); //normalize by averaging x and y coordinates
        Vector2d right = v[1].normalize(v[3]); //normalize legs first, then normalize two legs together to get overall transposition
        Vector2d center = left.normalize(right); //todo: insert rotational logic
        double distance = odometer.getDistance() - prevpos[4];
        g_distance = distance;
        telemetry.addData("Distance: ", distance);
        if (Math.abs(distance) >= 0.10){ //todo: tune this
            prevpos[4] = odometer.getDistance();
            Vector2d v2 = new Vector2d(p.x() - memo.x(), p.y() - memo.y());
            if (v2.y() != 0.0 && v2.norm() >= 0.5){
                avgHeading = avgHeading + (Math.PI / 2);
                avgHeading = avgHeading % (2 * Math.PI);
                sum = 0;
                count = 1;
                double dist = distance * Math.sin(avgHeading);
                double scale = dist / v2.y();
                v2.multiplied(scale);
                p = new Pose2d(memo.x() + v2.x(), memo.y() + v2.y(), 0);
            }
            memo = p;
        }
        else{
            sum += this.gyro.getHeading();
            avgHeading = sum / count;
            count++;
            avgHeading = Math.toRadians(avgHeading);
        }

        robotAngle = Math.atan2(center.y(), center.x());
        //robotAngle = Math.toRadians(robotHeading);
        telemetry.addData("Robot Angle: ", robotAngle);
        telemetry.addData("Robot Heading: ", Math.toDegrees(heading));
        telemetry.addData("Robot True Heading: ", this.gyro.getHeading());
        telemetry.addData("Norm: ", center.norm());
        telemetry.addData("Total: ", heading + robotAngle);
        //robotAngle = robotHeading; //todo: test if this even works

        double startx = center.norm() * Math.cos(heading + robotAngle);
        double starty = center.norm() * Math.sin(heading + robotAngle);
        telemetry.addData("Delta X: ", startx);
        telemetry.addData("Delta Y: ", starty);

        p = new Pose2d(p.x() + starty, p.y() + startx, heading);
        return p;
    }

    private double inverted(double heading){
        double y = Math.sin(heading);
        double x = Math.cos(heading);
        return Math.atan2(x, y);
    }


    public boolean goTo(Pose2d pos, Telemetry t){
        track();
        boolean result = false;

        double robotAngle = Math.atan2(pos.y() - p.x(), pos.x() - p.y()) - Math.toRadians(this.gyro.getHeading());
        double e = pos.heading() - invert(this.gyro.getHeading());
        if (e > 180){
            e -= 360;
        }
        double trans = Range.clip(e, -1, 1);
        t.addData("Trans:", trans);
        t.addData("Angle: ", Math.toDegrees(robotAngle));
        t.addData("X: ", this.p.x());
        t.addData("Y: ", this.p.y());
        double dist = pos.dist(new Pose2d(p.y(), p.x(), 0));
        if (first){
            primary = dist;
            primaryAngle = robotAngle;
            first = false;
        }
        t.addData("Dist: ", dist);
        double power = ((1.013567309815 / 2.0) * (Math.tanh((5 * (dist / primary)) - 2.5))) + 0.5;
        power += 0.3;
        power *= 0.5;
        power = Range.clip(power, 0, 1);
        t.addData("Power: ", power);
        if (dist > (5)) {
            t.addData("Moving: ", p.toString());
            drive(power, robotAngle, 0.0, 0.0);
        }
        else{
            result = true;
            first = true;
            t.addData("Moving on: ", p.toString());
            drive(0.0, 0, 0.0, 0.0);
            motor_reset();
        }
        return result;
    }
    public boolean goTo(Pose2d pos, Telemetry t, double r){
        track();
        boolean result = false;

        double robotAngle = Math.atan2(pos.y() - p.x(), pos.x() - p.y()) - Math.toRadians(this.gyro.getHeading());
        double e = pos.heading() - invert(this.gyro.getHeading());
        if (e > 180){
            e -= 360;
        }
        double trans = Range.clip(e, -1, 1);
        t.addData("Trans:", trans);
        t.addData("Angle: ", Math.toDegrees(robotAngle));
        t.addData("X: ", this.p.x());
        t.addData("Y: ", this.p.y());
        double dist = pos.dist(new Pose2d(p.y(), p.x(), 0));
        if (first){
            primary = dist;
            primaryAngle = robotAngle;
            first = false;
        }
        t.addData("Dist: ", dist);
        double power = ((1.013567309815 / 2.0) * (Math.tanh((5 * (dist / primary)) - 2.5))) + 0.5;
        power += 0.3;
        power *= 0.5;
        power = Range.clip(power, 0, 1);
        t.addData("Power: ", power);
        if (dist > r) {
            t.addData("Moving: ", p.toString());
            drive(power, robotAngle, 0.0, 0.0);
        }
        else{
            result = true;
            first = true;
            t.addData("Moving on: ", p.toString());
            drive(0.0, 0, 0.0, 0.0);
            motor_reset();
        }
        return result;
    }
    public boolean goTo(Pose2d pos, Telemetry t, double spower, double r){
        track();
        boolean result = false;

        double robotAngle = Math.atan2(pos.y() - p.x(), pos.x() - p.y()) - Math.toRadians(this.gyro.getHeading());
        double e = pos.heading() - invert(this.gyro.getHeading());
        if (e > 180){
            e -= 360;
        }
        double trans = Range.clip(e, -1, 1);
        t.addData("Trans:", trans);
        t.addData("Angle: ", Math.toDegrees(robotAngle));
        t.addData("X: ", this.p.x());
        t.addData("Y: ", this.p.y());
        double dist = pos.dist(new Pose2d(p.y(), p.x(), 0));
        if (first){
            primary = dist;
            primaryAngle = robotAngle;
            first = false;
        }
        t.addData("Dist: ", dist);
        double power = ((1.013567309815 / 2.0) * (Math.tanh((5 * (dist / primary)) - 2.5))) + 0.5;
        power += 0.3;
        power *= spower;
        power = Range.clip(power, 0, 1);
        t.addData("Power: ", power);
        if (dist > r) {
            t.addData("Moving: ", p.toString());
            drive(power, robotAngle, 0.0, 0.0);
        }
        else{
            result = true;
            first = true;
            t.addData("Moving on: ", p.toString());
            drive(0.0, 0, 0.0, 0.0);
            motor_reset();
        }
        return result;
    }
    public boolean goTo(Pose2d pos, Telemetry t, double spower, double r, int rand){
        track(t);
        boolean result = false;

        double robotAngle = Math.atan2(pos.y() - p.x(), pos.x() - p.y()) - Math.toRadians(this.gyro.getHeading());
        double e = pos.heading() - invert(this.gyro.getHeading());
        if (e > 180){
            e -= 360;
        }
        double trans = Range.clip(e, -1, 1);
        t.addData("Trans:", trans);
        t.addData("Angle: ", Math.toDegrees(robotAngle));
        t.addData("X: ", this.p.x());
        t.addData("Y: ", this.p.y());
        double dist = pos.dist(new Pose2d(p.y(), p.x(), 0));
        if (first){
            primary = dist;
            primaryAngle = robotAngle;
            first = false;
        }
        t.addData("Dist: ", dist);
        double power = ((1.013567309815 / 2.0) * (Math.tanh((5 * (dist / primary)) - 2.5))) + 0.5;
        power += 0.3;
        power *= spower;
        power = Range.clip(power, 0, 1);
        t.addData("Power: ", power);
        if (dist > r) {
            t.addData("Moving: ", p.toString());
            drive(power, robotAngle, 0.0, 0.0);
        }
        else{
            result = true;
            first = true;
            t.addData("Moving on: ", p.toString());
            drive(0.0, 0, 0.0, 0.0);
            motor_reset();
        }
        return result;
    }
    private double invert(double heading){
        return ((360 - heading) % 360);
    }
    private double flip(double heading){
        return 360 - heading;
    }
    private double inverse(double heading){
        return heading * -1;
    }

    public double getRobotHeading(){
        return this.robotHeading;
    }

    public double getMinimumDistance(){
        double min = 9999;
        for (int i = 0; i < 4; i++){
            double distance = motors[i].getCurrentPosition() - prevpos[i];
            if (Math.abs(distance) < min){
                min = distance;
            }
        }
        return min;
    }

    public double getOdoDistance(){
        return odometer.getDistance();
    }

    public void enable(){
        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void motor_reset(){
        odometer.reset();
        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            prevpos[i] = 0.0;
        }
    }
    public void engage(){
        odometer.engage();
        odometer.reset();
    }
    public void disengage(){
        odometer.disengage();
        odometer.reset();
    }

    public void odoReset(){
        odometer.reset();
    }
}
