package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by arnav on 2/3/2018.
 */
@Autonomous(name = "sin_test", group = "Gyro:")
public class sin_test extends OpMode{
    private State mRobotState;
    private boolean mResetEncoder = false;
    private ElapsedTime mStateTime = new ElapsedTime();
    private ElapsedTime mTurnTime = new ElapsedTime();

    DcMotor up_left;
    DcMotor up_right;
    DcMotor back_left;
    DcMotor back_right;

    double velocity;
    double power_threshold = 0.0;
    double avgpower = 0.0;
    double previouspower = 1.0;
    double decrementor = 0.0;
    double voltage = 0.0;
    double angular = 0.0;
    double deltav = 0.0;
    double benchmark = 0.0;
    double benchv = 0.0;
    double loss = 0.0;
    double interval = 0.0625;
    double avgint = 0.0;
    double prediction = 0.0;

    long sum = 0;
    long avgsum = 0;

    int counter = 0;
    int evalcounter = 0;
    int oldpos = 0;
    int newpos = 0;
    int delta = 0;
    int cone = 0;

    boolean init = false;
    boolean first = false;
    boolean flip = false;
    boolean reverse = false;
    boolean calibrate = false;
    boolean inverted = false;
    boolean reset = false;
    boolean eval = false;

    int passes = 0;
    int sign = 0;
    int slows = 0;
    int targetAngle = 0;

    ModernRoboticsI2cGyro gyro;

    VoltageSensor voltagemain;

    final String QTAG= "QL8719";

    private enum State{
        STATE_1,
        STATE_TURN,
        STATE_TURN2,
        STATE_TURN3,
        STATE_RETURN,
        STATE_OUT
    }
    public void init(){
        up_left = hardwareMap.dcMotor.get("up_left");
        up_right = hardwareMap.dcMotor.get("up_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");

        voltagemain = hardwareMap.voltageSensor.get("Motor Controller 1");

        back_right.setDirection(DcMotor.Direction.REVERSE);
        up_right.setDirection(DcMotor.Direction.REVERSE);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        newState(State.STATE_1);
    }
    @Override
    public void start(){
        voltage = voltagemain.getVoltage();
        decrementor = ((3 * voltage) / 5) - 5.98;
    }
    public void loop(){
        switch (mRobotState){
            case STATE_1:
                logMessage("Process Calibrating:", Double.toString(mStateTime.time()));
                if (!calibrate){
                    gyro.calibrate();
                    calibrate = true;
                }
                if (!gyro.isCalibrating()){
                    logMessage("Done Now:", Integer.toString(gyro.getIntegratedZValue()));
                    newState(State.STATE_TURN);
                }
                break;
            case STATE_TURN:
                logMessage("Starting Turn:", Integer.toString(gyro.getIntegratedZValue()));
                if (sinTurn(0.8,90,5, decrementor)){
                    logMessage("Finished Turn", Integer.toString(gyro.getIntegratedZValue()));
                    slows = 0;
                    flip = false;
                    newState(State.STATE_TURN2);
                }
                avgpower = Math.abs(up_left.getPower()) + Math.abs(up_right.getPower()) + Math.abs(back_left.getPower()) + Math.abs(back_right.getPower());
                avgpower /= 4;
                if (avgpower > previouspower){
                    slows++;
                }
                previouspower = avgpower;
                if ((avgpower <= 0.1 && avgpower >= 0.0) && slows > 2){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    flip = false;
                    slows = 0;
                    logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                    logMessage("Slows:", Integer.toString(slows));
                    targetAngle = gyro.getIntegratedZValue();
                    newState(State.STATE_TURN2);
                }
                break;
            case STATE_TURN2:
                if (mStateTime.time() >= 0.25) {
                    logMessage("Starting Turn:", Integer.toString(gyro.getIntegratedZValue()));
                    if (sinTurn(0.8, 270,5, 1.7)) {
                        logMessage("Finished Turn", Integer.toString(gyro.getIntegratedZValue()));
                        slows = 0;
                        flip = false;
                        newState(State.STATE_OUT);
                    }
                    avgpower = Math.abs(up_left.getPower()) + Math.abs(up_right.getPower()) + Math.abs(back_left.getPower()) + Math.abs(back_right.getPower());
                    avgpower /= 4;
                    if (avgpower > previouspower){
                        slows++;
                    }
                    previouspower = avgpower;
                    logMessage("Slows:", Integer.toString(slows));
                    if ((avgpower <= 0.1 && avgpower >= 0.0) && slows > 2){
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        flip = false;
                        slows = 0;
                        logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                        logMessage("Slows:", Integer.toString(slows));
                        targetAngle = gyro.getIntegratedZValue();
                        newState(State.STATE_OUT);
                    }
                }
                break;
            case STATE_TURN3:
                if (mStateTime.time() >= 0.25) {
                    logMessage("Starting Turn:", Integer.toString(gyro.getIntegratedZValue()));
                    if (sinTurn(0.8, 90,5, decrementor)) {
                        logMessage("Finished Turn", Integer.toString(gyro.getIntegratedZValue()));
                        slows = 0;
                        flip = false;
                        newState(State.STATE_OUT);
                    }
                    avgpower = Math.abs(up_left.getPower()) + Math.abs(up_right.getPower()) + Math.abs(back_left.getPower()) + Math.abs(back_right.getPower());
                    avgpower /= 4;
                    if (avgpower > previouspower){
                        slows++;
                    }
                    previouspower = avgpower;
                    if ((avgpower <= 0.1 && avgpower >= 0.0) && slows > 2){
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        flip = false;
                        slows = 0;
                        logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                        logMessage("Slows:", Integer.toString(slows));
                        targetAngle = gyro.getIntegratedZValue();
                        newState(State.STATE_OUT);
                    }
                }
                break;
            case STATE_OUT:
                //logMessage("...And he STICKS THE LANDING", Integer.toString(gyro.getIntegratedZValue()));
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                break;
        }
    }
    @Override
    public void stop(){
        System.gc();
    }
    private void logMessage( String sMsgHeader, String sMsg) {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii(QTAG, getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }
    private void newState (sin_test.State newState){
        // Reset State time and change to next state
        mStateTime.reset();
        mRobotState = newState;
        previouspower = 1.0;
        avgpower = 0.0;
        slows = 0;
        passes = 0;
        evalcounter = 0;
        prediction = 0;
        sum = 0;
        avgint = 0;
        avgpower = 0;
        avgsum = 0;
        counter = 0;
        benchmark = 0;
        init = false;
        eval = false;
        oldpos = 0;
        newpos = 0;
        delta = 0;
        interval = 0;
        angular = 0;
        loss = 0;
        flip = false;
        mResetEncoder = false;
    }
    boolean sinTurn (double speed, int angle, int acceptance, double decrement) {
        // keep looping while we are still active, and not on heading.
        boolean reached = false;
        int initial = 0;
        int cursign = 0;
        int deviation = angle - Math.abs(gyro.getIntegratedZValue());
        double theta = Math.abs(angle - gyro.getIntegratedZValue());
        double scale_factor = 1.0;

        if (!init){
            initial = angle - Math.abs(gyro.getIntegratedZValue());
            mTurnTime.reset();
            sign = Range.clip(initial, -1, 1);
            init = true;
        }
        //theta -= 30;
        if (Math.abs(initial) > 90) {
            if (mTurnTime.time() % interval == 0) {
                newpos = Math.abs(gyro.getIntegratedZValue());
                counter++;
                delta = Math.abs(newpos - oldpos); //input
                sum += delta;
                angular = sum / counter; //relu op
                avgsum += interval;
                avgint = avgsum / counter; //adaptive learning rate
                deltav = angular / avgint; //fully connected
                oldpos = Math.abs(gyro.getIntegratedZValue());
            }
            if (Math.abs(deviation) <= 90 && evalcounter == 0) {
                benchmark = mTurnTime.time();
                benchv = 90 / benchmark;
                loss = benchv / deltav;
                evalcounter++;
                if (Math.abs(loss - 1) >= 0.25) {
                    interval /= 2;
                    eval = true;
                }
            }
            if (eval) {
                if (Math.abs(deviation) <= 67 && evalcounter == 1) {
                    benchmark = mTurnTime.time();
                    benchv = 90 / benchmark;
                    loss = benchv / deltav;
                    evalcounter++;
                    if (Math.abs(loss - 1) >= 0.25) {
                        interval /= 2;
                    }
                }
            }
            if (Math.abs(deviation) <= 45 && evalcounter == 2) {
                prediction = deltav / 0.75;
                evalcounter++;
            }
        }
        if (inverted){
            theta = 5 - theta;
        }
        theta = (theta * Math.PI) / 180;

        if (Math.abs(deviation) <= 90){
            scale_factor = Math.sin(theta);
            speed *= scale_factor;
            if (flip){
                speed /= decrement;
            }
            /*else if (Math.abs(deviation) <= 30 && !flip && passes == 0 && (Math.abs(initial) > 90)){
                speed /= decrement;
            }*/
        }
        if (deviation > 0) {
            /*if (flip){
                if (mTurnTime.time() >= 0.25){
                    if (passes == 0 || (passes >= 1 && flip)) {
                        up_left.setPower(speed);
                        back_left.setPower(speed);
                        up_right.setPower(-1 * speed);
                        back_right.setPower(-1 * speed);
                        inverted = false;
                    }
                    if (passes >= 1 && !flip){
                        up_left.setPower(-1 * speed);
                        back_left.setPower(-1 * speed);
                        up_right.setPower(speed);
                        back_right.setPower(speed);
                        inverted = true;
                    }
                }
            }
            else {*/
            if (passes == 0 || (passes >= 1 && flip)) {
                up_left.setPower(speed);
                back_left.setPower(speed);
                up_right.setPower(-1 * speed);
                back_right.setPower(-1 * speed);
                inverted = false;
            }
            if (passes >= 1 && !flip) {
                up_left.setPower(-1 * speed);
                back_left.setPower(-1 * speed);
                up_right.setPower(speed);
                back_right.setPower(speed);
                inverted = true;
            }
            //}
            /*if (passes >= 1 && flip){
                up_left.setPower(speed);
                back_left.setPower(speed);
                up_right.setPower(-1 * speed);
                back_right.setPower(-1 * speed);
            }*/
            cursign = deviation;
            cursign = Range.clip(cursign, -1, 1);
            if (cursign == (sign * -1) && !reverse) {
                passes++;
                reverse = true;
                logMessage("Flips:", Integer.toString(passes));
            }

            logMessage("Pos", Integer.toString(gyro.getIntegratedZValue()));
            logMessage("Target", Integer.toString(angle));
            logMessage("Speed:", Double.toString(speed));
            logMessage("Passes:", Integer.toString(passes));
            logMessage("Flip:", Boolean.toString(flip));
            if (Math.abs(initial) > 90){
                if (Math.abs(deviation) <= prediction && !flip) {
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    logMessage("Coasting:", Double.toString(prediction));
                    logMessage("Loss:", Double.toString(loss));
                    logMessage("Rate:", Double.toString(deltav));
                }
            }
            if (Math.abs(deviation) <= acceptance) {
                if (passes >= 1 && flip) {
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    slows = 0;
                    passes = 0;
                    flip = false;
                    reached = true;
                }
                else if (passes == 0){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                }
            }
            else{
                if (passes >= 1 && !flip){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    flip = true;
                }
            }
        }
        if (deviation < 0){
            /*if (flip){
                if (mTurnTime.time() >= 0.25){
                    if (passes == 0 || (passes >= 1 && flip)) {
                        up_left.setPower(-1 * speed);
                        back_left.setPower(-1 * speed);
                        up_right.setPower(speed);
                        back_right.setPower(speed);
                        inverted = false;
                    }
                    if (passes >= 1 && !flip){
                        up_left.setPower(speed);
                        back_left.setPower(speed);
                        up_right.setPower(-1 * speed);
                        back_right.setPower(-1 * speed);
                        inverted = true;
                    }
                }
            }
            else {*/
            if (passes == 0 || (passes >= 1 && flip)) {
                up_left.setPower(-1 * speed);
                back_left.setPower(-1 * speed);
                up_right.setPower(speed);
                back_right.setPower(speed);
                inverted = false;
            }
            if (passes >= 1 && !flip) {
                up_left.setPower(speed);
                back_left.setPower(speed);
                up_right.setPower(-1 * speed);
                back_right.setPower(-1 * speed);
                inverted = true;
            }
            //}
            /*if (passes >= 1 && flip){
                up_left.setPower(-1 * speed);
                back_left.setPower(-1 * speed);
                up_right.setPower(speed);
                back_right.setPower(speed);
            }*/
            cursign = deviation;
            cursign = Range.clip(cursign, -1, 1);
            if ((cursign == (sign * -1)) && !reverse){
                passes++;
                reverse = true;
                logMessage("Flips:", Integer.toString(passes));
            }

            logMessage("Pos",Integer.toString(gyro.getIntegratedZValue()));
            logMessage("Target",Integer.toString(angle));
            logMessage("Speed:", Double.toString(speed));
            logMessage("Passes:", Integer.toString(passes));
            logMessage("Flip:", Boolean.toString(flip));
            if (Math.abs(initial) > 90) {
                if (Math.abs(deviation) <= prediction && !flip) {
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    logMessage("Coasting:", Double.toString(prediction));
                    logMessage("Loss:", Double.toString(loss));
                    logMessage("Rate:", Double.toString(deltav));
                }
            }
            if (Math.abs(deviation) <= acceptance) {
                if (passes >= 1 && flip) {
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    slows = 0;
                    passes = 0;
                    flip = false;
                    reached = true;
                }
                else if (passes == 0){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                }
            }
            else{
                if (passes >= 1 && !flip){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    flip = true;
                }
            }
        }
        /*if (Math.abs(deviation) == 0){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            reached = true;
        }*/
        return reached;
    }
}
