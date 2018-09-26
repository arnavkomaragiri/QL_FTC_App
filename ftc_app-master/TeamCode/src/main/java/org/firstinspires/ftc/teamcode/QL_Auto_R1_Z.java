package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by Arnav on 10/8/2017.
 */
@Autonomous(name = "State: QL_Auto_R1_Z",group = "State:")
public class QL_Auto_R1_Z extends OpMode{
    private State mRobotState;
    private boolean mResetEncoder = false;
    private ElapsedTime mStateTime = new ElapsedTime();
    private ElapsedTime mTurnTime = new ElapsedTime();

    boolean reset = false;

    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;
    double redval[];
    double blueval[];
    double offset;
    //final int THRESHOLD = 1;

    final String QTAG= "QL8719";

    //boolean bPrevState = false;
    //boolean bCurrState = false;
    boolean bLedOn = false;

    boolean left = false;
    boolean center = false;
    boolean right = false;
    boolean mReset = false;
    boolean mVueMarkFound = false;
    boolean stop = false;

    double left_grab_release = 0.1;
    double right_grab_release = 0.9;
    double velocity = 0.4;
    double distance = 5;
    double kickout = 1.0;
    double power_threshold = 0.2;
    double voltage;
    double avgpower = 0.0;
    double previouspower = 1.0;

    int targetAngle;
    int turn_counts;
    int i;
    int slidecounts;
    final int THRESHOLD = 1;

    boolean init = false;
    boolean first = false;
    boolean flip = false;
    boolean reverse = false;
    boolean calibrate = false;
    boolean inverted = false;

    int passes = 0;
    int sign = 0;
    int slows = 0;

    double decrementor = 2.0;
    double speed = 0.8;

    DcMotor up_right;
    DcMotor up_left;
    DcMotor back_right;
    DcMotor back_left;
    DcMotor slide;
    DcMotor slide2;

    Servo grab_right;
    Servo grab_left;
    Servo grab_top;
    Servo stick_x;
    Servo stick_y;

    ColorSensor colorsensor;
    ModernRoboticsI2cGyro gyro;
    VoltageSensor voltage_left;
    VoltageSensor voltage_right;
    OpticalDistanceSensor odr;

    static final double     HEADING_THRESHOLD       = 15;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;

    FTCUtils myUtils = new FTCUtils();

    public static final String TAG = "Vuforia VuMark Sample";
    VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables ;
    VuforiaTrackable relicTemplate ;
    RelicRecoveryVuMark vuMark ;
    VuforiaLocalizer.Parameters parameters;
    boolean bFoundColor = false;
    final double QUARTER_TURN = ((3 * 3) * 3.14);
    private String destination_Turn = "";

    boolean z = false;

    private enum TRAVEL_DIRECTION {
        DRIVE_RIGHT,
        DRIVE_LEFT,
        DRIVE_CENTER
    }
    private enum State{
        STATE_INITIAL,
        STATE_DROPSTICK,
        STATE_LIFTBLOCK,
        STATE_COLORSCAN,
        STATE_LEAVE_PLATFORM,
        STATE_DRIVE_TO_BOX,
        STATE_TURN,
        STATE_LOWER_GLYPH,
        STATE_TURN2,
        STATE_REACH_BOX,
        STATE_PLACE,
        STATE_DROP,
        STATE_SWIVEL,
        STATE_TRAVEL,
        STATE_GRAB,
        STATE_LIFT,
        STATE_BACK,
        STATE_LIFT2,
        STATE_SWIVEL2,
        STATE_RETURN,
        STATE_AIM,
        STATE_SCORE,
        STATE_PARK,
        STATE_REINFORCE,
        STATE_END,
        STATE_PREPARE,
        STATE_STOP
    }
    public void init(){
        up_right = hardwareMap.dcMotor.get("up_right");
        up_left = hardwareMap.dcMotor.get("up_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        slide = hardwareMap.dcMotor.get("slide1");
        slide2 = hardwareMap.dcMotor.get("slide2");
        grab_left = hardwareMap.servo.get("grab_left");
        grab_right = hardwareMap.servo.get("grab_right");

        stick_x = hardwareMap.servo.get("stick_y");
        stick_y = hardwareMap.servo.get("stick_x");
        grab_top = hardwareMap.servo.get("grab_top");


        colorsensor = hardwareMap.colorSensor.get("colorsensor");
        odr = hardwareMap.opticalDistanceSensor.get("odr");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        voltage_left = hardwareMap.voltageSensor.get("Motor Controller 2");
        voltage_right = hardwareMap.voltageSensor.get("Motor Controller 1");

        //gyro.calibrate();

        back_right.setDirection(DcMotor.Direction.REVERSE);
        up_right.setDirection(DcMotor.Direction.REVERSE);

        up_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        up_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // stick_x.setPosition(0.4);
        stick_y.setPosition(1.0);
        //stick_x.setPosition(0.9);
        stick_x.setPosition(1.0);

        grab_left.setPosition(0.1);
        grab_right.setPosition(0.9);
        grab_top.setPosition(0.35);

        logMessage("init :", " initializing Vuforia");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdxZ3kj/////AAAAGVcZGhm+xkRnm25bq3Zjd2RSQ2pfdOy7+EfbjO8XJD7NFRkWQ0Xx0uzVcMSKKXYgimvMnjKHFiFnaRiubni6joz57M2ei/tPxb54q7cGy6O/Yw9C9EE5OGGRaVppUDSG9hV7iWCpCb7PS3OOj9ST3grD+GfojQZOxaugVLxwqlNXF7KRODkbvBXpQ5bUGiRlL0k3AlhgUbHnlPes0hMQt/uZ+Dzg56ixG6W1SAbZpK6jubVTJZ9uOQ25hevaP5QF1nrpLQEWvqusNwc+W/BNL7//+hePPnRYZU4c9i0crjyG7R0f7UgQ/vLWppNTYnu6H8v/rY34rIIGyyJ8iXSoHSCcj0T9WObHDB3DHQeYQI+G";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    }
    @Override
    public void start(){
        telemetry.addData("Entering","start");
        System.gc();
        up_left.setPower(0.0);
        up_right.setPower(0.0);
        back_left.setPower(0.0);
        back_right.setPower(0.0);

        stick_x.setPosition(0.45);
        stick_y.setPosition(1.0);
        grab_left.setPosition(0.75);
        grab_right.setPosition(0.25);
        voltage = voltage_left.getVoltage();
        newState(State.STATE_INITIAL);

        logMessage("start :", " loading  Vuforia trackables");
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTemplate.setName("relicVuMarkTemplate");
        logMessage("Leaving", " Start..");
        if (gyro.isCalibrating()){
            logMessage("Gyro","Calibrating");
        }
        gyro.resetZAxisIntegrator();

        gyro.calibrate();
        calibrate = true;
    }


    public void loop(){

        switch (mRobotState){
            case STATE_INITIAL:
                telemetry.addData("State:","initial");
                if (!mVueMarkFound)
                    vfa_lookup ();
                if (!gyro.isCalibrating()) {
                    newState(State.STATE_DROPSTICK);
                }
                break;
            case STATE_DROPSTICK:
                stick_y.setPosition(0.05);
                if (!mVueMarkFound)
                    vfa_lookup ();
                if (mStateTime.time() >= 0.5) {
                    newState(State.STATE_COLORSCAN);
                }
                break;

            case STATE_COLORSCAN:
                // telemetry.addData("State:","color_scan");
                logMessage("State:","color_scan");
                logMessage ("Runtime is " , Double.toString(getRuntime() ) );

                logMessage("Color Sensor Red:", Integer.toString(colorsensor.red()));
                logMessage("Color Sensor Green:",Integer.toString(colorsensor.green()));
                logMessage("Color Sensor Blue:",Integer.toString(colorsensor.blue()));
                logMessage("Color Sensor White:",Integer.toString(colorsensor.alpha()));
                if (!mVueMarkFound)
                    vfa_lookup ();
                if (mStateTime.time() >= 0.25){
                    if (colorsensor.red() > colorsensor.blue()){
                        stick_x.setPosition(0.2);
                        // stick_y.setPosition(1.0);
                        logMessage("Red Selected",Integer.toString(colorsensor.red()));
                        bFoundColor = true;
                    }
                    if (colorsensor.blue() > colorsensor.red()){
                        stick_x.setPosition(0.8);
                        logMessage("Blue Selected",Integer.toString(colorsensor.blue()));
                        bFoundColor = true;
                    }
                    if (mStateTime.time() >= 0.375 && !bFoundColor){
                        stick_x.setPosition(0.5);
                    }
                    if ( bFoundColor || mStateTime.time() >= 0.5) {
                        //stick_x.setPosition(0.10);
                        logMessage("Stick","Times up, Lift Stick Y");
                        logMessage("Y Val:", Double.toString(stick_y.getPosition()));
                        stick_y.setPosition(0.9);
                        newState(State.STATE_LIFTBLOCK);
                    }
                }

                break;

            case STATE_LIFTBLOCK:
                telemetry.addData("State:","Lift BLock");
                slide.setPower(0.3);
                stick_y.setPosition(1.0);
                colorsensor.enableLed(false);
                if (!mVueMarkFound)
                    vfa_lookup ();
                if (mStateTime.time() >= 0.5) {
                    slide.setPower(0.0);
                    newState(State.STATE_LEAVE_PLATFORM);
                }
                break;

            case STATE_LEAVE_PLATFORM:
                //stop = true;
                if (leave_platform()){//gyroDrive(0.3,distance,0)) {
                    reset_encoders();
                    run_without_Encoders();
                    if (voltage > 13.5 || voltage > 13.5){
                        velocity = 0.25;
                        logMessage("Voltage:","Exceeding Threshold" + Double.toString(voltage_left.getVoltage()));
                    }
                    else{
                        velocity = 0.3;
                        logMessage("Voltage:","In Optimal Zone" + Double.toString(voltage_left.getVoltage()));
                    }
                    newState (State.STATE_TURN);
                }
                break;
            case STATE_TURN:
                if (mStateTime.time() >= 0.25) {
                    if (sinTurn(speed, 90, 1, 2.0)) {
                        newState(State.STATE_LOWER_GLYPH);
                    }
                    if (mStateTime.time() >= 2.0) {
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        logMessage("Kickout", Integer.toString(gyro.getIntegratedZValue()));
                        newState(State.STATE_LOWER_GLYPH);
                    }
                }
                break;
            case STATE_LOWER_GLYPH:
                slide.setPower(-0.3);
                if (mStateTime.time() > 0.3) {
                    slide.setPower(0.0);
                    grab_left.setPosition(left_grab_release); // test
                    grab_right.setPosition(right_grab_release); // test
                    newState(State.STATE_DRIVE_TO_BOX);
                }
                break;
            case STATE_DRIVE_TO_BOX:
                run_without_Encoders();
                powerdrive(-1);
                grab_left.setPosition(0.4);
                grab_right.setPosition(0.6);
                if (mStateTime.time() >= 0.75){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    //grab_left.setPosition(0.1);
                    //grab_right.setPosition(0.9);
                    grab_left.setPosition(0.1);
                    grab_right.setPosition(0.9);
                    run_without_Encoders();
                    newState(State.STATE_PLACE);
                }
                break;
            case STATE_PLACE:
                distance = 8;
                logMessage("State","Place");
                powerdrive(1);
                if (mStateTime.time() >= 0.4){
                    if (voltage > 12.8){
                        velocity = 0.25;
                        logMessage("Voltage:","Exceeding Threshold" + Double.toString(voltage_left.getVoltage()));
                    }
                    else{
                        velocity = 0.4;
                        logMessage("Voltage:","In Optimal Zone" + Double.toString(voltage_left.getVoltage()));
                    }
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    /*if (voltage >= 13.3){
                        decrementor = 2.0;
                    }
                    else{
                        decrementor = 1.7;
                    }*/
                    decrementor = ((3 * voltage) / 5) - 5.98;
                    run_without_Encoders();
                    newState(State.STATE_SWIVEL);
                }
                break;

            case STATE_SWIVEL:
                logMessage("State:","Swivel");
                if (mStateTime.time() >= 0.125) {
                    if (sinTurn(speed, 270, 1, decrementor)) { //1.7
                        //reset_encoders();
                        //run_to_position_Encoders();
                        grab_left.setPosition(0.3);
                        grab_right.setPosition(0.7);
                        logMessage("Done:", Double.toString(mStateTime.time()));
                        newState(State.STATE_TRAVEL);
                    }
                    if (z){
                        slide.setPower(-0.6);
                        if (mStateTime.time() >= 0.6){
                            slide.setPower(0.0);
                            grab_left.setPosition(0.4);
                            grab_right.setPosition(0.6);
                        }
                    }
                    avgpower = Math.abs(up_left.getPower()) + Math.abs(up_right.getPower()) + Math.abs(back_left.getPower()) + Math.abs(back_right.getPower());
                    avgpower /= 4;
                    if (avgpower > previouspower){
                        slows++;
                    }
                    previouspower = avgpower;
                    if ((avgpower <= 0.1 && avgpower >= 0.0) && slows >= 2){
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                        targetAngle = gyro.getIntegratedZValue();
                        if (targetAngle >= 256 && targetAngle <= 284) {
                            newState(State.STATE_TRAVEL);
                        }
                        else{
                            newState(State.STATE_STOP);
                        }
                    }
                    /*if (mStateTime.time() >= 2.5) {
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                        newState(State.STATE_TRAVEL);
                    }*/
                }
                break;
            case STATE_TRAVEL:
                logMessage("Powerdrive:",Double.toString(mStateTime.time()));
                int angle;
                int target = 260;
                up_left.setPower(-0.85);
                up_right.setPower(-0.85);
                back_left.setPower(-0.85);
                back_right.setPower(-0.85);
                grab_left.setPosition(0.4);
                grab_right.setPosition(0.6);
                /*if (odr.getLightDetected() <= 1){
                    logMessage("Stopping:",Double.toString(mStateTime.time()));
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    newState(State.STATE_GRAB);
                }*/
                if (mStateTime.time() >= 1.0){
                    logMessage("Stopping:",Double.toString(mStateTime.time()));
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    newState(State.STATE_GRAB);
                }
                break;
            case STATE_GRAB:
                logMessage("State:","Grab");
                grab_left.setPosition(0.75);
                grab_right.setPosition(0.25);
                grab_top.setPosition(1.0);
                if (mStateTime.time() >= 0.25){
                    newState(State.STATE_LIFT);
                }
                break;
            case STATE_LIFT:
                telemetry.addData("State:","Lift");
                slide.setPower(0.6);
                if (voltage_left.getVoltage() >= 13.5){
                    kickout = 0.6;
                }
                if (mStateTime.time() >= 0.5) {
                    slide.setPower(0.0);
                    velocity = 0.3;
                    newState(State.STATE_LIFT2);
                }
                break;
            case STATE_BACK:
                up_left.setPower(0.6);
                up_right.setPower(0.6);
                back_left.setPower(0.6);
                back_right.setPower(0.6);
                if (mStateTime.time() >= 0.5){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    if (voltage > 12.8){
                        velocity = 0.25;
                        logMessage("Voltage:","Exceeding Threshold" + Double.toString(voltage_left.getVoltage()));
                    }
                    else{
                        velocity = 0.4;
                        logMessage("Voltage:","In Optimal Zone" + Double.toString(voltage_left.getVoltage()));
                    }
                    newState(State.STATE_SWIVEL2);
                }
                break;
            case STATE_LIFT2:
                telemetry.addData("State:","Lift");
                slide.setPower(0.6);
                if (mStateTime.time() >= 0.5) {
                    slide.setPower(0.0);
                    velocity = 0.3;
                    decrementor = ((3 * voltage) / 5) - 5.98;
                    slows = 0;
                    newState(State.STATE_BACK);
                }
                break;
            case STATE_SWIVEL2:
                logMessage("State:","Swivel2");
                if (mStateTime.time() >= 0.25) {
                    if (sinTurn(speed, 90,1,decrementor)) {
                        //reset_encoders();
                        //run_to_position_Encoders();
                        logMessage("Returning:", Integer.toString(gyro.getIntegratedZValue()));
                        newState(State.STATE_RETURN);
                    }
                    /*if (mStateTime.time() >= 2.5) {
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));

                        newState(State.STATE_RETURN);
                    }*/
                    avgpower = Math.abs(up_left.getPower()) + Math.abs(up_right.getPower()) + Math.abs(back_left.getPower()) + Math.abs(back_right.getPower());
                    avgpower /= 4;
                    if (avgpower > previouspower){
                        slows++;
                    }
                    previouspower = avgpower;
                    if ((avgpower <= 0.1 && avgpower >= 0.0) && slows >= 2){
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                        targetAngle = gyro.getIntegratedZValue();
                        if (targetAngle >= 76 && targetAngle <= 104) {
                            newState(State.STATE_RETURN);
                        }
                        else{
                            newState(State.STATE_STOP);
                        }
                    }
                }
                break;
            case STATE_RETURN:
                /*distance = 24;
                logMessage("State:","Return");
                if (gyroDrive(0.5,distance,90)){
                    reset_encoders();
                    run_to_position_Encoders();
                    newState(State.STATE_SCORE);
                }*/
                if (mStateTime.time() >= 0.25) {
                    up_left.setPower(-0.85);
                    up_right.setPower(-0.85);
                    back_left.setPower(-0.85);
                    back_right.setPower(-0.85);
                    if (mStateTime.time() >= 1.0) {
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        if (voltage > 13.5 || voltage > 13.5) {
                            velocity = 0.25;
                            logMessage("Voltage:", "Exceeding Threshold" + Double.toString(voltage_left.getVoltage()));
                        } else {
                            velocity = 0.4;
                            logMessage("Voltage:", "In Optimal Zone" + Double.toString(voltage_left.getVoltage()));
                        }
                        left = true;
                        newState(State.STATE_SCORE);
                    }
                }
                break;
            case STATE_SCORE:
                run_without_Encoders();
                powerdrive(-1);
                if (mStateTime.time() >= 0.5){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    grab_left.setPosition(0.1);
                    grab_right.setPosition(0.9);
                    grab_top.setPosition(0.35);
                    run_to_position_Encoders();
                    reset_encoders();
                    newState(State.STATE_PARK);
                }
                break;
            case STATE_PARK:
                distance = 6;
                if (driveToBox2()){
                    grab_left.setPosition(0.1);
                    grab_right.setPosition(0.9);
                    if (z) {
                        newState(State.STATE_REINFORCE);
                    }
                    else{
                        newState(State.STATE_END);
                    }
                }
                break;
            case STATE_REINFORCE:
                run_without_Encoders();
                powerdrive(-1);
                if (mStateTime.time() >= 0.5){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    /*grab_left.setPosition(0.4);
                    grab_right.setPosition(0.45);*/
                    grab_top.setPosition(0.35);
                    run_to_position_Encoders();
                    reset_encoders();
                    newState(State.STATE_END);
                }
                break;
            case STATE_END:
                distance = 6;
                if (driveToBox2()){
                    grab_left.setPosition(0.1);
                    grab_right.setPosition(0.9);
                    if (!z){
                        newState(State.STATE_PREPARE);
                        z = true;
                    }
                    else{
                        newState(State.STATE_STOP);
                    }
                }
                break;
            case STATE_PREPARE:
                speed = 0.8;
                decrementor = 3;
                slows = 0;
                newState(State.STATE_SWIVEL);
                break;
            case STATE_STOP:
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                stick_x.setPosition(1.0);
                colorsensor.enableLed(false);
                relicTrackables.deactivate();
                logMessage("Ta","-Da");
                break;
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    private void newState (QL_Auto_R1_Z.State newState){
        // Reset State time and change to next state
        mStateTime.reset();
        mRobotState = newState;
        mResetEncoder = false;
    }
    private void powerdrive(int direction){
        up_right.setPower(0.3 * direction);
        up_left.setPower(0.3 * direction);
        back_left.setPower(0.3 * direction);
        back_right.setPower(0.3 * direction);
    }
    private void resetEncoders (){
        up_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void run_to_position_Encoders (){
        up_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void run_without_Encoders (){
        up_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        up_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void vfa_lookup () {
        //if (!stop) {
        logMessage("Runtime 1 is ", Double.toString(getRuntime()));

        relicTrackables.activate();
        logMessage("Entering STATE_VUMARK ", "VuMark");
        logMessage("Runtime 2 /is ", Double.toString(getRuntime()));

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        distance = 30; // initialize to center
        offset = 2;    // initialize to center
        targetAngle = 90;   // initialize target angle for center
        left_grab_release = 0.4; // initialize for center
        right_grab_release = 0.45; // initialize for center
        telemetry.addData("Entering STATE_VUMARK ", "Got VuMark" + vuMark.toString());

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            logMessage("VuMark", "  visible :: " + vuMark);
            mVueMarkFound = true;
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                logMessage("VuMark", " --> LEFT" + vuMark.toString());
                //destination_Turn = TRAVEL_DIRECTION.DRIVE_LEFT.name();
                distance = 42;//35.5;//35 //40.5 //40
                offset = 3;
                targetAngle = 135;
                left_grab_release = 0.1;
                right_grab_release = 0.9;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                logMessage("VuMark", "--> CENTER " + vuMark.toString());
                //destination_Turn = TRAVEL_DIRECTION.DRIVE_CENTER.name();
                distance = 30;//29 //30 /34 //30
                offset = 2;
                targetAngle = 90;
                left_grab_release = 0.1;
                right_grab_release = 0.9;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                logMessage("VuMark", "--> RIGHT " + vuMark.toString());
                //destination_Turn = TRAVEL_DIRECTION.DRIVE_RIGHT.name();
                distance = 22; //28
                offset = 1;
                targetAngle = 45;
                left_grab_release = 0.1;
                right_grab_release = 0.9;
            }
            // destination_Turn = TRAVEL_DIRECTION.DRIVE_LEFT.name();
            relicTrackables.deactivate();

        }
        //}
    }
    private boolean leave_platform () {
        boolean done = false;
        if (!mResetEncoder) {
            reset_encoders();
            run_to_position_Encoders();
            mResetEncoder = true;
        }
        int forwardcounts = myUtils.getNewCounts((int)distance,myUtils.NEVEREST60,3);
        up_left.setTargetPosition(forwardcounts);
        up_right.setTargetPosition(forwardcounts);
        back_right.setTargetPosition(forwardcounts);
        back_left.setTargetPosition(forwardcounts);
        up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (up_left.getCurrentPosition() <= (2 * (forwardcounts / 3))){
            up_left.setPower(0.8);
            up_right.setPower(0.8);
            back_left.setPower(0.8);
            back_right.setPower(0.8);
        }
        else {
            up_right.setPower(0.3);
            up_left.setPower(0.3);
            back_left.setPower(0.3);
            back_right.setPower(0.3);
        }

        telemetry.addData("Forward",forwardcounts);
        telemetry.addData("C Position",up_left.getCurrentPosition());
        telemetry.addData("T Position",up_left.getTargetPosition());
        if (up_left.getCurrentPosition()>=forwardcounts ||
                up_right.getCurrentPosition() >=forwardcounts ||
                back_left.getCurrentPosition() >= forwardcounts ||
                back_right.getCurrentPosition() >= forwardcounts){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_right.setPower(0.0);
            back_left.setPower(0.0);
            telemetry.addData("Done moving",0);
            done = true;
        }
        return done;
    }
    private boolean lift_block (){
        boolean lifted = false;
        if (mResetEncoder == false) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mResetEncoder = true;
        }
        int slidecounts = myUtils.getNewCounts(1,myUtils.NEVEREST40,1/3);
        telemetry.addData("Slide:",slidecounts);
        slide.setTargetPosition(slidecounts);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        telemetry.addData("Slidecounts",slide.getCurrentPosition());
        if (slide.getCurrentPosition() >= slidecounts) {
            slide.setPower(0.0);
            telemetry.addData("Done", "slides");
            lifted = true;
        }
        return lifted;
    }
    private boolean swivel_turn(boolean sign){
        boolean done = false;
        int turn_distance = ((9 * 9) * 3)/4;
        turn_counts = myUtils.getNewCounts(turn_distance,myUtils.NEVEREST60,3);
        if (mResetEncoder == false) {
            reset_encoders();
            run_to_position_Encoders();
            mResetEncoder = true;
        }
        up_right.setTargetPosition(turn_counts);
        up_left.setTargetPosition(turn_counts);
        back_left.setTargetPosition(turn_counts);
        back_right.setTargetPosition(turn_counts);
        if (sign) {
            up_left.setPower(-0.3);
            up_right.setPower(0.3);
            back_left.setPower(-0.3);
            back_right.setPower(0.3);
        }
        else {
            up_left.setPower(0.3);
            up_right.setPower(-0.3);
            back_left.setPower(0.3);
            back_right.setPower(-0.3);
        }
        if (up_left.getCurrentPosition() >= turn_counts ||
                up_right.getCurrentPosition() >= turn_counts ||
                back_right.getCurrentPosition() >= turn_counts ||
                back_left.getCurrentPosition() >= turn_counts){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            done = true;
        }
        return done;
    }

    private boolean driveToBox(int target){
        boolean done = false;
        if (mResetEncoder == false) {
            reset_encoders();
            run_to_position_Encoders();
            mResetEncoder = true;
        }
        int distcounts = myUtils.getNewCounts((int)distance,myUtils.NEVEREST60,3);
        int angle;
        up_right.setTargetPosition(distcounts);
        up_left.setTargetPosition(distcounts);
        back_left.setTargetPosition(distcounts);
        back_right.setTargetPosition(distcounts);

        up_right.setPower(-0.3);
        up_left.setPower(-0.3);
        back_left.setPower(-0.3);
        back_right.setPower(-0.3);

        telemetry.addData("Forward",distcounts);
        telemetry.addData("C1 Position",up_left.getCurrentPosition());
        telemetry.addData("T1 Position",up_left.getTargetPosition());
        angle = gyro.getIntegratedZValue();
        logMessage("Angle:",Integer.toString(angle));
        if ((angle - target) > 3){
            up_left.setPower(-0.6);
            up_right.setPower(-0.3);
            back_left.setPower(-0.6);
            back_right.setPower(-0.3);
        }
        if ((angle - target) < -3){
            up_left.setPower(-0.3);
            up_right.setPower(-0.6);
            back_left.setPower(-0.3);
            back_right.setPower(-0.6);
        }
        if (Math.abs(angle - target) < 3){
            up_left.setPower(-0.3);
            up_right.setPower(-0.3);
            back_left.setPower(-0.3);
            back_right.setPower(-0.3);
        }
        if (Math.abs(up_left.getCurrentPosition()) >= distcounts ||
                Math.abs(up_right.getCurrentPosition()) >= distcounts ||
                Math.abs(back_left.getCurrentPosition()) >= distcounts ||
                Math.abs(back_right.getCurrentPosition()) >= distcounts){
            up_right.setPower(0.0);
            up_left.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            telemetry.addData("Done",0);
            done = true;
        }
        return done;
    }
    private boolean driveToBox2(){
        boolean done = false;
        if (mResetEncoder == false) {
            reset_encoders();
            run_to_position_Encoders();
            mResetEncoder = true;
        }
        int distcounts = myUtils.getNewCounts((int)distance,myUtils.NEVEREST60,3);
        up_right.setTargetPosition(distcounts);
        up_left.setTargetPosition(distcounts);
        back_left.setTargetPosition(distcounts);
        back_right.setTargetPosition(distcounts);

        up_right.setPower(0.3);
        up_left.setPower(0.3);
        back_left.setPower(0.3);
        back_right.setPower(0.3);

        telemetry.addData("Forward",distcounts);
        telemetry.addData("C1 Position",up_left.getCurrentPosition());
        telemetry.addData("T1 Position",up_left.getTargetPosition());
        if (Math.abs(up_left.getCurrentPosition()) >= distcounts ||
                Math.abs(up_right.getCurrentPosition()) >= distcounts ||
                Math.abs(back_left.getCurrentPosition()) >= distcounts ||
                Math.abs(back_right.getCurrentPosition()) >= distcounts){
            up_right.setPower(0.0);
            up_left.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            telemetry.addData("Done",0);
            done = true;
        }
        return done;
    }

    private void reset_encoders(){
        up_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void turn_left(){
        int turn_distance = ((9 * 9) * 3)/4;
        turn_counts = myUtils.getNewCounts(turn_distance,myUtils.NEVEREST60,3);

        up_right.setTargetPosition(turn_counts);
        up_left.setTargetPosition(turn_counts);
        back_left.setTargetPosition(turn_counts);
        back_right.setTargetPosition(turn_counts);

        up_left.setPower(-0.3);
        up_right.setPower(0.3);
        back_left.setPower(-0.3);
        back_right.setPower(0.3);

        if (up_left.getCurrentPosition() >= turn_counts || up_right.getCurrentPosition() >= turn_counts || back_right.getCurrentPosition() >= turn_counts || back_left.getCurrentPosition() >= turn_counts){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
        }
    }
    private void robot_Travel(String destination_Turn, double iNumberOfTurns){
        turn_counts = myUtils.getNewCounts( (int) ((QUARTER_TURN * iNumberOfTurns) - offset),myUtils.NEVEREST60,3); //todo: tune turning
        telemetry.addData("Entering  State Turn %",turn_counts);
        logMessage ("Entering  State Turn ",Integer.toString(turn_counts));

        if (destination_Turn.equals(TRAVEL_DIRECTION.DRIVE_LEFT.name())) {
            up_right.setTargetPosition(turn_counts);
            up_left.setTargetPosition(0);
            back_left.setTargetPosition(0);
            back_right.setTargetPosition(turn_counts);

            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            logMessage("Up Power" , Double.toString(up_right.getPower()));
            logMessage("Back Power", Double.toString(back_right.getPower()));

            up_left.setPower(0.3);
            back_left.setPower(0.3);
            up_right.setPower(-0.3);
            back_right.setPower(-0.3);

            logMessage("Left Turn C2 Position", Integer.toString(up_right.getCurrentPosition()) );
            logMessage("Left Turn T2 Position", Integer.toString(up_right.getTargetPosition()) );
        }
        else if (destination_Turn.equals(TRAVEL_DIRECTION.DRIVE_RIGHT.name())) {
            up_right.setTargetPosition(0);
            up_left.setTargetPosition(turn_counts);
            back_left.setTargetPosition(turn_counts);
            back_right.setTargetPosition(0);

            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //logMessage("Right Turn:",Double.toString(up_right.getTargetPosition()));
            //logMessage("Right Counts:",Double.toString(up_right.getCurrentPosition()));
            up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            up_left.setPower(0.3);
            back_left.setPower(0.3);
            up_right.setPower(0);
            back_right.setPower(0);

            logMessage("Right Turn C2 Position", Integer.toString(up_left.getCurrentPosition()));
            logMessage("Right Turn T2 Position",Integer.toString( up_left.getTargetPosition()));
        }
        else if (destination_Turn.equals(TRAVEL_DIRECTION.DRIVE_CENTER.name()) ) {
            up_right.setTargetPosition(turn_counts);
            up_left.setTargetPosition(turn_counts);
            back_left.setTargetPosition(turn_counts);
            back_right.setTargetPosition(turn_counts);

            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            up_left.setPower(0.3);
            back_left.setPower(0.3);
            up_right.setPower(0.3);
            back_right.setPower(0.3);

            logMessage("CENTER C2 Position", Integer.toString(up_left.getCurrentPosition()));
            logMessage("CENTER T2 Position", Integer.toString(up_left.getTargetPosition()));
        }
        else {
            logMessage("Going nowhere :( ", Integer.toString(up_left.getCurrentPosition()));
        }

    }

    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii(QTAG, getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }
    private boolean turn180(){
        boolean mDone = false;
        if (!mReset){
            reset_encoders();
            run_to_position_Encoders();
        }
        turn_counts = myUtils.getNewCounts( (int) (QUARTER_TURN * 2),myUtils.NEVEREST60,3);
        up_right.setTargetPosition(turn_counts);
        up_left.setTargetPosition(turn_counts);
        back_left.setTargetPosition(turn_counts);
        back_right.setTargetPosition(turn_counts);

        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        logMessage("Up Power" , Double.toString(up_right.getPower()));
        logMessage("Back Power", Double.toString(back_right.getPower()));

        up_left.setPower(0.3);
        back_left.setPower(0.3);
        up_right.setPower(-0.3);
        back_right.setPower(-0.3);

        logMessage("Left Turn C2 Position", Integer.toString(up_right.getCurrentPosition()) );
        logMessage("Left Turn T2 Position", Integer.toString(up_right.getTargetPosition()) );
        if (up_left.getCurrentPosition() >= turn_counts){
            up_left.setPower(0.0);
            back_left.setPower(0.0);
            up_right.setPower(0.0);
            back_right.setPower(0.0);
            mDone = true;
        }
        return mDone;
    }
    private boolean swivel(double numturns){
        boolean mDone = false;
        double quarter = ((4.5 * Math.PI) * numturns);
        int turncounts = myUtils.getNewCounts((int)quarter,myUtils.NEVEREST60,3);
        up_left.setTargetPosition(-1 * turncounts);
        up_right.setTargetPosition(turncounts);
        back_left.setTargetPosition(-1 * turncounts);
        back_right.setTargetPosition(turncounts);

        up_left.setPower(-0.5);
        up_right.setPower(0.5);
        back_left.setPower(-0.5);
        back_right.setPower(0.5);

        if (Math.abs(up_left.getCurrentPosition()) >= turncounts ||
                Math.abs(up_right.getCurrentPosition()) >= turncounts ||
                Math.abs(back_left.getCurrentPosition()) >= turncounts ||
                Math.abs(back_right.getCurrentPosition()) >= turncounts){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            mDone = true;
        }
        return mDone;
    }

    public boolean gyroDrive ( double speed,
                               double mdistance,
                               double angle) {

        boolean complete = false;
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer = 0.0;
        double  leftSpeed;
        double  rightSpeed;



        // Determine new target position, and pass to motor controller
        moveCounts = myUtils.getNewCounts((int)mdistance,myUtils.NEVEREST60,3);


        // Set Target and Turn On RUN_TO_POSITION
        up_left.setTargetPosition(moveCounts);
        up_right.setTargetPosition(moveCounts);
        back_left.setTargetPosition(moveCounts);
        back_right.setTargetPosition(moveCounts);

        up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        up_left.setPower(-1 * speed);
        up_right.setPower(-1 * speed);
        back_left.setPower(-1 * speed);
        back_right.setPower(-1 * speed);

        // keep looping while we are still active, and BOTH motors are running.

        // adjust relative speed based on heading error.
        error = getError(angle);
        steer = getSteer(error, P_DRIVE_COEFF);
        logMessage("Steer:",Double.toString(steer));

        // if driving in reverse, the motor correction also needs to be reversed
        if (distance < 0)
            steer *= -1.0;

        leftSpeed = speed - steer;
        rightSpeed = speed + steer;

        // Normalize speeds if either one exceeds +/- 1.0;
        max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        up_left.setPower(leftSpeed);
        back_left.setPower(leftSpeed);
        up_right.setPower(rightSpeed);
        back_right.setPower(rightSpeed);
        if (up_left.getCurrentPosition() >= moveCounts || up_right.getCurrentPosition() >= moveCounts || back_left.getCurrentPosition() >= moveCounts || back_right.getCurrentPosition() >= moveCounts){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            complete = true;
        }
        return complete;
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    boolean gyroTurn (  double speed, int angle) {
        // keep looping while we are still active, and not on heading.
        boolean reached = false;
        int deviation = angle - Math.abs(gyro.getIntegratedZValue());
        if (deviation > 0) {
            up_left.setPower(speed);
            back_left.setPower(speed);
            up_right.setPower(-1 * speed);
            back_right.setPower(-1 * speed);
            logMessage("Pos",Integer.toString(gyro.getIntegratedZValue()));
            logMessage("Target",Integer.toString(angle));
            if (Math.abs(gyro.getIntegratedZValue()) > angle) {
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                reached = true;
            }
        }
        if (deviation < 0){
            up_left.setPower(-1 * speed);
            back_left.setPower(-1 * speed);
            up_right.setPower(speed);
            back_right.setPower(speed);
            logMessage("Pos",Integer.toString(gyro.getIntegratedZValue()));
            logMessage("Target",Integer.toString(angle));
            if (Math.abs(gyro.getIntegratedZValue()) < angle) {
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                reached = true;
            }
        }
        if (deviation <= 60 && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (voltage >= 13){
                power_threshold = 0.2;
            }
            if (velocity < 0.2)
                velocity = 0.2;
            logMessage("Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) > angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (deviation >= -60 && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (voltage >= 13){
                power_threshold = 0.2;
            }
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) < angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (Math.abs(deviation) < 10){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            reached = true;
        }
        // Update telemetry & Allow time for other processes to run.
        //telemetry.update();
        return reached;
    }
    boolean gyroTurn2 (  double speed, int angle) {
        // keep looping while we are still active, and not on heading.
        boolean reached = false;
        int deviation = angle - Math.abs(gyro.getIntegratedZValue());
        if (deviation > 0) {
            up_left.setPower(speed);
            back_left.setPower(speed);
            up_right.setPower(-1 * speed);
            back_right.setPower(-1 * speed);
            logMessage("Pos",Integer.toString(gyro.getIntegratedZValue()));
            logMessage("Target",Integer.toString(angle));
            if (Math.abs(gyro.getIntegratedZValue()) > angle) {
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                reached = true;
            }
        }
        if (deviation < 0){
            up_left.setPower(-1 * speed);
            back_left.setPower(-1 * speed);
            up_right.setPower(speed);
            back_right.setPower(speed);
            logMessage("Pos",Integer.toString(gyro.getIntegratedZValue()));
            logMessage("Target",Integer.toString(angle));
            if (Math.abs(gyro.getIntegratedZValue()) < angle) {
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                reached = true;
            }
        }
        if (deviation <= 100 && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (voltage > 13)
                power_threshold = 0.2;

            if (velocity < power_threshold)
                velocity = power_threshold;
            logMessage("Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) > angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (deviation >= -100 && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (velocity < power_threshold)
                velocity = power_threshold;
            if (voltage > 13)
                power_threshold = 0.2;
            logMessage("Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) < angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (Math.abs(deviation) < 10){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            reached = true;
        }
        // Update telemetry & Allow time for other processes to run.
        //telemetry.update();
        return reached;
    }
    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();

        // Update telemetry & Allow time for other processes to run.
        onHeading(speed, angle, P_TURN_COEFF);
        telemetry.update();


        // Stop all motion;

    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            logMessage("Steer:",Double.toString(steer));
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        up_left.setPower(leftSpeed);
        back_left.setPower(leftSpeed);
        up_right.setPower(rightSpeed);
        back_right.setPower(rightSpeed);

        // Display it for the driver.
        logMessage("Position",Integer.toString(gyro.getIntegratedZValue()));
        logMessage("Pos",Integer.toString(gyro.getHeading()));
        logMessage("Target", Double.toString(angle));
        logMessage("Err/St", Double.toString(error));
        logMessage("Steer",Double.toString(steer));
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();

        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
            sign = Range.clip(initial, -1, 1);
            init = true;
        }

        //theta -= 30;
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
            else if (Math.abs(deviation) <= 30 && !flip && passes == 0 && (Math.abs(initial) > 90)){
                speed /= decrement;
            }
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
            if (cursign == (sign * -1) && !reverse){
                passes++;
                reverse = true;
                logMessage("Flips:", Integer.toString(passes));
            }

            logMessage("Pos",Integer.toString(gyro.getIntegratedZValue()));
            logMessage("Target",Integer.toString(angle));
            logMessage("Speed:", Double.toString(speed));
            if (Math.abs(deviation) <= acceptance) {
                if (passes >= 1 && flip) {
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    passes = 0;
                    reached = true;
                }
            }
            else{
                if (passes >= 1 && !flip){
                    if (!reset){
                        mTurnTime.reset();
                    }
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
            if (Math.abs(deviation) <= acceptance) {
                if (passes >= 1 && flip) {
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    passes = 0;
                    reached = true;
                }
            }
            else{
                if (passes >= 1 && !flip){
                    if (!reset){
                        mTurnTime.reset();
                    }
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
    /*
    todo:
        **check on 4th motor (not running in auto) 10/18
        **run vuforia seperately 10/18
        *reimplement code (vuforia is last priority) 10/18
        smarter vuforia (angle correction)
        finished auto (go to box)
        *states/code for slides
        color sensing
        search time with sensors
        *placing glyphs(duh)
        *releasing glyphs
        possible height detection for existing glyphs/debris
        *hold the dang glyph(duh)
        clean code
        less parameters 10/22
        *check getCounts() formula
        *!THRESHOLD contingency!*
        opt:
            move back 6 in
            about face
     */
}

