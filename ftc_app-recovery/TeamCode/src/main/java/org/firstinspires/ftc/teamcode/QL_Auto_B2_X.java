package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by Arnav on 10/8/2017.
 */
@Autonomous(name = "State: QL_Auto_B2_X",group = "State:")
public class QL_Auto_B2_X extends OpMode{
    private State mRobotState;
    private GRAB_STATE grabstate;
    private boolean mResetEncoder = false;
    private ElapsedTime mStateTime = new ElapsedTime();
    private TURN_STATE turnstate;
    private COLUMN destination = COLUMN.CENTER;
    private ElapsedTime mTurnStateTime = new ElapsedTime();
    private ElapsedTime mGrabTime = new ElapsedTime();

    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;
    //final int THRESHOLD = 1;

    final String QTAG= "QL8719";

    //boolean bPrevState = false;
    //boolean bCurrState = false;
    boolean bLedOn = false;
    boolean vuMarkfound = false;
    boolean zero = false;
    boolean initing = false;


    double delta;
    int target;
    int grabcount = 0;
    double offset = 0.0;
    double velocity = 0.0;
    double voltageval;
    double leaveplatform = 19;
    double power_threshold;

    double deviation;
    double kickout = 0.5;

    boolean done = false;
    int turn;

    int sigma;
    double rsigma;

    double deltax;
    double pos = 0;
    long sum = 0;
    int isocounter = 0;

    boolean left = false;
    boolean center = false;
    boolean right = false;
    boolean initial = false;
    boolean inverted = false;
    boolean calibrate = false;
    boolean exit = false;

    int distance = 5;
    int turn_counts;
    int i;
    int slidecounts;
    int counter = 0;
    final int THRESHOLD = 4;

    DcMotor up_right;
    DcMotor up_left;
    DcMotor back_right;
    DcMotor back_left;
    DcMotor slide;
    DcMotor slide2;

    ModernRoboticsI2cGyro gyro;

    VoltageSensor voltage;

    Servo grab_right;
    Servo grab_left;
    Servo grab_top;
    Servo stick_x;
    Servo stick_y;

    ColorSensor colorsensor;

    ModernRoboticsTouchSensor touch;
    ModernRoboticsAnalogOpticalDistanceSensor odr;

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
    private enum GRAB_STATE{
        ISOLATE1,
        ISOLATE2,
        ISOLATE3,
        PUSH,
        RETRACT,
        TEST
    }
    private enum TURN_STATE{
        STATE_1,
        RESET,
        DELTA,
        STATE_2
    }
    private enum COLUMN{
        LEFT,
        CENTER,
        RIGHT
    }
    private enum TRAVEL_DIRECTION {
        DRIVE_RIGHT,
        DRIVE_LEFT,
        DRIVE_CENTER
    }
    private enum State{
        STATE_INITIAL,
        STATE_DROPSTICK1,
        STATE_LIFTBLOCK,
        STATE_COLORSCAN,
        STATE_LEAVE_PLATFORM,
        STATE_DRIVE_TO_BOX,
        STATE_TURN,
        STATE_LOWER_GLYPH,
        STATE_PLACE,
        STATE_PREPARE,
        STATE_RENDEZVOUS,
        STATE_POSITION,
        STATE_TRAVEL,
        STATE_GRAB,
        STATE_REMOVE,
        STATE_ISOLATE,
        STATE_BACK,
        STATE_LIFT,
        STATE_CORRECT,
        STATE_RETRACT,
        STATE_SWIVEL,
        STATE_LIFT2,
        STATE_RETURN,
        STATE_RECOVER,
        STATE_POS2,
        STATE_DEVIATE,
        STATE_ALIGN,
        STATE_SCORE,
        STATE_CLEAR,
        STATE_REINFORCE,
        STATE_PARK,
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
        grab_top = hardwareMap.servo.get("grab_top");

        stick_x = hardwareMap.servo.get("stick_y");
        stick_y = hardwareMap.servo.get("stick_x");


        colorsensor = hardwareMap.colorSensor.get("colorsensor");

        touch = hardwareMap.get(ModernRoboticsTouchSensor.class, "touch");
        odr = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "odr");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        voltage = hardwareMap.voltageSensor.get("Motor Controller 1");

        //vfa_init ();


        //colorsensor = hardwareMap.//colorsensor.get("color_stick");
        //colorsensor.enableLed(false);

        back_left.setDirection(DcMotor.Direction.REVERSE);
        up_left.setDirection(DcMotor.Direction.REVERSE);

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
        stick_x.setPosition(0.0);

        grab_left.setPosition(0.0);
        grab_right.setPosition(1.0);

        logMessage("init :", " initializing Vuforia");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdxZ3kj/////AAAAGVcZGhm+xkRnm25bq3Zjd2RSQ2pfdOy7+EfbjO8XJD7NFRkWQ0Xx0uzVcMSKKXYgimvMnjKHFiFnaRiubni6joz57M2ei/tPxb54q7cGy6O/Yw9C9EE5OGGRaVppUDSG9hV7iWCpCb7PS3OOj9ST3grD+GfojQZOxaugVLxwqlNXF7KRODkbvBXpQ5bUGiRlL0k3AlhgUbHnlPes0hMQt/uZ+Dzg56ixG6W1SAbZpK6jubVTJZ9uOQ25hevaP5QF1nrpLQEWvqusNwc+W/BNL7//+hePPnRYZU4c9i0crjyG7R0f7UgQ/vLWppNTYnu6H8v/rY34rIIGyyJ8iXSoHSCcj0T9WObHDB3DHQeYQI+G";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        gyro.calibrate();
        calibrate = true;
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
        grab_left.setPosition(0.75);
        grab_right.setPosition(0.15);

        voltageval = voltage.getVoltage();

        newState(State.STATE_INITIAL);

        logMessage("start :", " loading  Vuforia trackables");
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTemplate.setName("relicVuMarkTemplate");
        logMessage("Leaving", " Start..");
    }


    public void loop(){
        switch (mRobotState){
            case STATE_INITIAL:
                telemetry.addData("State:","initial");
                if (!vuMarkfound)
                    vfa_lookup ();
                if (!calibrate){
                    gyro.calibrate();
                    calibrate = true;
                }
                if (!gyro.isCalibrating()) {
                    int pos = gyro.getIntegratedZValue();
                    if (pos != -1) {
                        newState(State.STATE_DROPSTICK1);
                    }
                    else{
                        calibrate = false;
                    }
                }
                break;
            case STATE_DROPSTICK1:
                stick_y.setPosition(0.05);
                if (!vuMarkfound)
                    vfa_lookup ();
                if (mStateTime.time() >= 1.0) {
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
                if (!vuMarkfound)
                    vfa_lookup ();
                if (mStateTime.time() >= 0.25){
                    if (colorsensor.red() > colorsensor.blue()){
                        stick_x.setPosition(0.8);
                        // stick_y.setPosition(1.0);
                        logMessage("Red Selected",Integer.toString(colorsensor.red()));
                        bFoundColor = true;
                    }
                    if (colorsensor.blue() > colorsensor.red()){
                        stick_x.setPosition(0.2);
                        logMessage("Blue Selected",Integer.toString(colorsensor.blue()));
                        bFoundColor = true;
                    }
                    if (mStateTime.time() >= 0.375 && !bFoundColor){
                        stick_x.setPosition(0.5);
                        kickout += 0.01;
                    }
                    if (kickout >= 3.0){
                        kickout = 3.0;
                    }
                    if ( bFoundColor || mStateTime.time() >= kickout) {
                        //stick_x.setPosition(0.10);
                        logMessage("Stick","Times up, Lift Stick Y");
                        logMessage("Y Val:", Double.toString(stick_y.getPosition()));
                        stick_y.setPosition(0.9);
                        newState(State.STATE_LIFTBLOCK);
                    }
                }
                break;
            /*case STATE_DROPSTICK3:
                telemetry.addData("State:","dropstick3");
                //stick_y.setPosition(0.8);
                stick_x.setPosition(0.45);
                if (mStateTime.time() >= 2.0){
                    newState(State.STATE_LEAVE_PLATFORM);
                    telemetry.addData("Done with stick",0);
                }
                break;*/
            case STATE_LIFTBLOCK:
                telemetry.addData("State:","Lift BLock");
                slide.setPower(0.3);
                stick_y.setPosition(1.0);
                colorsensor.enableLed(false);
                if (!vuMarkfound)
                    vfa_lookup ();
                if (mStateTime.time() >= 0.5) {
                    slide.setPower(0.0);
                    newState(State.STATE_LEAVE_PLATFORM);
                }
                if (voltageval >= 13){
                    leaveplatform = 18;
                }
                if (voltageval <= 12){
                    leaveplatform = 20;
                }
                break;
            case STATE_LEAVE_PLATFORM:
                //if (mStateTime.time() > 1.0) {
                logMessage("State:","Leave Platform");
                if (mStateTime.time() % (1 / 512) != 0){
                    counter++;
                    sum += Math.abs(gyro.getIntegratedZValue());
                    sigma = (int)sum / counter;
                }
                if (leave_platform()) {
                    reset_encoders();
                    run_to_position_Encoders();
                    //sigma = Math.abs(gyro.getIntegratedZValue());
                    rsigma = (sigma * Math.PI) / 180;
                    deltax = 19 * Math.sin(rsigma);
                    logMessage("Sigma:",Integer.toString(sigma));
                    logMessage("Delta-x:", Double.toString(deltax));
                    newState (State.STATE_TURN);
                }
                //}

                break;
            case STATE_TURN:
                //if (mStateTime.time() > 1.5) {
                //robot_Travel(TRAVEL_DIRECTION.DRIVE_LEFT.name(),1);
                if (robot_travel_v2(destination)){
                    logMessage("Result",Boolean.toString(robot_travel_v2(destination)));
                    grab_left.setPosition(0.1);
                    grab_right.setPosition(0.9);
                    run_without_Encoders();
                    newState(State.STATE_LOWER_GLYPH);
                }
                //}
                break;
            case STATE_LOWER_GLYPH:
                slide.setPower(-0.3);
                if (mStateTime.time() >= 0.3) {
                    slide.setPower(0.0);
                    grab_left.setPosition(0.1); // test
                    grab_right.setPosition(0.9); // test
                    newState(State.STATE_DRIVE_TO_BOX);
                }
                break;
            case STATE_DRIVE_TO_BOX:
                //if (mStateTime.time () > 1.0){
                /*distance = 3;
                if (driveToBox ()){
                    newState(State.STATE_PLACE);
                }*/
                powerdrive(1);
                if (mStateTime.time() >= 0.75){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    run_to_position_Encoders();
                    reset_encoders();
                    newState(State.STATE_PLACE);
                }
                //}
                break;
            case STATE_PLACE:
                distance = 6;
                //logMessage("State","Place");
                if (driveToBox2()){
                    if (voltageval > 12.8){
                        if (voltageval > 13.3){
                            velocity = 0.25;
                        }
                        else{
                            velocity = 0.35;
                        }
                        logMessage("Overshoot:",Double.toString(voltageval));
                    }
                    else{
                        velocity = 0.4;
                    }
                    run_without_Encoders();
                    newState(State.STATE_PREPARE);
                }
                break;
            case STATE_PREPARE:
                if (gyroTurn4(velocity,90, 60)){
                    logMessage("Entering:","X Maneuver");
                    newState(State.STATE_RENDEZVOUS);
                    run_to_position_Encoders();
                    reset_encoders();
                }
                if (mStateTime.time() >= 1.5){
                    logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                    newState(State.STATE_RENDEZVOUS);
                    run_to_position_Encoders();
                    reset_encoders();
                }
                break;
            case STATE_RENDEZVOUS:
                //run_without_Encoders();
                switch (destination){
                    case LEFT:
                        deviation = 15;//32
                        break;
                    case CENTER:
                        deviation = 9;//26
                        break;
                    case RIGHT:
                        deviation = 6;//20
                        break;
                }
                //powerdrive(2);
                if (rendezvous((int)deviation)){
                    logMessage("Target Achieved:",Integer.toString(gyro.getIntegratedZValue()));
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    if (voltageval > 12.8){
                        if (voltageval > 13.5){
                            velocity = 0.25;
                        }
                        else{
                            velocity = 0.3;
                        }
                        logMessage("Overshoot:",Double.toString(voltageval));
                    }
                    else{
                        velocity = 0.35;
                    }
                    run_without_Encoders();
                    newState(State.STATE_POSITION);
                }
                break;
            case STATE_POSITION:
                if (mStateTime.time() >= 0.25) {
                    if (gyroTurn6(velocity, 140)) {
                        logMessage("Entering Travel:", Integer.toString(gyro.getIntegratedZValue()));
                        newState(State.STATE_TRAVEL);
                    }
                    if (mStateTime.time() >= 2.0) {
                        logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                        newState(State.STATE_TRAVEL);
                    }
                }
                break;
            case STATE_TRAVEL:
                grab_left.setPosition(0.4);
                grab_right.setPosition(0.6);
                up_left.setPower(0.4);
                up_right.setPower(0.4);
                back_left.setPower(0.4);
                back_right.setPower(0.4);
                if (mStateTime.time() >= 1.5){
                    logMessage("Arrved at Glyph Pit:", Integer.toString(gyro.getIntegratedZValue()));
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
                if (mStateTime.time() >= 0.5){
                    newState(State.STATE_REMOVE);
                }
                break;
            case STATE_REMOVE:
                up_left.setPower(-0.5);
                up_right.setPower(-0.5);
                back_left.setPower(-0.5);
                back_right.setPower(-0.5);
                if (mStateTime.time() >= 0.5){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    newState(State.STATE_ISOLATE);
                }
            case STATE_ISOLATE:
                if (isolate()){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    logMessage("Grab Clear:", Integer.toString(gyro.getIntegratedZValue()));
                    newState(State.STATE_LIFT);
                }
                break;
            case STATE_LIFT:
                telemetry.addData("State:","Lift");
                logMessage("Power:", Double.toString(up_left.getPower()) + Double.toString(back_left.getPower()));
                slide.setPower(0.5);
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                if (mStateTime.time() >= 1.0) {
                    slide.setPower(0.0);
                    if (voltageval > 12.8){
                        if (voltageval > 13.3){
                            velocity = 0.2;
                        }
                        else{
                            velocity = 0.25;
                        }
                        logMessage("Overshoot:",Double.toString(voltageval));
                    }
                    else{
                        velocity = 0.3;
                    }
                    newState(State.STATE_CORRECT);
                }
                break;
            case STATE_CORRECT:
                if (mStateTime.time() >= 0.25) {
                    if (gyroTurn7(velocity, 140)) {
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        logMessage("Target Locked:", Integer.toString(gyro.getIntegratedZValue()));
                        if (voltageval > 12.8){
                            if (voltageval > 13.3){
                                velocity = 0.25;
                            }
                            else{
                                velocity = 0.3;
                            }
                            logMessage("Overshoot:",Double.toString(voltageval));
                        }
                        else{
                            velocity = 0.35;
                        }
                        newState(State.STATE_SWIVEL);
                    }
                }
                break;
            case STATE_RETRACT:
                double kickout = 0.75;
                if (isocounter >= 1){
                    kickout += (0.125 * isocounter);
                }
                up_left.setPower(-0.4);
                up_right.setPower(-0.4);
                back_left.setPower(-0.4);
                back_right.setPower(-0.4);
                if (mStateTime.time() >= kickout){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    if (voltageval > 12.8){
                        if (voltageval > 13.3){
                            velocity = 0.3;
                        }
                        else{
                            velocity = 0.35;
                        }
                        logMessage("Overshoot:",Double.toString(voltageval));
                    }
                    else{
                        velocity = 0.4;
                    }
                    if (!exit) {
                        newState(State.STATE_SWIVEL);
                    }
                    else{
                        newState(State.STATE_RECOVER);
                    }
                }
                break;
            case STATE_RECOVER:
                up_left.setPower(-0.4);
                up_right.setPower(-0.4);
                back_left.setPower(-0.4);
                back_right.setPower(-0.4);
                if (mStateTime.time() >= 1.0){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    if (voltageval > 12.8){
                        if (voltageval > 13.3){
                            velocity = 0.35;
                        }
                        else{
                            velocity = 0.4;
                        }
                        logMessage("Overshoot:",Double.toString(voltageval));
                    }
                    else{
                        velocity = 0.45;
                    }
                    target = 0;
                    newState(State.STATE_SCORE);
                }
                break;
            case STATE_BACK:
                up_left.setPower(0.4);
                up_right.setPower(0.4);
                back_left.setPower(0.4);
                back_right.setPower(0.4);
                if (mStateTime.time() >= 1.0){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    if (voltageval > 12.8){
                        if (voltageval > 13.3){
                            velocity = 0.35;
                        }
                        else{
                            velocity = 0.4;
                        }
                        logMessage("Overshoot:",Double.toString(voltageval));
                    }
                    else{
                        velocity = 0.45;
                    }
                    target = 0;
                    newState(State.STATE_SCORE);
                }
                break;
            case STATE_SWIVEL:
                if (mStateTime.time() >= 0.25) {
                    if (gyroTurn4(velocity, 330, 75)) {
                        logMessage("Returning:", Integer.toString(gyro.getIntegratedZValue()));
                        newState(State.STATE_BACK);
                    }
                    if (mStateTime.time() >= 2.0) {
                        target = target - 5;
                    }
                    if (mStateTime.time() >= 2.5) {
                        if (!(Math.abs(gyro.getIntegratedZValue()) >= 45 && Math.abs(gyro.getIntegratedZValue()) <= 180)) {
                            logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                            newState(State.STATE_BACK);
                        } else {
                            logMessage("Abort:", Integer.toString(gyro.getIntegratedZValue()));
                            newState(State.STATE_STOP);
                        }
                    }
                }
                break;
            case STATE_LIFT2:
                telemetry.addData("State:","Lift");
                slide.setPower(0.3);
                if (mStateTime.time() >= 1.0) {
                    slide.setPower(0.0);
                    velocity = 0.3;
                    newState(State.STATE_RETURN);
                }
                break;
            case STATE_RETURN:
                up_left.setPower(0.4);
                up_right.setPower(0.4);
                back_left.setPower(0.4);
                back_right.setPower(0.4);
                slide.setPower(0.2);
                if (mStateTime.time() >= 1.5){
                    logMessage("Arrved at Rendezvous:", Integer.toString(gyro.getIntegratedZValue()));
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    slide.setPower(0.0);
                    velocity = 0.4;
                    newState(State.STATE_ALIGN);
                }
                break;
            case STATE_ALIGN:
                if (gyroTurn5(velocity,90)){
                    run_to_position_Encoders();
                    reset_encoders();
                    logMessage("Preparing to Deviate:", Integer.toString(gyro.getIntegratedZValue()));
                    newState(State.STATE_DEVIATE);
                }
                if (mStateTime.time() >= 2.5){
                    run_to_position_Encoders();
                    reset_encoders();
                    logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                    newState(State.STATE_DEVIATE);
                }
                break;
            case STATE_DEVIATE:
                run_without_Encoders();
                powerdrive(2);
                if (mStateTime.time() >= deviation){
                    run_without_Encoders();
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    logMessage("Almost There...", Integer.toString(gyro.getIntegratedZValue()));
                    velocity = 0.4;
                    newState(State.STATE_POS2);
                }
                break;
            case STATE_POS2:
                if (gyroTurn3(velocity,0)){
                    logMessage("Positioned:", Integer.toString(gyro.getIntegratedZValue()));
                    newState(State.STATE_SCORE);
                }
                if (mStateTime.time() >= 2.5){
                    logMessage("Kickout:", Integer.toString(gyro.getIntegratedZValue()));
                    newState(State.STATE_SCORE);
                }
                break;
            case STATE_SCORE:
                powerdrive(1);
                if (mStateTime.time() >= 1.0){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    grab_left.setPosition(0.3);
                    grab_right.setPosition(0.7);
                    grab_top.setPosition(0.3);
                    logMessage("Backing Up:", Integer.toString(gyro.getIntegratedZValue()));
                    run_to_position_Encoders();
                    reset_encoders();
                    if (!exit){
                        newState(State.STATE_CLEAR);
                    }
                    else{
                        newState(State.STATE_STOP);
                    }
                }
                break;
            case STATE_CLEAR:
                distance = 4;
                //logMessage("State","Place");
                if (mStateTime.time() >= 0.25) {
                    if (driveToBox2()) {
                        velocity = 0.4;
                        grab_left.setPosition(0.1);
                        grab_right.setPosition(0.9);
                        run_without_Encoders();
                        newState(State.STATE_REINFORCE);
                    }
                }
                break;
            case STATE_REINFORCE:
                powerdrive(1);
                if (mStateTime.time() >= 0.5){
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                    logMessage("Backing Up:", Integer.toString(gyro.getIntegratedZValue()));
                    run_to_position_Encoders();
                    reset_encoders();
                    newState(State.STATE_PARK);
                }
                break;
            case STATE_PARK:
                distance = 6;
                //logMessage("State","Place");
                if (driveToBox2()){
                    velocity = 0.4;
                    run_without_Encoders();
                    newState(State.STATE_STOP);
                }
                break;
            case STATE_STOP:
                //if (mStateTime.time() > 1.0) { // for previous state to finish
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                stick_x.setPosition(1.0);
                colorsensor.enableLed(false);
                relicTrackables.deactivate();
                //stick_x.setPosition(0.45);
                //telemetry.addData("Ta-Da!", "I did what you asked");
                break;
            //}
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    private void powerdrive(int direction){
        up_right.setPower(0.3 * direction);
        up_left.setPower(0.3 * direction);
        back_left.setPower(0.3 * direction);
        back_right.setPower(0.3 * direction);
    }
    private void newState (QL_Auto_B2_X.State newState){
        // Reset State time and change to next state
        up_left.setPower(0.0);
        up_right.setPower(0.0);
        back_left.setPower(0.0);
        back_right.setPower(0.0);
        mStateTime.reset();
        mRobotState = newState;
        mResetEncoder = false;
    }
    private void newturnState (QL_Auto_B2_X.TURN_STATE newTurnState){
        mTurnStateTime.reset();
        turnstate = newTurnState;
        mResetEncoder = false;
    }
    private void newgrabState (QL_Auto_B2_X.GRAB_STATE newGrabState){
        mGrabTime.reset();
        grabstate = newGrabState;
        mResetEncoder = false;
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
    private void vfa_init () {
         /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdxZ3kj/////AAAAGVcZGhm+xkRnm25bq3Zjd2RSQ2pfdOy7+EfbjO8XJD7NFRkWQ0Xx0uzVcMSKKXYgimvMnjKHFiFnaRiubni6joz57M2ei/tPxb54q7cGy6O/Yw9C9EE5OGGRaVppUDSG9hV7iWCpCb7PS3OOj9ST3grD+GfojQZOxaugVLxwqlNXF7KRODkbvBXpQ5bUGiRlL0k3AlhgUbHnlPes0hMQt/uZ+Dzg56ixG6W1SAbZpK6jubVTJZ9uOQ25hevaP5QF1nrpLQEWvqusNwc+W/BNL7//+hePPnRYZU4c9i0crjyG7R0f7UgQ/vLWppNTYnu6H8v/rY34rIIGyyJ8iXSoHSCcj0T9WObHDB3DHQeYQI+G";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTemplate.setName("relicVuMarkTemplate");*/
    }
    private void vfa_lookup () {
        //if (!stop) {
        logMessage ("Runtime 1 is " , Double.toString(getRuntime() ) );

        relicTrackables.activate();
        logMessage("Entering STATE_VUMARK ","VuMark");
        logMessage ("Runtime 2 /is " , Double.toString(getRuntime() ) );

        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("Entering STATE_VUMARK ","Got VuMark" +  vuMark.toString());


        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            logMessage("VuMark", "  visible :: " + vuMark);

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                logMessage("VuMark"," --> LEFT" + vuMark.toString());
                //destination_Turn = TRAVEL_DIRECTION.DRIVE_LEFT.name();
                destination = COLUMN.LEFT;
                vuMarkfound = true;
                distance = 16;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                logMessage("VuMark","--> CENTER " + vuMark.toString());
                //destination_Turn = TRAVEL_DIRECTION.DRIVE_CENTER.name();
                destination = COLUMN.CENTER;
                vuMarkfound = true;
                distance = 29;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                logMessage("VuMark","--> RIGHT " + vuMark.toString());
                //destination_Turn = TRAVEL_DIRECTION.DRIVE_RIGHT.name();
                destination = COLUMN.RIGHT;
                vuMarkfound = true;
                distance = 35;
            }
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
        int forwardcounts = myUtils.getNewCounts(22,myUtils.NEVEREST60,3);
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
            up_right.setPower(0.5);
            up_left.setPower(0.5);
            back_left.setPower(0.5);
            back_right.setPower(0.5);
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
    private boolean rendezvous(int distance) {
        boolean done = false;
        if (!mResetEncoder) {
            reset_encoders();
            run_to_position_Encoders();
            mResetEncoder = true;
        }
        int forwardcounts = myUtils.getNewCounts(distance,myUtils.NEVEREST60,3);
        up_left.setTargetPosition(forwardcounts);
        up_right.setTargetPosition(forwardcounts);
        back_right.setTargetPosition(forwardcounts);
        back_left.setTargetPosition(forwardcounts);
        up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (up_left.getCurrentPosition() <= (5 * (forwardcounts / 6))){
            up_left.setPower(0.8);
            up_right.setPower(0.8);
            back_left.setPower(0.8);
            back_right.setPower(0.8);
        }
        else {
            up_right.setPower(0.5);
            up_left.setPower(0.5);
            back_left.setPower(0.5);
            back_right.setPower(0.5);
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
    private boolean secureJewel () {
        // DETECT COLOR
        boolean detected = false;
        /*
        int THRESHOLD = 1;
        telemetry.addData("Entering", "opMode is Active");
        // convert the RGB values to HSV values.
        Color.RGBToHSV(//colorsensor.red() * 8, //colorsensor.green() * 8, //colorsensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorsensor.alpha());
        telemetry.addData("Red  ", colorsensor.red());
        telemetry.addData("Green", colorsensor.green());
        telemetry.addData("Blue ", colorsensor.blue());
        telemetry.addData("Hue", hsvValues[0]);

        telemetry.addData("within securebeacon - Before", "THRESHOLD");

        if (colorsensor.blue() > THRESHOLD) {
            detected = true;
        } else {
            detected = false;
        }
        */
        return detected;
    }
    private boolean driveToBox(){
        boolean done = false;
        if (mResetEncoder == false) {
            reset_encoders();
            run_to_position_Encoders();
            mResetEncoder = true;
        }
        int distcounts = myUtils.getNewCounts(distance,myUtils.NEVEREST60,3);
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
        if (up_left.getCurrentPosition() >= distcounts ||
                up_right.getCurrentPosition() >= distcounts ||
                back_left.getCurrentPosition() >= distcounts ||
                back_right.getCurrentPosition() >= distcounts){
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
        int distcounts = myUtils.getNewCounts(distance,myUtils.NEVEREST60,3);
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
    private void push_button() {
        telemetry.addData("within PUSH BUTTON", "push");
        up_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        up_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        up_left.setPower(0.6);
        up_right.setPower(0.6);
    }

    private void scoot_back(){
        telemetry.addData("private", "scoot back");
        up_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        up_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        up_left.setPower(-0.6);
        up_right.setPower(-0.6);
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
    private void vfa_run () {

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
        // DbgLog.err("ERROR MESSAGE for uday");
        // DbgLog.msg("PLAIN MESSAGE for uday");

        if (destination_Turn.equals(TRAVEL_DIRECTION.DRIVE_LEFT.name())) {
            up_right.setTargetPosition(turn_counts);
            up_left.setTargetPosition(0);
            back_left.setTargetPosition(0);
            back_right.setTargetPosition(turn_counts);

            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            logMessage("Up Power" , Double.toString(up_right.getPower()));
            logMessage("Back Power", Double.toString(back_right.getPower()));

            up_left.setPower(0.0);
            back_left.setPower(0.0);
            up_right.setPower(0.3);
            back_right.setPower(0.3);

            logMessage("Left Turn C2 Position", Integer.toString(up_right.getCurrentPosition()) );
            logMessage("Left Turn T2 Position", Integer.toString(up_right.getTargetPosition()) );

            if (up_left.getCurrentPosition() >= turn_counts ||
                    up_right.getCurrentPosition() >= turn_counts ||
                    back_left.getCurrentPosition() >= turn_counts ||
                    back_right.getCurrentPosition() >= turn_counts){
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);

            }
        }
        else if (destination_Turn.equals(TRAVEL_DIRECTION.DRIVE_RIGHT.name())) {
            up_right.setTargetPosition(0);
            up_left.setTargetPosition(turn_counts);
            back_left.setTargetPosition(turn_counts);
            back_right.setTargetPosition(0);

            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            up_left.setPower(0.3);
            back_left.setPower(0.3);
            up_right.setPower(-0.3);
            back_right.setPower(-0.3);


            logMessage("Right Turn C2 Position", Integer.toString(up_left.getCurrentPosition()));
            logMessage("Right Turn T2 Position",Integer.toString( up_left.getTargetPosition()));

            if (up_left.getCurrentPosition() >= turn_counts ||
                    up_right.getCurrentPosition() >= turn_counts ||
                    back_left.getCurrentPosition() >= turn_counts ||
                    back_right.getCurrentPosition() >= turn_counts){
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);

            }
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

            up_left.setPower(0.1);
            back_left.setPower(0.1);
            up_right.setPower(0.1);
            back_right.setPower(0.1);


            logMessage("CENTER C2 Position", Integer.toString(up_left.getCurrentPosition()));
            logMessage("CENTER T2 Position", Integer.toString(up_left.getTargetPosition()));

            if (up_left.getCurrentPosition() >= turn_counts ||
                    up_right.getCurrentPosition() >= turn_counts ||
                    back_left.getCurrentPosition() >= turn_counts ||
                    back_right.getCurrentPosition() >= turn_counts){
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);

            }
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
        if (deviation <= -75 && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Positive Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) > angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (deviation >= -75 && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Negative Slowing Down:", Integer.toString(deviation));
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
        if (deviation <= 90 && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (voltageval > 13)
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
        if (deviation >= -90 && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (velocity < 0.2)
                velocity = 0.2;
            if (voltageval > 13)
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
    boolean gyroTurn3(  double speed, int angle) {
        // keep looping while we are still active, and not on heading.
        boolean reached = false;
        int deviation = angle - Math.abs(gyro.getIntegratedZValue());
        if (deviation > 0) {
            up_left.setPower(-1 * speed);
            back_left.setPower(-1 * speed);
            up_right.setPower(speed);
            back_right.setPower(speed);
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
            up_left.setPower(speed);
            back_left.setPower(speed);
            up_right.setPower(-1 * speed);
            back_right.setPower(-1 * speed);
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
        if (deviation <= 45 && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Positive Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) > angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (deviation >= -45 && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Negative Slowing Down:", Integer.toString(deviation));
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
    boolean gyroTurn4 (  double speed, int angle, int cone) {
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
        if (deviation <= cone && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Positive Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) > angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (deviation >= -cone && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Negative Slowing Down:", Integer.toString(deviation));
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
    boolean gyroTurn5 (  double speed, int angle, int cone) {
        // keep looping while we are still active, and not on heading.
        boolean reached = false;
        int deviation = angle - gyro.getIntegratedZValue();
        if (deviation > 0) {
            up_left.setPower(-1 * speed);
            back_left.setPower(-1 * speed);
            up_right.setPower(speed);
            back_right.setPower(speed);
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
            up_left.setPower(speed);
            back_left.setPower(speed);
            up_right.setPower(-1 * speed);
            back_right.setPower(-1 * speed);
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
        if (deviation <= cone && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Positive Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 5) { //Math.abs(gyro.getIntegratedZValue) > angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (deviation >= -cone && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Negative Slowing Down:", Integer.toString(deviation));
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
    boolean gyroTurn6 (  double speed, int angle) {
        // keep looping while we are still active, and not on heading.
        boolean reached = false;
        int deviation = angle - gyro.getIntegratedZValue();
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
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Positive Slowing Down:", Integer.toString(deviation));
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
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Negative Slowing Down:", Integer.toString(deviation));
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
    boolean gyroTurn5 (  double speed, int angle) {
        // keep looping while we are still active, and not on heading.
        boolean reached = false;
        int deviation = angle - gyro.getIntegratedZValue();
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
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Positive Slowing Down:", Integer.toString(deviation));
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
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Negative Slowing Down:", Integer.toString(deviation));
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
    boolean gyroTurn7 (  double speed, int angle) {
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
        if (deviation <= 15 && deviation > 0){
            //if ((mStateTime.time() % 0.5) == 0) { //mod used to slow down motors every second
            velocity = velocity / 2;
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Positive Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 2.5) { //Math.abs(gyro.getIntegratedZValue) > angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (deviation >= -15 && deviation < 0){
            //if ((mStateTime.time() % 0.5) == 0) {
            velocity = velocity / 2;//a bit manual, to be fixed later
            //}
            if (velocity < 0.2)
                velocity = 0.2;

            logMessage("Negative Slowing Down:", Integer.toString(deviation));
            if (Math.abs(deviation) < 2.5) { //Math.abs(gyro.getIntegratedZValue) < angle
                up_left.setPower(0.0);
                up_right.setPower(0.0);
                back_left.setPower(0.0);
                back_right.setPower(0.0);
                logMessage("Stopping:",Integer.toString(deviation));
                reached = true;
            }
        }
        if (Math.abs(deviation) == 0.0){
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
        opt:
            move back 6 in
            about face
        tasklist:
            Score jewel
            leave platform according to vuforia
            turn 90
            release the glyph
            push the glyph in
            back up 6 in
     */


    private boolean robot_travel_v2(COLUMN input){

        if (!initial) {
            delta = 0.0;
            turn = 0;
            logMessage("Phase","Initial");
            switch (input) {

                case LEFT:
                    delta = 4 - deltax;//4.25;//1.25
                    if (zero){
                        delta = 0;
                    }
                    //distance = 5;
                    break;
                case CENTER:
                    delta = 9 - deltax; //8.05;//5.05
                    break;
                case RIGHT:
                    delta = 15 - deltax; //9.28; //6.28
                    break;
            }
            initial = true;
            if (voltageval > 12.8){
                if (voltageval > 13.3){
                    velocity = 0.3;
                }
                else{
                    velocity = 0.35;
                }
                logMessage("Overshoot:",Double.toString(voltageval));
            }
            else{
                velocity = 0.4;
            }
            turn = myUtils.getNewCounts((int)delta,FTCUtils.NEVEREST60,3);
            newturnState(TURN_STATE.STATE_1);
            run_without_Encoders();
        }
        if (initial){
            switch (turnstate){
                case STATE_1:
                    /*up_left.setTargetPosition(turn);
                    back_left.setTargetPosition(turn);
                    up_right.setTargetPosition(0);
                    back_right.setTargetPosition(0);
                    up_left.setPower(0.5);
                    back_left.setPower(0.5);
                    up_right.setPower(0.0);
                    back_right.setPower(0.0);
                    logMessage("Position",Double.toString(back_left.getCurrentPosition()));
                    logMessage("Target",Double.toString((back_left.getTargetPosition())));
                    logMessage("Intended",Double.toString(turn));
                    if (up_left.getCurrentPosition() >= turn || back_left.getCurrentPosition() >= turn){
                        up_left.setPower(0.0);
                        back_left.setPower(0.0);
                        newturnState(turnstate.RESET);
                    }*/
                    //robot_Travel(TRAVEL_DIRECTION.DRIVE_RIGHT.name(),0.8);
                    if (gyroTurn4(velocity,90, 60)){
                        up_left.setPower(0.0);
                        back_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_right.setPower(0.0);
                        run_to_position_Encoders();
                        reset_encoders();
                        newturnState(TURN_STATE.RESET);
                    }
                    if (mTurnStateTime.time() >= 3.0){
                        up_left.setPower(0.0);
                        back_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_right.setPower(0.0);
                        newturnState(TURN_STATE.RESET);
                        //done = true;
                    }
                    break;
                case RESET:
                    reset_encoders();
                    run_to_position_Encoders();
                    //newturnState(turnstate.STATE_2);
                    //done = true;
                    newturnState(TURN_STATE.DELTA);
                    break;
                case DELTA:
                    up_left.setTargetPosition(turn);
                    back_left.setTargetPosition(turn);
                    up_right.setTargetPosition(turn);
                    back_right.setTargetPosition(turn);
                    up_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    up_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (turn >= 0) {
                        if (up_left.getCurrentPosition() <= (2 * (turn / 3))){
                            up_left.setPower(0.8);
                            up_right.setPower(0.8);
                            back_left.setPower(0.8);
                            back_right.setPower(0.8);
                        }
                        else {
                            up_right.setPower(0.4);
                            up_left.setPower(0.4);
                            back_left.setPower(0.4);
                            back_right.setPower(0.4);
                        }
                    }
                    if (turn < 0){
                        if (up_left.getCurrentPosition() <= (2 * (turn / 3))){
                            up_left.setPower(-0.8);
                            up_right.setPower(-0.8);
                            back_left.setPower(-0.8);
                            back_right.setPower(-0.8);
                        }
                        else {
                            up_right.setPower(-0.4);
                            up_left.setPower(-0.4);
                            back_left.setPower(-0.4);
                            back_right.setPower(-0.4);
                        }
                    }
                    logMessage("Destination",Integer.toString(turn) + " " + Integer.toString(up_right.getTargetPosition()));
                    logMessage("Current",Integer.toString(up_right.getCurrentPosition()));

                    if (Math.abs(up_right.getCurrentPosition())>= Math.abs(turn) ||
                            Math.abs(up_left.getCurrentPosition()) >= Math.abs(turn) ||
                            Math.abs(back_left.getCurrentPosition()) >= Math.abs(turn) ||
                            Math.abs(back_right.getCurrentPosition()) >= Math.abs(turn)){
                        logMessage("Target Delta:", "Achieved" + up_right.getCurrentPosition());
                        up_left.setPower(0.0);
                        back_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_right.setPower(0.0);
                        if (voltageval > 12.8){
                            if (voltageval > 13.3){
                                velocity = 0.25;
                            }
                            else{
                                velocity = 0.35;
                            }
                            logMessage("Overshoot:",Double.toString(voltageval));
                        }
                        else{
                            velocity = 0.4;
                        }
                        run_without_Encoders();
                        newturnState(TURN_STATE.STATE_2);
                    }
                    break;
                case STATE_2:
                    /*up_right.setTargetPosition((int)turn - 140);
                    back_right.setTargetPosition((int)turn- 140);
                    up_left.setTargetPosition(0);
                    back_left.setTargetPosition(0);
                    up_right.setPower(0.5);
                    back_right.setPower(0.5);
                    up_left.setPower(0.0);
                    back_left.setPower(0.0);
                    logMessage("Position",Double.toString(back_right.getCurrentPosition()));
                    logMessage("Target",Double.toString((back_right.getTargetPosition())));
                    logMessage("Intended",Double.toString(turn));
                    if (up_right.getCurrentPosition() >= (turn - 140) || back_right.getCurrentPosition() >= (turn - 140)){
                        up_right.setPower(0.0);
                        back_right.setPower(0.0);
                        done = true;
                    }*/
                    //robot_Travel(TRAVEL_DIRECTION.DRIVE_LEFT.name(),0.8);
                    if (mTurnStateTime.time() >= 0.5) {
                        if (gyroTurn4(velocity, 0, 60)) {
                            up_left.setPower(0.0);
                            back_left.setPower(0.0);
                            up_right.setPower(0.0);
                            back_right.setPower(0.0);
                            done = true;
                        }
                        if (mTurnStateTime.time() >= 3.0) {
                            up_left.setPower(0.0);
                            back_left.setPower(0.0);
                            up_right.setPower(0.0);
                            back_right.setPower(0.0);
                            done = true;
                        }
                    }
                    break;

            }
        }
        return done;
    }
    private boolean isolate(){
        boolean complete = false;
        if (!initing){
            up_left.setPower(0.0);
            up_right.setPower(0.0);
            back_left.setPower(0.0);
            back_right.setPower(0.0);
            if (voltageval > 12.8){
                if (voltageval > 13.3){
                    velocity = 0.25;
                }
                else{
                    velocity = 0.35;
                }
                logMessage("Overshoot:",Double.toString(voltageval));
            }
            else{
                velocity = 0.4;
            }
            initing = true;
            newgrabState(GRAB_STATE.TEST);
        }
        else{
            switch (grabstate){
                case ISOLATE1:
                    if (gyroTurn7(velocity, 150)){
                        logMessage("Turn 1 Complete:", Integer.toString(gyro.getIntegratedZValue()));
                        grab_left.setPosition(0.3);
                        grab_right.setPosition(0.7);
                        grab_top.setPosition(0.3);
                        if (voltageval > 12.8){
                            if (voltageval > 13.3){
                                velocity = 0.25;
                            }
                            else{
                                velocity = 0.35;
                            }
                            logMessage("Overshoot:",Double.toString(voltageval));
                        }
                        else{
                            velocity = 0.4;
                        }
                        grabcount = 1;
                        newgrabState(GRAB_STATE.PUSH);
                    }
                    break;
                case ISOLATE2:
                    if (gyroTurn7(velocity, 135)){
                        logMessage("Turn 2 Complete:", Integer.toString(gyro.getIntegratedZValue()));
                        grab_left.setPosition(0.3);
                        grab_right.setPosition(0.7);
                        grab_top.setPosition(0.3);
                        if (voltageval > 12.8){
                            if (voltageval > 13.3){
                                velocity = 0.25;
                            }
                            else{
                                velocity = 0.35;
                            }
                            logMessage("Overshoot:",Double.toString(voltageval));
                        }
                        else{
                            velocity = 0.4;
                        }
                        grabcount = 2;
                        newgrabState(GRAB_STATE.PUSH);
                    }
                    break;
                case ISOLATE3:
                    if (gyroTurn7(velocity, 140)){
                        logMessage("Turn 3 Complete:", Integer.toString(gyro.getIntegratedZValue()));
                        grab_left.setPosition(0.3);
                        grab_right.setPosition(0.7);
                        grab_top.setPosition(0.3);
                        if (voltageval > 12.8){
                            if (voltageval > 13.3){
                                velocity = 0.25;
                            }
                            else{
                                velocity = 0.35;
                            }
                            logMessage("Overshoot:",Double.toString(voltageval));
                        }
                        else{
                            velocity = 0.4;
                        }
                        grabcount = 3;
                        newgrabState(GRAB_STATE.PUSH);
                    }
                    break;
                case PUSH:
                    grab_left.setPosition(0.3);
                    grab_right.setPosition(0.7);
                    up_left.setPower(0.8);
                    up_right.setPower(0.8);
                    back_left.setPower(0.8);
                    back_right.setPower(0.8);
                    if (mGrabTime.time() >= 0.5){
                        up_left.setPower(0.0);
                        up_right.setPower(0.0);
                        back_left.setPower(0.0);
                        back_right.setPower(0.0);
                        grab_left.setPosition(0.75);
                        grab_right.setPosition(0.25);
                        grab_top.setPosition(1.0);
                        logMessage("In Glyph Pit:", Double.toString(odr.getLightDetected()));
                        newgrabState(GRAB_STATE.RETRACT);
                    }
                    break;
                case RETRACT:
                    if (mGrabTime.time() >= 0.25) {
                        up_left.setPower(-0.4);
                        up_right.setPower(-0.4);
                        back_left.setPower(-0.4);
                        back_right.setPower(-0.4);
                        if (mGrabTime.time() >= 0.75) {
                            up_left.setPower(0.0);
                            up_right.setPower(0.0);
                            back_left.setPower(0.0);
                            back_right.setPower(0.0);
                            logMessage("Out of Glyph Pit:", Double.toString(odr.getLightDetected()));
                            pos = odr.getLightDetected();
                            newgrabState(GRAB_STATE.TEST);
                        }
                    }
                    break;
                case TEST:
                    grab_left.setPosition(0.75);
                    grab_right.setPosition(0.25);
                    grab_top.setPosition(1.0);
                    if (mGrabTime.time() >= 0.25) {
                        pos = odr.getLightDetected();
                        if (test_grab()) {
                            if (pos > 0.35) {
                                up_left.setPower(0.0);
                                up_right.setPower(0.0);
                                back_left.setPower(0.0);
                                back_right.setPower(0.0);
                                logMessage("Done with Procedure:", Integer.toString(gyro.getIntegratedZValue()));
                                complete = true;
                            }
                            /*else{
                                switch (grabcount) {
                                    case 0:
                                        newgrabState(GRAB_STATE.ISOLATE1);
                                        break;
                                    case 1:
                                        newgrabState(GRAB_STATE.ISOLATE2);
                                        break;
                                    case 2:
                                        logMessage("Kickout:", Double.toString(odr.getLightDetected()));
                                        complete = true;
                                        break;
                                    case 3:
                                        logMessage("Kickout:", Double.toString(odr.getLightDetected()));
                                        complete = true;
                                        break;
                                }
                            }*/
                        } else {
                            switch (grabcount) {
                                case 0:
                                    newgrabState(GRAB_STATE.ISOLATE1);
                                    break;
                                case 1:
                                    newgrabState(GRAB_STATE.ISOLATE2);
                                    break;
                                case 2:
                                    logMessage("Kickout:", Double.toString(odr.getLightDetected()));
                                    complete = true;
                                    exit = true;
                                    break;
                                case 3:
                                    logMessage("Kickout:", Double.toString(odr.getLightDetected()));
                                    complete = true;
                                    exit = true;
                                    break;
                            }
                        }
                    }
                    break;
            }
        }
        return complete;
    }
    private boolean test_grab(){
        double pos = touch.getValue();
        boolean grab = false;
        if (!(pos == 1.0)){
            logMessage("Interference Detected:", Double.toString(pos));
            grab = true;
        }
        else{
            logMessage("Interference Absent:", Double.toString(pos));
            grab = false;
        }
        return grab;
    }
}

