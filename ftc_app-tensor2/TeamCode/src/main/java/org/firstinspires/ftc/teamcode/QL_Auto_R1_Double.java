package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.control.Pose2d;
import org.firstinspires.ftc.teamcode.movement.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Sample_Wall;

import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

@Autonomous(name = "QL_Auto_R1_Double", group = "Competition")
public class QL_Auto_R1_Double extends OpMode {
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
    Hanger hanger;
    Arm arm;
    double radius = 24.5;
    double heading = -53.0;
    double offset = 0.0;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    Gimbel g;
    boolean first = true;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AR0LYP3/////AAABmQ7b/5H8u0A7uZvKeYoEg0JVsyhd/1+GlaxSV0Pw+lm+4zOHSxXgvk04VjLKBIO1iwO6StAmpOwSGYC4RYIwg2IwnKVVmKbn1XD11+qF/sYPz4xu+tvrexORZ8575m5Blh5GP+EU61ej86TfAGDVViwicrIvS/iUDncuiCE7BY7oogBfYcum7fCG5JIX41+NSMhrwXdTNxXTTPVF6tlS15g9zJE2ds1I9+OmUwITV8iK6udThWEE/beK7xRPdjzwf1o75C+WTlbSMUDvk2W7rNVqfqWNrxQFqenn29mOH1qAGfVL2J4j9Vfq/ZQ3lyDaBaNqoNUduTn2hEyO8oF1VxTUz1hoaddrz04STWGBiBon";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    Servo box_left;
    Servo box_right;
    Servo marker;
    Sample_Wall wall;
    boolean flip = false;
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime precision = new ElapsedTime();
    ElapsedTime mRunTime = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    Pose2d chosen = new Pose2d(-1, -1, 0);
    int pos = -1;
    double height = 25.0;
    double x = 0.0;

    State mRobotState = State.STATE_START;
    ElapsedTime mStateTime = new ElapsedTime();
    PriorityQueue<Pose2d> recog;
    Pose2d memo = new Pose2d(0, 0, 0);

    class PoseComparator implements Comparator<Pose2d>{
        public int compare(Pose2d o1, Pose2d o2){
            if (o1.dist(new Pose2d(0, 0, 0)) > o2.dist(new Pose2d(0, 0, 0))){
                return -1;
            }
            else if (o1.dist(new Pose2d(0, 0, 0)) < o2.dist(new Pose2d(0, 0, 0))){
                return 1;
            }
            else{
                return 0;
            }
        }
    }

    public enum State{
        STATE_START,
        STATE_SCAN,
        STATE_SCAN2,
        STATE_DROP,
        STATE_EXTEND,
        STATE_DELATCH,
        STATE_DELATCH2,
        STATE_PREP,
        STATE_SAMPLE,
        STATE_SAMPLE2,
        STATE_CENTER2,
        STATE_CENTER,
        STATE_TRANSFER,
        STATE_CHECKPOINT,
        STATE_TURN,
        STATE_TURN2,
        STATE_TURN3,
        STATE_ALIGN,
        STATE_TRAVEL,
        STATE_TRAVEL2,
        STATE_TRAVEL3,
        STATE_RETURN,
        STATE_SECURE,
        STATE_STOP
    }

    public void init(){
        hanger = new Hanger(hardwareMap);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        box_left = hardwareMap.get(Servo.class, "box_left");
        box_right = hardwareMap.get(Servo.class, "box_right");
        marker = hardwareMap.get(Servo.class, "tm");
        marker.setPosition(1.0);
        box_left.setPosition(1.0);
        box_right.setPosition(0.0);
        wall = new Sample_Wall(hardwareMap);

        arm = new Arm(sweeper, arm1, hardwareMap.get(Servo.class, "back"));

        g = new Gimbel(hardwareMap);
        g.GoTo(-0.5, -0.19);

        drive = new Mecanum_Drive(hardwareMap);
        drive.calibrate();
        //drive.loadModifier(hardwareMap.appContext);

        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public void start(){
        newState(State.STATE_START);
        tfod.activate();
    }

    public void loop(){
        switch (mRobotState){
            case STATE_START:
                telemetry.addData("Entering Auto", "1");
                newState(State.STATE_SCAN);
                break;
            case STATE_DROP:
                //insert drop code once hanging is complete
                hanger.drop();
                if (hanger.getHang().getCurrentPosition() >= 7500) {
                    newState(State.STATE_EXTEND);
                    arm.move(0.0, 2, true);
                    drive.engage();
                    telemetry.addData("Wheels on Ground: ", pose.toString());
                }
                else{
                    telemetry.addData("Dropping...", hanger.getHang().getCurrentPosition());
                }
                break;
            case STATE_EXTEND:
                hanger.drop();
                if (hanger.extend()){
                    newState(State.STATE_DELATCH);
                    drive.motor_reset();
                    telemetry.addData("Extended: ", hanger.getExtend().getCurrentPosition());
                }
                else{
                    telemetry.addData("Extending: ", hanger.getExtend().getCurrentPosition());
                }
                break;
            case STATE_DELATCH:
                hanger.drop();
                if (drive.goTo(new Pose2d(0, 1, 0), telemetry,0.25, 0.5)){
                    telemetry.addData("Moving to sample: ", pose.toString());
                    newState(State.STATE_DELATCH2);
                }
                else{
                    telemetry.addData("Attempting to delatch: ", pose.toString());
                }
                break;
            case STATE_DELATCH2:
                hanger.drop();
                if (drive.goTo(new Pose2d(5, 1, 0), telemetry,0.3, 2.5)){
                    telemetry.addData("Moving to sample: ", pose.toString());
                    newState(State.STATE_PREP);
                }
                else{
                    telemetry.addData("Attempting to delatch: ", pose.toString());
                }
                break;
            case STATE_PREP:
                if (hanger.contract() || mStateTime.time() >= 3.0){
                    /*if (drive.goTo(new Pose2d(0, 1, 0), telemetry, 0.3, 1.5)){
                        newState(State.STATE_SAMPLE);
                    }*/
                    if (pos != 2) {
                        newState(State.STATE_CENTER);
                    }
                    else{
                        x = 2;
                        newState(State.STATE_CENTER);
                    }
                }
                break;
            case STATE_SCAN:
                if (first) {
                    g.GoTo(0.11, 0.178);
                    first = true;
                }
                pos = getPos();
                List<Recognition> peek = tfod.getUpdatedRecognitions();
                if (peek != null) {
                    if (peek.size() > 3) {
                        g.move(0.0, 0.01);
                    }
                }
                telemetry.addData("Pos: ", pos);
                if (pos != -1){
                    newState(State.STATE_SCAN2);
                }
                else if (mStateTime.time() >= 10.0){
                    pos = 1;
                    newState(State.STATE_SCAN2);
                }
                break;
            case STATE_SCAN2:
                g.GoTo(0.09, 0.03);
                recog = vision_pulse();
                arm.move(0, 2, true);
                telemetry.addData("Arm Pos: ", arm.getArm().getCurrentPosition());
                if (recog.peek() != null) {
                    telemetry.addData("Target X: ", recog.peek().x());
                    telemetry.addData("Target Y: ", recog.peek().y());
                    chosen = recog.peek();
                }
                if (Math.abs(arm.getArm().getCurrentPosition() + 1450) < 50){
                    box_left.setPosition(0.925);
                    box_right.setPosition(0.075);
                    newState(State.STATE_DROP);
                }
                break;
            case STATE_SAMPLE:
                Pose2d dest;
                telemetry.addData("Sampling: ", pose.toString());
                double x_int = 0.0;
                switch (pos){
                    case 0:
                        x_int = -12 * Math.sqrt(2) - 3 + 2;
                        dest = new Pose2d(-12 * Math.sqrt(2) - 3 + 2, 12 * Math.sqrt(2) - 2 - 2, 0);
                        break;
                    case 1:
                        x_int = 0;
                        dest = new Pose2d(0, 12 * Math.sqrt(2) - 3 - 2, 0);
                        break;
                    case 2:
                        x_int = 12 * Math.sqrt(2) + 0 - 2;
                        dest = new Pose2d(12 * Math.sqrt(2) + 0 - 2, 12 * Math.sqrt(2) - 3 - 2, 0);
                        break;
                    default:
                        dest = new Pose2d(0, 0, 0);
                }
                if (Math.abs(drive.getPos().y() - x_int) < 10){
                    arm.move(-1.0, 2, true);
                }
                if (drive.goTo(dest, telemetry, 3.5)){// && !dest.equals(new Pose2d(0, 0, 0))){
                    //kill motors
                    arm.move(0.0, 1, true);
                    switch (pos) {
                        case 0:
                            //radius = 23;
                            newState(State.STATE_TRANSFER);
                            drive.disengage();
                            break;
                        case 1:
                            newState(State.STATE_TRANSFER);
                            drive.disengage();
                            break;
                        case 2:
                            drive.disengage();
                            newState(State.STATE_TRANSFER);
                            break;
                    }
                    //newState(State.STATE_SECURE);
                }
                break;
            case STATE_SECURE:
                Pose2d dest2;
                telemetry.addData("Sampling: ", pose.toString());
                switch (pos){
                    case 0:
                        dest2 = new Pose2d(-12 * Math.sqrt(2), 18 * Math.sqrt(2), 0);
                        break;
                    case 1:
                        dest2 = new Pose2d(0, 18 * Math.sqrt(2), 0);
                        break;
                    case 2:
                        dest2 = new Pose2d(14 * Math.sqrt(2), 18 * Math.sqrt(2), 0);
                        break;
                    default:
                        dest2 = new Pose2d(0, 0, 0);
                }
                if (drive.goTo(dest2, telemetry, 0.4, 4)){// && !dest.equals(new Pose2d(0, 0, 0))){
                    //kill motors
                    arm.move(0.0, 1, true);
                    switch (pos) {
                        case 0:
                            //radius = 23;
                            newState(State.STATE_TRANSFER);
                            break;
                        case 1:
                            newState(State.STATE_TRANSFER);
                            break;
                        case 2:
                            drive.disengage();
                            newState(State.STATE_STOP);
                            break;
                    }
                    newState(State.STATE_STOP);
                }
                break;
            case STATE_CHECKPOINT:
                if (drive.goTo(new Pose2d(0, 12 * Math.sqrt(2), 0), telemetry, 0.5, 3)){
                    drive.setPos(new Pose2d(0, 12 * Math.sqrt(2), 0));
                    newState(State.STATE_ALIGN);
                }
                break;
            case STATE_TURN:
                arm.move(0.0, 1, true);
                if (drive.turn(-45, 2, 0.3, telemetry)){
                    drive.setPos(new Pose2d(12 * Math.sqrt(2), -24 * Math.sqrt(2), 315));
                    //drive.engage();
                    offset = ((-offset + 24) * Math.tan(Math.toRadians(drive.getGyro().getHeading() - 315))) + offset;
                    drive.odoReset();
                    newState(State.STATE_TRAVEL);
                }
                break;
            case STATE_ALIGN:
                //arm.move(0.0, 1, true);
                telemetry.addData("Aligning", pose.toString());
                if (drive.goTo(new Pose2d(-24 * Math.sqrt(2), 12 * Math.sqrt(2), 315), telemetry, 7)){// && arm.getmTransferState() == 1){
                    memo = drive.getPos();
                    offset = ((-24 * Math.sqrt(2)) - memo.x());
                    offset /= Math.sqrt(2);
                    //offset = -((-offset + 24) * Math.tan(Math.toRadians(315 - drive.getGyro().getHeading()))) + offset;
                    //drive.disengage();
                    drive.engage();
                    newState(State.STATE_TURN);
                }
                break;
            case STATE_CENTER:
                //arm.move(0.0, 1, true);
                telemetry.addData("Centering: ", pose.toString());
                if (pos == 0){
                    x = -3.5;
                }
                if (drive.goTo(new Pose2d(x, 1, 0), telemetry, 0.4, 1.5)){
                    newState(State.STATE_SAMPLE);
                }
                break;
            case STATE_TRANSFER:
                arm.move(0.0, 1, true);
                telemetry.addData("Transferring: ", pose.toString());
                if (arm.getmTransferState() == 1){
                    newState(State.STATE_ALIGN);
                }
                break;
            case STATE_TRAVEL:
                telemetry.addData("Travelling (insert prayers): ", pose.toString());
                /*if (drive.goTo(new Pose2d((-48 * Math.sqrt(2)), 12 * Math.sqrt(2), 315), telemetry, 0.5, radius, 1)){
                    drive.motor_reset();
                    if (pos != 0){
                        heading = -50;
                        newState(State.STATE_TURN2);
                    }
                    else {
                        newState(State.STATE_TURN2);
                    }
                }*/
                drive.drive(0.5, Math.PI, 0.0, 0.0);
                if (mStateTime.time() >= 2.0){
                    drive.drive(0.0, Math.PI, 0.0, 0.0);
                    drive.odoReset();
                    newState(State.STATE_TRAVEL2);
                }
                break;
            case STATE_TURN2:
                telemetry.addData("Turning Again: ", drive.getRobotHeading());
                if (drive.turn(-59, 2, 0.2, telemetry)){
                    //drive.setPos(new Pose2d(12 * Math.sqrt(2), -24 * Math.sqrt(2), 315));
                    //drive.engage();
                    newState(State.STATE_TRAVEL2);
                }
                break;
            case STATE_TRAVEL2:
                telemetry.addData("Travelling 2: ", drive.getOdoDistance());
                /*if (drive.goTo(new Pose2d(-48 * Math.sqrt(2), 0 * Math.sqrt(2), 315), telemetry, 0.3,15, 1)){
                    newState(State.STATE_STOP);
                }*/
                arm.move(0.0, 3, true);
                double new_pos = -1;
                double dist = -(pos == 0 ? 42 : 42) + (12 * (pos == 0 ? -0.5 : pos)) + offset - (pos == 0 ? 9 : 7);
                dist = -45 + (12 * (pos));// + offset;
                telemetry.addData("Dist: ", dist);
                telemetry.addData("Offset: ", offset);
                if (drive.getOdoDistance() <= (dist - 6)){
                    drive.drive(0.0, 0.0, 0.0, 0.0);
                    //marker.setPosition(0.3);
                    //box_left.setPosition(0.0);
                    //box_right.setPosition(1.0);
                    //drive.odoReset();
                    wall.engage();
                    newState(State.STATE_SAMPLE2);
                }
                else{
                    drive.drive(0.5, Math.toRadians(268), 0, 0);
                    mStateTime.reset();
                }
                break;
            case STATE_SAMPLE2:
                telemetry.addData("Tracker Pos: ", drive.getOdoDistance());
                drive.drive(1.0, 0, 0.0, 0.0);
                double time = 1 - ((pos - 1) * 0.5) + 0.25;
                if (mStateTime.time() >= time){
                    drive.drive(0.0, Math.PI, 0.0, 0.0);
                    newState(State.STATE_CENTER2);
                }
                break;
            case STATE_CENTER2:
                telemetry.addData("Tracker Pos: ", drive.getOdoDistance());
                drive.drive(1.0, Math.PI, 0.0, 0.0);
                time = 1 - ((pos - 1) * 0.5) + 0.25;
                if (mStateTime.time() >= time){
                    drive.drive(0.0, Math.PI, 0.0, 0.0);
                    wall.disengage();
                    newState(State.STATE_TRAVEL3);
                }
                break;
            case STATE_TRAVEL3:
                double prev_dist = drive.getOdoDistance();
                double target = -39;
                double direction = Range.clip(target - prev_dist, -0.7, 0.7);
                telemetry.addData("Travelling 3: ", drive.getOdoDistance());
                if (drive.getOdoDistance() <= target){
                    drive.drive(0.0, 0.0, 0.0, 0.0);
                    marker.setPosition(0.3);
                    box_left.setPosition(0.0); //todo: ADD THE SERVO FLIP
                    box_right.setPosition(1.0);
                    if (mStateTime.time() >= 0.0) {
                        //drive.odoReset();
                        newState(State.STATE_TURN3);
                    }
                }
                else{
                    drive.drive(-direction, Math.toRadians(268), 0, 0);
                    mStateTime.reset();
                }
                break;
            case STATE_TURN3:
                telemetry.addData("Turning Again Again (that's three turns now monkaS): ", drive.getRobotHeading());
                if (mStateTime.time() >= 0.25) {
                    if (drive.turn(-44, 2, 0.15, telemetry)) {
                        //drive.setPos(new Pose2d(12 * Math.sqrt(2), -24 * Math.sqrt(2), 315));
                        //drive.engage();
                        drive.odoReset();
                        if (mStateTime.time() >= 0.0) {
                            newState(State.STATE_RETURN);
                        }
                    }
                }
                break;
            case STATE_RETURN:
                telemetry.addData("Returning: ", drive.getOdoDistance());
                drive.drive(1.0, Math.toRadians(92), 0, 0);
                arm.move(0.0, 3, false);
                double distance = (-(-45 + (12 * pos))) + 15;
                if (drive.getOdoDistance() >= distance) {
                    drive.drive(0.0, 0.0, 0.0, 0.0);
                    drive.motor_reset();
                    newState(State.STATE_STOP);
                }
                break;
            case STATE_STOP:
                telemetry.addData("Ta-Da (insert jazz hands)", pose.toString());
                telemetry.addData("Pos: ", pos);
                telemetry.addData("X: ", chosen.x());
                telemetry.addData("Y: ", chosen.y());
                arm.move(1.0, 2, false);
                if (mStateTime.time() >= 0.0) {
                    box_left.setPosition(0.9);
                    box_right.setPosition(0.1);
                }
                if (mStateTime.time() >= 1.5){
                    arm.move(0.0, 2, false);
                }
                drive.drive(0.0, 0.0, 0.0, 0.0);
                break;
        }
        //if (mRunTime.time() >= 0.5) {
        //pose = drive.track();
        //mRunTime.reset();
        //}
    }

    @Override
    public void stop(){
        tfod.shutdown();
    }

    public void newState(State s){
        mRobotState = s;
        mStateTime.reset();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private int getPos(){
        int pos = -1;
        if (tfod != null) {
            telemetry.addData("Not Null: ", tfod.toString());
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("Recognition Not Null: ", updatedRecognitions.toString());
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            pos = 0;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            pos = 2;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            pos = 1;
                        }
                    }
                    telemetry.addData("X: ", g.getPos().x());
                    telemetry.addData("Y: ", g.getPos().y());
                    //g.GoTo(68.82 / 90.0, -56.87 / 90.0);
                }
                telemetry.update();
            }
        }
        return pos;
    }

    private PriorityQueue<Pose2d> vision_pulse(){
        PriorityQueue<Pose2d> recog = new PriorityQueue<Pose2d>(1, new PoseComparator());
        if (tfod != null) {
            //telemetry.addData("Not Null: ", tfod.toString());
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("Recognition Not Null: ", updatedRecognitions.toString());
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions){
                    double y = 0.0;
                    double x = 0.0;
                    y = (((recognition.getTop() + recognition.getBottom()) / 2)) / recognition.getImageHeight();
                    y *= 43.30;
                    y += g.getPos().y() * 90;
                    y += 10;
                    y = 90 - y;
                    //y /= 90.0;
                    //y *= -1;
                    y = Math.toRadians(y);
                    x = (((recognition.getLeft() + recognition.getRight()) / 2)) / recognition.getImageWidth();
                    x *= 70.42;
                    x += g.getPos().x() * 90;
                    x = 90 - x;
                    //x /= 90.0;
                    x = Math.toRadians(x);
                    double c = height * Math.tan(y);
                    double dx = c * Math.cos(x) - 7.5;
                    double dy = c * Math.sin(x) + 8.0;
                    recog.add(new Pose2d(dx, dy, 0));
                }
            }
        }
        return recog;
    }
}
