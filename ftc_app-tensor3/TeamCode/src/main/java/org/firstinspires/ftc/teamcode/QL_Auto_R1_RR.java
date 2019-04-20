package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.movement.DriveConstants;
import org.firstinspires.ftc.teamcode.movement.Runner_Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Slide;

public class QL_Auto_R1_RR extends OpMode {
    Runner_Mecanum_Drive drive;
    Arm arm;
    Slide slide;
    Servo tm;
    Trajectory path;

    int pos = -1;
    boolean first = true;

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

    Gimbel g;

    State mRobotState = State.STATE_START;
    ElapsedTime mStateTime = new ElapsedTime();

    private enum State{
        STATE_START,
        STATE_SCAN,
        STATE_SCAN2,
        STATE_DROP,
        STATE_EXTEND,
        STATE_DELATCH,
        STATE_DELATCH2,
        STATE_CONTRACT,
        STATE_CENTER,
        STATE_SPLINE_TO_SAMPLE,
        STATE_INTAKE,
        STATE_TRANSFER,
        STATE_SPLINE_TO_DEPOT,
        STATE_PARK,
        STATE_STOP
    }


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    Pose2d memo = new Pose2d();

    public void init(){
        drive = new Runner_Mecanum_Drive();
        drive.setDrive(hardwareMap);
        drive.setSmoothing(DriveConstants.smoothing);
        drive.disengage();
        arm = new Arm(hardwareMap);
        slide = new Slide(hardwareMap);
        slide.getBox().compress();
        g = new Gimbel(hardwareMap);
        tm = hardwareMap.get(Servo.class, "tm");
        tm.setPosition(0.6);
        slide.getHanger().getHang().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.getHanger().getHang().setTargetPosition(slide.getHanger().getHang().getCurrentPosition());
        slide.getHanger().getHang().setPower(1.0);
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        g.GoTo(-0.5, -0.19);
    }

    public void loop(){
        switch (mRobotState) {
            case STATE_START:
                telemetry.addData("Entering Auto", "1");
                tfod.activate();
                newState(State.STATE_SCAN);
                break;
            case STATE_SCAN:
                if (first) {
                    g.GoTo(0.11, 0.178);
                    first = false;
                }
                pos = getPos();
                telemetry.addData("Pos: ", pos);
                if (pos != -1) {
                    newState(State.STATE_SCAN2);
                } else if (mStateTime.time() >= 10.0) {
                    pos = 1;
                    newState(State.STATE_SCAN2);
                }
                break;
            case STATE_SCAN2:
                g.GoTo(-0.5, 0);
                arm.setExit(true);
                arm.move(0.0, 4, false);

                telemetry.addData("Arm Pos: ", arm.getArm().getCurrentPosition());
                telemetry.addData("Arm Target: ", arm.getArm().getTargetPosition());
                if (arm.getArm().getCurrentPosition() < -600) {
                    arm.setExit(false);
                    slide.getBox().e_init();
                    slide.getHanger().getHang().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slide.getHanger().getHang().setPower(0.0);
                    newState(State.STATE_DROP);
                }
                break;
            case STATE_DROP:
                slide.getHanger().drop();
                telemetry.addData("Hang Pos: ", slide.getHanger().getHang().getCurrentPosition());
                if (slide.getHanger().getHang().getCurrentPosition() < -5400) {
                    //drive.engage();
                    drive.reset();
                    newState(State.STATE_EXTEND);
                }
                break;
            case STATE_EXTEND:
                slide.getHanger().drop();
                if (slide.getHanger().extend() || mStateTime.time() >= 0.5) {
                    slide.getHanger().extend_stop();
                    newState(State.STATE_DELATCH2);
                    path = drive.trajectoryBuilder()
                            .turnTo(Math.toRadians(30))
                            .build();
                    drive.followTrajectory(path);
                    drive.reset();
                    telemetry.addData("Extended: ", slide.getHanger().getExtend().getCurrentPosition());
                } else {
                    telemetry.addData("Extending: ", slide.getHanger().getExtend().getCurrentPosition());
                }
                break;
            case STATE_DELATCH:
                slide.getHanger().drop();
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                /*if (drive.goTo(new Pose2d(0, 0.5, 0), 0.4, 1)) {
                    newState(State.STATE_DELATCH2);
                }*/
                if (drive.isFollowingTrajectory()) {
                    drive.updateFollower();
                } else {
                    drive.drive(0.0, 0.0, 0.0, 0.0);
                    newState(State.STATE_CENTER);
                }
                break;
            case STATE_DELATCH2:
                slide.getHanger().drop();
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                if (drive.isFollowingTrajectory()) {
                    drive.updateFollower();
                } else {
                    drive.drive(0.0, 0.0, 0.0, 0.0);
                    slide.getHanger().contract();
                    newState(State.STATE_CONTRACT);
                }
                break;
            case STATE_CONTRACT:
                drive.stop();
                if (slide.getHanger().contract()) {
                    drive.stop();
                    path = drive.trajectoryBuilder().turnTo(Math.toRadians(0)).build();
                    drive.followTrajectory(path);
                    newState(State.STATE_CENTER);
                    first = true;
                }
                break;
            case STATE_CENTER:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                telemetry.addData("Vertical Pos: ", slide.getHanger().getExtend().getCurrentPosition());
                //slide.getHanger().contract();
                if (drive.isFollowingTrajectory()) {
                    new Runnable() {
                        @Override
                        public void run() {
                            drive.updateFollower();
                        }
                    }.run();
                } else {
                    if (first) {
                        mStateTime.reset();
                        first = false;
                    }
                    drive.engage();
                    drive.reset();
                    drive.stop();
                    if (pos != 1) {
                        path = drive.trajectoryBuilder()
                                .splineTo(new Pose2d(24 * Math.sqrt(2), (1 - pos) * 12 * Math.sqrt(2), Math.toRadians((pos - 1) * 30)))
                                .build();
                    }
                    else{
                        path = drive.trajectoryBuilder()
                                .forward(24 * Math.sqrt(2))
                                .build();
                    }
                    //drive.followTrajectory(path);
                    drive.reset();
                    if (mStateTime.time() >= 0.5) {
                        drive.reset();
                        drive.followTrajectory(path);
                        newState(State.STATE_SPLINE_TO_SAMPLE);
                    }
                }
                break;
            case STATE_SPLINE_TO_SAMPLE:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                telemetry.addData("Error :", drive.getFollowingError());
                if (drive.isFollowingTrajectory()){
                    drive.updateFollower();
                }
                else{
                    drive.stop();
                    first = true;
                    arm.move(1.0, 2, true);
                    newState(State.STATE_INTAKE);
                }
                break;
            case STATE_INTAKE:
                telemetry.addData("Slide Pos: ", arm.getSweeper().getCurrentPosition());
                if (arm.extend()){
                    if (first){
                        mStateTime.reset();
                        first = false;
                    }
                    if (mStateTime.time() >= 1.0){
                        arm.move(-1.0, 1, true);
                        newState(State.STATE_TRANSFER);
                    }
                }
                else{
                    arm.move(1.0, 2, true);
                }
                break;
            case STATE_TRANSFER:
                telemetry.addData("Arm Pos: ", arm.getArm().getCurrentPosition());
                if (arm.getArm().getCurrentPosition() < -550 && arm.getSweeper().getCurrentPosition() < 200){
                    arm.move(-1.0, 1, true);
                }
                else{
                    arm.move(0.0, 4, true);
                    path = drive.trajectoryBuilder()
                            .back(12 * Math.sqrt(2))
                            .turnTo(Math.toRadians(-90))
                            .splineTo(new Pose2d(0, 48 * Math.sqrt(2), Math.toRadians(45)))
                            .build();
                    drive.followTrajectory(path);
                    newState(State.STATE_SPLINE_TO_DEPOT);
                }
                break;
            case STATE_SPLINE_TO_DEPOT:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                telemetry.addData("Error: ", drive.getFollowingError());
                if (drive.isFollowingTrajectory()){
                    drive.updateFollower();
                }
                else{
                    drive.stop();
                    tm.setPosition(0.0);
                    path = drive.trajectoryBuilder()
                            .forward(72)
                            .build();
                    first = true;
                    newState(State.STATE_PARK);
                }
                break;
            case STATE_PARK:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                telemetry.addData("Error: ", drive.getFollowingError());
                if (mStateTime.time() >= 1.0){
                    if (first){
                        drive.followTrajectory(path);
                        first = false;
                    }
                    if (drive.isFollowingTrajectory()){
                        drive.updateFollower();
                    }
                    else{
                        drive.stop();
                        arm.move(0.0, 2, true);
                        newState(State.STATE_STOP);
                    }
                }
                break;
            case STATE_STOP:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                telemetry.addData("Arm Pos: ", arm.getArm().getCurrentPosition());
                if (arm.getArm().getCurrentPosition() < -1300 && arm.extend()){
                    arm.getArm().setPower(0.0);
                }
                else{
                    arm.move(0.0, 2, true);
                }
                drive.stop();
                drive.disengage();
                break;
        }
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
}
