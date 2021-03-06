package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.movement.DriveConstants;
import org.firstinspires.ftc.teamcode.movement.Runner_Mecanum_Drive;
import org.firstinspires.ftc.teamcode.wrapper.Arm;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;
import org.firstinspires.ftc.teamcode.wrapper.Hanger;
import org.firstinspires.ftc.teamcode.wrapper.Slide;

import java.util.List;
import java.util.PriorityQueue;

@Autonomous(name = "QL_Auto_R2_RR", group = "Competition RR")
public class QL_Auto_R2_RR extends OpMode {
    Runner_Mecanum_Drive drive;
    Arm arm;
    Slide slide;
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


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public enum State{
        STATE_START,
        STATE_SCAN,
        STATE_SCAN2,
        STATE_DROP,
        STATE_EXTEND,
        STATE_DELATCH,
        STATE_DELATCH2,
        STATE_CONTRACT,
        STATE_CENTER,
        STATE_DRIVE_TO_DEPLOY,
        STATE_DEPLOY,
        STATE_RETRACT,
        STATE_TURN_TO_SAMPLE,
        STATE_INTAKE_SAMPLE,
        STATE_RETURN_TO_LANDER,
        STATE_DUMP,
        STATE_IDLE,
        STATE_STOP
    }

    public void init(){
        drive = new Runner_Mecanum_Drive();
        drive.setDrive(hardwareMap);
        drive.setSmoothing(DriveConstants.smoothing);
        drive.disengage();
        arm = new Arm(hardwareMap);
        slide = new Slide(hardwareMap);
        slide.getBox().compress();
        g = new Gimbel(hardwareMap);
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
        switch (mRobotState){
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
                if (pos != -1){
                    newState(State.STATE_SCAN2);
                }
                else if (mStateTime.time() >= 10.0){
                    pos = 1;
                    newState(State.STATE_SCAN2);
                }
                break;
            case STATE_SCAN2:
                g.GoTo(-0.5, 0);
                slide.getBox().e_init();
                arm.setExit(true);
                arm.move(0.0, 4, false);

                telemetry.addData("Arm Pos: ", arm.getArm().getCurrentPosition());
                telemetry.addData("Arm Target: ", arm.getArm().getTargetPosition());
                if (arm.getArm().getCurrentPosition() < -600){
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
                if (slide.getHanger().getHang().getCurrentPosition() < -5400){
                    drive.engage();
                    drive.reset();
                    newState(State.STATE_EXTEND);
                }
                break;
            case STATE_EXTEND:
                slide.getHanger().drop();
                if (slide.getHanger().extend()){
                    newState(State.STATE_IDLE);
                    drive.reset();
                    telemetry.addData("Extended: ", slide.getHanger().getExtend().getCurrentPosition());
                }
                else{
                    telemetry.addData("Extending: ", slide.getHanger().getExtend().getCurrentPosition());
                }
                break;
            case STATE_IDLE:
                if (mStateTime.time() >= 1.0){
                    drive.reset();
                    newState(State.STATE_DELATCH2);
                }
                break;
            case STATE_DELATCH:
                slide.getHanger().drop();
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                if (drive.goTo(new Pose2d(0, 0.5, 0), 0.4, 1)) {
                    newState(State.STATE_DELATCH2);
                }
                break;
            case STATE_DELATCH2:
                slide.getHanger().drop();
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                if (drive.goTo(new Pose2d(2, 0, 0), 0.2, 2)) {
                    slide.getHanger().contract();
                    newState(State.STATE_CONTRACT);
                }
                break;
            case STATE_CONTRACT:
                if (slide.getHanger().contract()){
                    newState(State.STATE_CENTER);
                }
                break;
            case STATE_CENTER:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                //slide.getHanger().contract();
                if (drive.goTo(new Pose2d(0, 0, 0), 0.2, 2)) {
                    path = drive.trajectoryBuilder()
                            .forward(12 * Math.sqrt(2))
                            .build();
                    drive.followTrajectory(path);
                    newState(State.STATE_DRIVE_TO_DEPLOY);
                }
                break;
            case STATE_DRIVE_TO_DEPLOY:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                telemetry.addData("Extend Pos: ", arm.getSweeper().getCurrentPosition());
                slide.getBox().e_init();
                if (drive.isFollowingTrajectory()) {
                    drive.updateFollower();
                    arm.extend();
                    arm.move(0.0, 3, false);
                }
                else{
                    path = drive.trajectoryBuilder()
                            .back(6 * Math.sqrt(2))
                            .turnTo((1 - pos) * 45) //todo: tune the heading you turn to for samples :)
                            .build();
                    //drive.followTrajectory(path);
                    newState(State.STATE_DEPLOY);
                }
                break;
            case STATE_DEPLOY:
                if (mStateTime.time() >= 2.0){
                    arm.move(-1.0, 1, false);
                    drive.followTrajectory(path);
                    newState(State.STATE_RETRACT);
                }
                else{
                    arm.move(-1.0, 3, false);
                }
                break;
            case STATE_RETRACT:
                telemetry.addData("Arm Pos: ", arm.getSweeper().getCurrentPosition());
                telemetry.addData("Drive Pos: ", drive.getEstimatedPose());
                if (drive.isFollowingTrajectory()){
                    arm.move(-1.0, 1, false);
                    drive.updateFollower();
                }
                else{
                    path = drive.trajectoryBuilder()
                            .turnTo(0)
                            .back(3 * Math.sqrt(2))
                            .build();
                    arm.move(1.0, 2, false);
                    newState(State.STATE_INTAKE_SAMPLE);
                }
                break;
            case STATE_INTAKE_SAMPLE:
                telemetry.addData("Slide Pos: ", arm.getSweeper().getCurrentPosition());
                if (arm.extend() && mStateTime.time() >= 2.0){
                    arm.move(1.0, 1, false);
                    drive.followTrajectory(path);
                    first = true;
                    newState(State.STATE_RETURN_TO_LANDER);
                }
                else{
                    arm.move(1.0, 1, false);
                }
                break;
            case STATE_RETURN_TO_LANDER:
                telemetry.addData("Pos: ", drive.getEstimatedPose());
                telemetry.addData("Slide Pos: ", arm.getSweeper().getCurrentPosition());
                if (drive.isFollowingTrajectory()){
                    if (first) {
                        arm.move(1.0, 1, false);
                        first = false;
                    }
                    drive.updateFollower();
                }
                else{
                    newState(State.STATE_DUMP);
                }
                break;
            case STATE_DUMP:
                if (slide.dump(false, true, telemetry)){
                    newState(State.STATE_STOP);
                }
                break;
            case STATE_STOP:
                telemetry.addData("FIRST TRY!!!!", drive.getEstimatedPose());
                tfod.deactivate();
                break;
        }
    }

    public void stop(){
        tfod.shutdown();
    }

    public void newState(State arg){
        mRobotState = arg;
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
}
