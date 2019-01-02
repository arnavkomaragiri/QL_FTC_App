package org.firstinspires.ftc.teamcode;

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

import java.util.List;
import java.util.PriorityQueue;

public class LIVE extends OpMode {
    DcMotor motors[] = new DcMotor[4];
    DcMotor arm1;
    DcMotor sweeper;
    Mecanum_Drive drive;
    Hanger hanger;
    Arm arm;
    double radius = 24.5;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    Gimbel g;
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
    boolean flip = false;
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime precision = new ElapsedTime();
    ElapsedTime mRunTime = new ElapsedTime();
    Pose2d pose = new Pose2d(0, 0, 0);
    Pose2d chosen = new Pose2d(-1, -1, 0);
    int pos = -1;
    double height = 25.0;

    QL_Auto_R1.State mRobotState = QL_Auto_R1.State.STATE_START;
    ElapsedTime mStateTime = new ElapsedTime();
    PriorityQueue<Pose2d> recog;
    Pose2d memo = new Pose2d(0, 0, 0);

    public void init(){
        hanger = new Hanger(hardwareMap);

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        box_left = hardwareMap.get(Servo.class, "box_left");
        box_right = hardwareMap.get(Servo.class, "box_right");
        marker = hardwareMap.get(Servo.class, "tm");
        marker.setPosition(0.8);
        box_left.setPosition(1.0);
        box_right.setPosition(0.0);

        arm = new Arm(sweeper, arm1, hardwareMap.get(Servo.class, "back"));

        g = new Gimbel(hardwareMap);
        g.GoTo(-0.5, -0.19);

        drive = new Mecanum_Drive(hardwareMap);

        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public void loop(){
        drive.goTo(cluster(), telemetry);
    }

    public PriorityQueue<Pose2d> vision_pulse(){
        PriorityQueue<Pose2d> recog = new PriorityQueue<Pose2d>(1, new LIVE().State());
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

    private Pose2d cluster(){
        Pose2d clusters;
        recog = vision_pulse();
        double mu = 1;
        double sigma = 1;
        double x1 = 2;
        //1/2pisigma * e^(-((x-mu)^2)/(2sigma^2)))
        for(int i = 0; i <= 100; i++){
            double X = recog.peek().x();
            double Y = recog.peek().y();
            double sqr1 = Math.pow(sigma, 2);
            double sqr2 = x1 - mu;
            double sqr3 =Math.pow(2, x1 - mu);
            Pose2d cluster_X = 1/2 * sigma * Math.PI * Math.E ^(-(sqr3/(2*sqr1));
            clusters = cluster_X;
        }
        return clusters;
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
        int tfodMonitorViewId = hardwareMap .appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}


