package org.firstinspires.ftc.teamcode.wrapper;

import android.graphics.ColorFilter;

import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.ViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigationWebcam;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;

public class Test_Webcam extends OpMode {

    Dogeforia vuforia;
    private WebcamName webcamName;
    public DogeCVColorFilter ColorFilter;
    public SamplingOrderDetector d;

    public void init(){

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");     //Initializing hardware map for cam

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);    //setting parameters from parameter class of VuforiaLocalizer

        //This code refers to sample ConceptVumarkIdentificationWebcam.java

        //Licence Key from registration of VuForia
        parameters.vuforiaLicenseKey = "AR0LYP3/////AAABmQ7b/5H8u0A7uZvKeYoEg0JVsyhd/1+GlaxSV0Pw+lm+4zOHSxXgvk04VjLKBIO1iwO6StAmpOwSGYC4RYIwg2IwnKVVmKbn1XD11+qF/sYPz4xu+tvrexORZ8575m5Blh5GP+EU61ej86TfAGDVViwicrIvS/iUDncuiCE7BY7oogBfYcum7fCG5JIX41+NSMhrwXdTNxXTTPVF6tlS15g9zJE2ds1I9+OmUwITV8iK6udThWEE/beK7xRPdjzwf1o75C+WTlbSMUDvk2W7rNVqfqWNrxQFqenn29mOH1qAGfVL2J4j9Vfq/ZQ3lyDaBaNqoNUduTn2hEyO8oF1VxTUz1hoaddrz04STWGBiBon";


        parameters.cameraName = webcamName;    //Setting the webcam name to the parameters class
        this.vuforia = new Dogeforia(parameters);
        vuforia.setDogeCVDetector(d);
    }

    public void loop(){


    }
}
