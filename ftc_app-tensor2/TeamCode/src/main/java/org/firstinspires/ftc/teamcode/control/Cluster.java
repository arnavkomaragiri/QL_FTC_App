package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.wrapper.Gimbel;

public class Cluster {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    Gimbel g;
    boolean pulse = false;
    double x = -1.0;
    double y = -1.0;

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
}
