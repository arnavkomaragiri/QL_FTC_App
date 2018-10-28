package org.firstinspires.ftc.teamcode.wrapper;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Sentry implements UVCCamera.Callback{
    Gimbel g;
    UVCCamera camera;
    Bitmap buffer;
    static String load = "";

    static {
        if (!OpenCVLoader.initDebug()) {
            load = "Error Loading!";
        } else {
            load = "Loaded Successfully!";
        }
    }

    public Sentry(HardwareMap h){
        g = new Gimbel(h);
        camera = UVCCamera.getCamera(this);
    }

    public int getPosition(Telemetry telemetry){
        Mat input = new Mat();
        Mat hsv = new Mat();
        double posX = 0.0;
        double locationX = 0.0;
        double size = 0.0;
        int sampPos = -1;

        Bitmap bmp32 = buffer.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, input);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(25, 100, 100), new Scalar(35, 255, 255), mask);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0;
        List<MatOfPoint> biggest = new ArrayList<>();
        int index = -1;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea) {
                maxArea = area;
                List<MatOfPoint> current = new ArrayList<>();
                current.add(wrapper);
                biggest = current;
            }
        }

        //Centroid setup
        Moments mmnts = Imgproc.moments(mask, true);
        locationX = mmnts.get_m10() / mmnts.get_m00();
        posX = (input.width() / 2) - locationX;
        size = maxArea;

        telemetry.addData("Location", locationX);
        telemetry.addData("Position", posX);
        telemetry.addData("Size", size);

        if(posX < -100){
            sampPos = 2;
        }else if(posX > -100 && posX < 100){
            sampPos = 1;
        }else if(posX > 100){
            sampPos = 0;
        }

        return sampPos;
    }

    public boolean lockOn(){
        Mat input = new Mat();
        Mat hsv = new Mat();
        double posX = 0.0;
        double locationX = 0.0;
        double locationY = 0.0;
        double size = 0.0;
        int sampPos = -1;
        boolean outcome = false;

        Bitmap bmp32 = buffer.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, input);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(100, 100, 0), new Scalar(255, 255, 15), mask);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0;
        List<MatOfPoint> biggest = new ArrayList<>();
        int index = -1;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea) {
                maxArea = area;
                List<MatOfPoint> current = new ArrayList<>();
                current.add(wrapper);
                biggest = current;
            }
        }

        //Centroid setup
        Moments mmnts = Imgproc.moments(mask, true);
        locationX = mmnts.get_m10() / mmnts.get_m00();
        locationY = mmnts.get_m01() / mmnts.get_m00();
        posX = (input.width() / 2) - locationX;
        locationY = (input.height() / 2) - locationY;
        size = maxArea;

        //gimbal logic

        if (locationY < 100){
            g.move(0, 0.05);
        }
        else if (locationY >= 100){
            outcome = true;
        }
        return outcome;
    }

    @Override
    public Bitmap onFrame(Bitmap bm){
        buffer = bm;
        return bm;
    }

    public void start(){
        camera.start();
    }

    public void stop(){
        camera.stop();
    }
}
