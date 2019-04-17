package org.firstinspires.ftc.teamcode.movement;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//@Config
public class Tracker_Mecanum_Drive extends Root_Mecanum_Drive{
    Two_Axis_Localizer pos;
    private double[] prevPowers = {-2.0, -2.0, -2.0, -2.0};

    public static final int MOTOR_WRITE_DELAY = 10;

    public Tracker_Mecanum_Drive(HardwareMap h) {
        super(h);
        calibrate();
        pos = new Two_Axis_Localizer(h);
    }

    public void engage(){
        pos.engage();
    }

    public void disengage(){
        pos.disengage();
    }

    public Two_Axis_Localizer getPositioner(){
        return pos;
    }

    public void setPowers(double v, double v1, double v2, double v3){
        DcMotor[] motors = super.getMotors();
        double[] powers = {v, v3, v1, v2};

        for (int i = 0; i < 4; i++){
            motors[i].setPower(powers[i]);
        }
        //prevPowers = powers;

        try{
            Thread.sleep(MOTOR_WRITE_DELAY);
        }
        catch (InterruptedException e){
            //I would do something here, but it wouldn't really matter so...
        }
    }
}
