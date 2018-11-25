package org.firstinspires.ftc.teamcode.wrapper.sensors;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class QL_Encoder {
    private AnalogInput odo;
    private double previous = 0.0;
    private double rot = 0.0;

    public QL_Encoder(HardwareMap h){
        odo = h.analogInput.get("odometer");
        previous = odo.getVoltage() * 72;
    }

    public QL_Encoder(AnalogInput a){
        odo = a;
        previous = odo.getVoltage() * 72;
    }

    private double getDistance(double voltage){
        double heading = voltage * 72;
        if ((heading - previous) < -270){
            rot += (360 + (heading - previous)) / 360;
        } else if((heading - previous) > 270){
            rot -= (360 + (previous - heading)) / 360;
        }
        else{
            rot += (heading - previous) / 360;
        }
        previous = heading;
        return (6 * Math.PI * rot);
    }

    public double getDistance(){
        return getDistance(odo.getVoltage());
    }

    public void reset(){
        previous = odo.getVoltage() * 72;
    }
}
