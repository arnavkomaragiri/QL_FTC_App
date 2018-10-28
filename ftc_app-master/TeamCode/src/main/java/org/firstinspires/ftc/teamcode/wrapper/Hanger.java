package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanger {
    private DcMotor hang;
    private DcMotor extend;

    private boolean hangstate = false;

    public Hanger(HardwareMap h){
        hang = h.get(DcMotor.class, "hang");
        extend = h.get(DcMotor.class,"extend");
    }

    public void setHang(DcMotor hang) {
        this.hang = hang;
    }

    public DcMotor getHang() {
        return hang;
    }

    public void operate(Gamepad g){
        if (g.x){
            if (hangstate){
                hangstate = false;
            }
            else{
                hangstate = true;
            }
        }
        if (!hangstate){
            extend.setPower(g.right_stick_y);
        }
        else{
            hang.setPower(g.right_stick_y);
        }
    }
}
