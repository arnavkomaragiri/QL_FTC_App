package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanger {
    DcMotor hang;
    DcMotor extend;

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
        hang.setPower(g.right_stick_y);
        extend.setPower(g.left_stick_y);
    }
}
