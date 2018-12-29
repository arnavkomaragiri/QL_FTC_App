package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCUtils;

public class Hanger {
    private DcMotor hang;
    private DcMotor extend;

    private ElapsedTime cooldown = new ElapsedTime();
    private ElapsedTime cooldown2 = new ElapsedTime();
    private ElapsedTime jam = new ElapsedTime();
    private double previous = 0.0;
    private boolean complete = true;
    private boolean first = true;

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

    public void setExtend(DcMotor extend){
        this.extend = extend;
    }

    public DcMotor getExtend(){
        return this.extend;
    }

    public boolean drop(){
        int pos = 10000;
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(pos);
        hang.setPower(1.0);

        if (Math.abs(hang.getCurrentPosition() - pos) < 10){
            hang.setTargetPosition(hang.getCurrentPosition());
            hang.setPower(0.0);
            hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }
        else{
            return false;
        }
    }
    public boolean contract(){
        int pos = 0;
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(pos);
        extend.setPower(1.0);

        if (Math.abs(extend.getCurrentPosition() - pos) < 10){
            extend.setTargetPosition(extend.getCurrentPosition());
            extend.setPower(0.0);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }
        else{
            return false;
        }
    }

    public boolean extend(){
        boolean result = false;

        if (first){
            jam.reset();
            first = false;
        }
        if (Math.abs(extend.getCurrentPosition() - 850) <= 50 || jam.time() >= 2.0){
            complete = true;
            //if (cooldown2.time() >= 0.125) {
                //extend.setTargetPosition(hang.getCurrentPosition());
                extend.setPower(0.0);
                //extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hangstate = false;
                cooldown.reset();
                result = true;
                first = true;
            //}
        }
        else{
            //extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //extend.setTargetPosition(750);
            extend.setPower(1.0);
            hangstate = true;
            complete = false;
        }
        previous = extend.getCurrentPosition();
        return result;
    }

    public void operate(Gamepad g){
        if (g.dpad_up){
            hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hang.setTargetPosition(7300);
            hang.setPower(1.0);
            hangstate = true;
            complete = false;
        }

        if (Math.abs(hang.getCurrentPosition() - 7300) < 10){
            complete = true;
            if (cooldown2.time() >= 1.0) {
                hang.setTargetPosition(hang.getCurrentPosition());
                hang.setPower(0.0);
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hangstate = false;
                cooldown.reset();
            }
        }

        if (!complete){
            cooldown2.reset();
        }
        if (!hangstate) {
            hang.setPower(g.left_stick_y * -1);
        }
        //if (!hangstate) {
            /*if (extend.getCurrentPosition() >= 0.0) {
                extend.setPower(g.right_stick_y);
            }
            else{
                extend.setPower(1.0);
            }*/
        //}
        //}
        //else{
            extend.setPower(g.right_stick_y);
        //}
    }
}
