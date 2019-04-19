package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCUtils;
import org.firstinspires.ftc.teamcode.movement.Two_Axis_Localizer;

public class Hanger {
    private DcMotor hang;
    private DcMotor extend;
    private Two_Axis_Localizer localizer;

    private ElapsedTime cooldown = new ElapsedTime();
    private ElapsedTime cooldown2 = new ElapsedTime();
    private ElapsedTime jam = new ElapsedTime();
    private double previous = 0.0;
    private boolean complete = true;
    private boolean first = true;

    private boolean hangstate = false;
    private ElapsedTime matchTime = new ElapsedTime();

    private State mDiagnosticState = State.STATE_EXTEND;
    private ElapsedTime mStateTime = new ElapsedTime();

    private enum State{
        STATE_EXTEND,
        STATE_CONTRACT,
        STATE_STOP
    }

    private void newState(State s){
        mDiagnosticState = s;
        mStateTime.reset();
    }

    public boolean diagnose(){
        boolean result = false;
        switch (mDiagnosticState){
            case STATE_EXTEND:
                if (cycle()){
                    newState(State.STATE_CONTRACT);
                }
                break;
            case STATE_CONTRACT:
                if (contract()) {
                    result = true;
                    newState(State.STATE_STOP);
                }
                break;
        }
        return result;
    }

    public Hanger(HardwareMap h){
        hang = h.get(DcMotor.class, "hang");
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(hang.getCurrentPosition());
        hang.setPower(1.0);
        extend = h.get(DcMotor.class,"extend");
    }

    public Hanger(HardwareMap h, boolean rand){
        hang = h.get(DcMotor.class, "hang");
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend = h.get(DcMotor.class,"extend");
        localizer = new Two_Axis_Localizer(h);
        localizer.disengage();
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

    public void resetMatchTime(){
        matchTime.reset();
    }

    public boolean drop(){
        int pos = -8000;

        if (hang.getCurrentPosition() < pos){
            hang.setPower(0.0);
            hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }
        else{
            hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hang.setPower(-1.0);
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
        if (Math.abs(extend.getCurrentPosition() - 825) <= 25 || jam.time() >= 2.0){
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

    public void operate(Gamepad g, Gamepad g2){
        if (g2.x && matchTime.time() >= 3.0){
            hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hang.setTargetPosition(-5508);
            hang.setPower(1.0);
            localizer.engage();
            hangstate = true;
            complete = false;
        }

        if (Math.abs(hang.getCurrentPosition() + 5508) < 10){
            complete = true;
            if (cooldown2.time() >= 0.1) {
                hang.setTargetPosition(hang.getCurrentPosition());
                hang.setPower(0.0);
                hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangstate = false;
                cooldown.reset();
            }
        }

        if (!complete){
            cooldown2.reset();
        }
        if (!hangstate) {
            hang.setPower(g.left_stick_y);
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
            extend.setPower((g.dpad_up ? 1.0 : (g.dpad_right ? -1.0 : 0.0)));
        //}
    }

    public boolean lift(){
        boolean result = false;
        int target = 1000;

        if (first){
            jam.reset();
            extend.setTargetPosition(target);
            first = false;
        }
        if (Math.abs(extend.getCurrentPosition() - target) <= 75){
            complete = true;
            //if (cooldown2.time() >= 0.125) {
            extend.setTargetPosition(hang.getCurrentPosition());
            extend.setPower(0.0);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public boolean cycle(){
        boolean result = false;
        int target = 1050;

        if (first){
            jam.reset();
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setTargetPosition(target);
            first = false;
        }
        if (Math.abs(extend.getCurrentPosition() - target) <= 75){
            complete = true;
            //if (cooldown2.time() >= 0.125) {
            extend.setTargetPosition(hang.getCurrentPosition());
            extend.setPower(0.0);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public boolean cycle(int target){
        boolean result = false;

        if (first){
            jam.reset();
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setTargetPosition(target);
            first = false;
        }
        if (Math.abs(extend.getCurrentPosition()) >= Math.abs(target)){
            complete = true;
            //if (cooldown2.time() >= 0.125) {
            extend.setTargetPosition(hang.getCurrentPosition());
            extend.setPower(0.0);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public boolean cycle(int target, int ceiling){
        boolean result = false;

        if (first){
            jam.reset();
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setTargetPosition(target);
            first = false;
        }
        if (Math.abs(extend.getCurrentPosition()) >= Math.abs(ceiling) || Math.abs(extend.getCurrentPosition() - ceiling) <= 100){
            complete = true;
            //if (cooldown2.time() >= 0.125) {
            extend.setTargetPosition(hang.getCurrentPosition());
            extend.setPower(0.0);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hangstate = false;
            cooldown.reset();
            result = true;
            first = true;
            //}
        }
        else{
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setTargetPosition(target);
            extend.setPower(1.0);
            hangstate = true;
            complete = false;
        }
        previous = extend.getCurrentPosition();
        return result;
    }
}
