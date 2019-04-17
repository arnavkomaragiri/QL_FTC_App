package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Box {
    Servo box_left;
    Servo box_right;

    Servo filter;

    boolean flip = false;
    boolean mode = false;
    boolean flip2 = false;
    boolean first = true;
    boolean alt = false;
    boolean[] previous = {false, false, false};
    boolean secured = false;
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime cooldown2 = new ElapsedTime();

    private State mFlipState = State.STATE_END;
    private ElapsedTime mStateTime = new ElapsedTime();

    public enum State{
        STATE_FLIP,
        STATE_RETURN,
        STATE_END
    }

    public Box(Servo box_left, Servo box_right){
        this.box_left = box_left;
        this.box_right = box_right;
        init();
    }

    public Box(HardwareMap h){
        this.box_left = h.get(Servo.class, "box_left");
        this.box_right = h.get(Servo.class, "box_right");
        this.filter = h.get(Servo.class, "filter");
        e_init();
    }

    public void init(){
        box_left.setPosition(0.8875); //0.85
        box_right.setPosition(0.1125); //0.15
        filter.setPosition(0.316);
    }

    public void e_init(){
        box_left.setPosition(0.85); //0.85
        box_right.setPosition(0.15); //0.15
        filter.setPosition(0.892);
    }

    public void setAlt(boolean alt){
        this.alt = alt;
    }

    public void compress(){
        box_left.setPosition(1.0);
        box_right.setPosition(0.0);
        filter.setPosition(0.316);
    }

    public void flip(boolean bState){
        if (bState){
            //box_left.setPosition(0.075);
            //box_right.setPosition(0.925);
            box_left.setPosition(0.175);
            box_right.setPosition(0.825);
        }
        else{
            box_left.setPosition(0.175);
            box_right.setPosition(0.825);
        }
        secured = false;
    }

    public Servo getFilter(){
        return this.filter;
    }

    public void flip(boolean bState, boolean fState){
        if (bState && !fState){
            box_left.setPosition(0.075);
            box_right.setPosition(0.925);
            //box_left.setPosition(0.175);
            //box_right.setPosition(0.825);
        }
        else if (!bState && !fState){
            box_left.setPosition(0.175);
            box_right.setPosition(0.825);
        }
        else{
            box_left.setPosition(0.125);
            box_right.setPosition(0.875);
        }
        if (!fState){
            filter.setPosition(0.316);
        }
        else{
            filter.setPosition(0.434);
        }
        secured = false;
    }

    public void secure(){
        box_left.setPosition(0.6);
        box_right.setPosition(0.4);
        if (filter.getPosition() != 0.434) {
            filter.setPosition(0.316);
        }
        secured = true;
    }

    public void secure(boolean fState){
        box_left.setPosition(0.6);
        box_right.setPosition(0.4);
        if (fState){
            filter.setPosition(0.434);
        }
        else{
            filter.setPosition(0.316);
        }
        secured = true;
    }

    public boolean getSecured(){
        return secured;
    }

    public void operate(Gamepad gamepad2, boolean bState){
        if (isPress(gamepad2.left_bumper)){// && cooldown.time() > 0.25){
            if (!flip) {
                newState(State.STATE_FLIP);
            }
            else{
                newState(State.STATE_END);
                e_init();
            }
            secured = false;
        }
        if (isPress(gamepad2.right_bumper,1)){// && cooldown2.time() > 0.25){
            if (!flip2) {
                newState(State.STATE_END);
                secure();
                flip2 = true;
            }
            else{
                newState(State.STATE_END);
                e_init();
                secured = false;
                flip2 = false;
            }
        }
        switch (mFlipState){
            case STATE_FLIP:
                if (!alt) {
                    flip(bState);
                }
                else{
                    flip(!bState);
                }
                if (mStateTime.time() >= 0.9){
                    newState(State.STATE_RETURN);
                }
                break;
            case STATE_RETURN:
                e_init();
                newState(State.STATE_END);
                break;
        }
    }
    public void operate(Gamepad gamepad2, boolean bState, boolean fState){
        if (isPress(gamepad2.left_bumper)){// && cooldown.time() > 0.25){
            if (!flip) {
                newState(State.STATE_FLIP);
            }
            else{
                newState(State.STATE_END);
                e_init();
            }
            secured = false;
        }
        if (isPress(gamepad2.right_bumper,1)){// && cooldown2.time() > 0.25){
            if (!flip2) {
                newState(State.STATE_END);
                secure(fState);
                flip2 = true;
            }
            else{
                newState(State.STATE_END);
                e_init();
                secured = false;
                flip2 = false;
            }
        }
        switch (mFlipState){
            case STATE_FLIP:
                if (!alt) {
                    flip(bState, fState);
                }
                else{
                    flip(!bState, fState);
                }
                if (mStateTime.time() >= 0.9){
                    newState(State.STATE_RETURN);
                }
                break;
            case STATE_RETURN:
                e_init();
                newState(State.STATE_END);
                break;
        }
    }
    private void newState(State s){
        mFlipState = s;
        mStateTime.reset();
    }
    private boolean isPress(boolean input){
        boolean result = false;
        if (!previous[0] && input){
            result = true;
        }
        else{
            result = false;
        }
        previous[0] = input;
        return result;
    }
    private boolean isPress(boolean input, int mode){
        boolean result = false;
        if (mode == 1) {
            if (!previous[mode] && input) {
                result = true;
            } else if (previous[mode] && !input){
                result = false;
            }
            previous[mode] = input;
        }
        else if (mode == 0){
            if (!previous[0] && input){
                result = true;
            }
            else{
                result = false;
            }
            previous[0] = input;
        }
        return result;
    }
}
