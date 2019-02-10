package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Box {
    Servo box_left;
    Servo box_right;

    boolean flip = false;
    boolean mode = false;
    boolean flip2 = false;
    boolean first = true;
    boolean[] previous = {false, false, false};
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
        init();
    }

    public void init(){
        box_left.setPosition(0.925);
        box_right.setPosition(0.075);
    }

    public void compress(){
        box_left.setPosition(1.0);
        box_right.setPosition(0.0);
    }

    public void flip(boolean bState){
        if (bState){
            box_left.setPosition(0.05);
            box_right.setPosition(0.95);
        }
        else{
            box_left.setPosition(0.15);
            box_right.setPosition(0.85);
        }
    }

    public void secure(){
        box_left.setPosition(0.8);
        box_right.setPosition(0.2);
    }

    public void operate(Gamepad gamepad2, boolean bState){
        if (isPress(gamepad2.left_bumper)){// && cooldown.time() > 0.25){
            if (!flip) {
                newState(State.STATE_FLIP);
            }
            else{
                newState(State.STATE_END);
                init();
            }
        }
        if (isPress(gamepad2.right_bumper,1)){// && cooldown2.time() > 0.25){
            if (!flip2) {
                newState(State.STATE_END);
                secure();
                flip2 = true;
            }
            else{
                newState(State.STATE_END);
                init();
                flip2 = false;
            }
        }
        switch (mFlipState){
            case STATE_FLIP:
                flip(bState);
                if (mStateTime.time() >= 0.8){
                    newState(State.STATE_RETURN);
                }
                break;
            case STATE_RETURN:
                init();
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
