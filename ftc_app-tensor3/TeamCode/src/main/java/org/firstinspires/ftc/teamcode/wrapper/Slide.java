package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {
    Box box;
    Hanger hanger;

    boolean[] previous = {false, false};

    boolean first = true;

    boolean alt = false;

    double box_time = 0.0;

    double time_offset = 0.0;

    private State mRobotState = State.STATE_END;

    private ElapsedTime mStateTime = new ElapsedTime();

    private boolean busy = false;

    public enum State{
        STATE_LIFT,
        STATE_CONTRACT,
        STATE_END
    }

    private void newState(State s){
        mRobotState = s;
        mStateTime.reset();
    }

    public Slide(HardwareMap h){
        box = new Box(h);
        hanger = new Hanger(h, true);
    }

    public boolean inBounds(int val){
        if (Math.abs(val) <= 200 || Math.abs(val - 1050) <= 200){
            return true;
        }
        else{
            return false;
        }
    }

    public void setAlt(boolean alt){
        this.alt = alt;
    }

    public State getState(){
        return this.mRobotState;
    }

    public void operate(Gamepad g, boolean bState, Telemetry t, Gamepad g2){
        double rate = 0.0;
        double predicted_time = 0.0;
        double final_time = 0.0;

        if (isPress(g.left_bumper)){
            if (Math.abs(g.right_stick_y) <= 0.5){// && inBounds(hanger.getExtend().getCurrentPosition())) {
                busy = true;
                newState(State.STATE_LIFT);
            }
        }
        else if (!busy){
            box.operate(g, (alt != bState));
            hanger.operate(g, g2);
        }
        if (Math.abs(g.right_stick_y) > 0.5 && busy){
            box.init();
            hanger.getExtend().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hanger.getExtend().setPower(0.0);
            busy = false;
            first = true;
            newState(State.STATE_END);
        }

        switch (mRobotState){
            case STATE_LIFT:
                if (hanger.getExtend().getCurrentPosition() >= (box.getSecured() ? 500 : 200)){
                    rate = hanger.getExtend().getCurrentPosition() / mStateTime.time();
                    predicted_time = (1000 - hanger.getExtend().getCurrentPosition()) / rate;
                    predicted_time += mStateTime.time();
                    time_offset = predicted_time - 0.9;
                    if (first){
                        box_time = mStateTime.time() + (box.getSecured() ? 0.626 : 0.9); //todo: tune the 0.9 to match box flip speed, can't really do much without encoders/potentiometers
                        first = false;
                    }
                }
                if (predicted_time == 0.0 || box_time == 0.0){
                    final_time = 2.0;
                }
                else{
                    if (predicted_time >= box_time){
                        final_time = predicted_time;
                    }
                    else{
                        final_time = box_time;
                    }
                }
                t.addData("Time: ", final_time);
                t.addData("Offset: ", time_offset);
                if (hanger.cycle(1100, 950) && mStateTime.time() >= final_time + 0.25){
                    newState(State.STATE_CONTRACT);
                }
                break;
            case STATE_CONTRACT:
                box.init();
                if (hanger.contract()){
                    newState(State.STATE_END);
                    busy = false;
                    first = true;
                }
                break;
        }

        if (hanger.getExtend().getCurrentPosition() >= (box.getSecured() ? 500 : 200) && busy && mRobotState != State.STATE_CONTRACT){
            if (alt) {
                box.flip(!bState);
            }
            else{
                box.flip(bState);
            }
        }
    }

    public void operate(Gamepad g, boolean bState, boolean fState, Telemetry t, Gamepad g2){
        double rate = 0.0;
        double predicted_time = 0.0;
        double final_time = 0.0;

        if (isPress(g.left_bumper)){
            if (Math.abs(g.right_stick_y) <= 0.5){// && inBounds(hanger.getExtend().getCurrentPosition())) {
                busy = true;
                newState(State.STATE_LIFT);
            }
        }
        else if (!busy){
            box.operate(g, (alt != bState), fState);
            hanger.operate(g, g2);
        }
        if (Math.abs(g.right_stick_y) > 0.5 && busy){
            box.e_init();
            hanger.getExtend().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hanger.getExtend().setPower(0.0);
            busy = false;
            first = true;
            newState(State.STATE_END);
        }

        switch (mRobotState){
            case STATE_LIFT:
                if (hanger.getExtend().getCurrentPosition() >= (box.getSecured() ? 500 : 200)){
                    rate = hanger.getExtend().getCurrentPosition() / mStateTime.time();
                    predicted_time = (1100 - hanger.getExtend().getCurrentPosition()) / rate;
                    predicted_time += mStateTime.time();
                    time_offset = predicted_time - 0.9;
                    if (first){
                        box_time = mStateTime.time() + (box.getSecured() ? 0.626 : 0.9); //todo: tune the 0.9 to match box flip speed, can't really do much without encoders/potentiometers
                        first = false;
                    }
                }
                else{
                    if (fState){
                        box.secure(fState);
                    }
                }
                if (predicted_time == 0.0 || box_time == 0.0){
                    final_time = 2.0;
                }
                else{
                    if (predicted_time >= box_time){
                        final_time = predicted_time;
                    }
                    else{
                        final_time = box_time;
                    }
                }
                t.addData("Time: ", final_time);
                t.addData("Offset: ", time_offset);
                if ((hanger.cycle(1100, 1100) && mStateTime.time() >= final_time + (fState ? 0.5 : 0.0)) || mStateTime.time() >= 1.5){
                    newState(State.STATE_CONTRACT);
                }
                break;
            case STATE_CONTRACT:
                box.e_init();
                if (hanger.contract()){
                    newState(State.STATE_END);
                    busy = false;
                    first = true;
                    //fState = false;
                }
                break;
        }

        if (hanger.getExtend().getCurrentPosition() >= (box.getSecured() ? 500 : 200) && busy && mRobotState != State.STATE_CONTRACT){
            if (alt) {
                box.flip(!bState, fState);
            }
            else{
                box.flip(bState, fState);
            }
        }
    }

    public boolean dump(boolean fState, boolean bState, Telemetry t){
        double rate = 0.0;
        double predicted_time = 0.0;
        double final_time = 0.0;
        boolean result = false;

        if (!busy){
            busy = true;
            newState(State.STATE_LIFT);
        }

        switch (mRobotState){
            case STATE_LIFT:
                if (hanger.getExtend().getCurrentPosition() >= (box.getSecured() ? 500 : 200)){
                    rate = hanger.getExtend().getCurrentPosition() / mStateTime.time();
                    predicted_time = (1100 - hanger.getExtend().getCurrentPosition()) / rate;
                    predicted_time += mStateTime.time();
                    time_offset = predicted_time - 0.9;
                    if (first){
                        box_time = mStateTime.time() + (box.getSecured() ? 0.626 : 0.9); //todo: tune the 0.9 to match box flip speed, can't really do much without encoders/potentiometers
                        first = false;
                    }
                }
                else{
                    if (fState){
                        box.secure();
                    }
                }
                if (predicted_time == 0.0 || box_time == 0.0){
                    final_time = 2.0;
                }
                else{
                    if (predicted_time >= box_time){
                        final_time = predicted_time;
                    }
                    else{
                        final_time = box_time;
                    }
                }
                t.addData("Time: ", final_time);
                t.addData("Offset: ", time_offset);
                if ((hanger.cycle(1100, 1100) && mStateTime.time() >= final_time + (fState ? 0.5 : 0.0)) || mStateTime.time() >= 1.5){
                    newState(State.STATE_CONTRACT);
                }
                break;
            case STATE_CONTRACT:
                box.e_init();
                if (hanger.contract()){
                    newState(State.STATE_END);
                    busy = false;
                    first = true;
                    //fState = false;
                    result = true;
                }
                break;
        }

        if (hanger.getExtend().getCurrentPosition() >= (box.getSecured() ? 500 : 200) && busy && mRobotState != State.STATE_CONTRACT){
            if (alt) {
                box.flip(!bState, fState);
            }
            else{
                box.flip(bState, fState);
            }
        }

        return result;
    }

    public Box getBox(){
        return this.box;
    }

    public Hanger getHanger(){
        return this.hanger;
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
}
