package org.firstinspires.ftc.teamcode.wrapper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.wrapper.motors.QL_Servo;

public class Arm {
    private DcMotor sweeper;
    private DcMotor arm;
    private QL_Servo back;
    private int state = 0;
    private int offset = 0;
    private boolean bstate = true; //todo: READ: BALL STATE
    private boolean alternate = false;
    private ElapsedTime cooldown = new ElapsedTime();
    private ElapsedTime toggle = new ElapsedTime();
    private ElapsedTime clearTime = new ElapsedTime();
    boolean complete = false;
    boolean engage = false;
    private double speed = 0.5;
    private double arm_speed = 0.0;
    private int prevPos = 0;
    private boolean[] previous = {false, false, false};
    private VoltageSensor voltage;

    private State mTransferState = State.STATE_END;

    private enum State{
        STATE_TRANSFER,
        STATE_CLEAR,
        STATE_END
    }

    public int getmTransferState(){
        if (mTransferState == State.STATE_END){
            return 1;
        }
        else{
            return 0;
        }
    }
    
    public Arm(DcMotor sweeper, DcMotor arm, Servo back){
        this.sweeper = sweeper;
        this.arm = arm;
        this.back = new QL_Servo(back);
        back.setPosition(0.41);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Arm(DcMotor sweeper, DcMotor arm, Servo back, VoltageSensor voltage){
        this.sweeper = sweeper;
        this.arm = arm;
        this.back = new QL_Servo(back);
        this.voltage = voltage;
        back.setPosition(0.41);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Arm(DcMotor sweeper, DcMotor arm, Servo back, boolean alt){
        this.sweeper = sweeper;
        this.arm = arm;
        this.back = new QL_Servo(back);
        back.setPosition(0.41);
        alternate = true;
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArm(DcMotor arm){
        this.arm = arm;
    }
    
    public DcMotor getArm(){
        return this.arm;
    }
    
    public void setSweeper(DcMotor sweeper){
        this.sweeper = sweeper;
    }
    
    public DcMotor getSweeper(){
        return this.sweeper;
    }

    public double getVoltage(){
        return voltage.getVoltage();
    }

    public void setBack(QL_Servo back){
        this.back = back;
    }

    public QL_Servo getBack(){
        return this.back;
    }

    public boolean getBState(){
        return bstate;
    }

    public void move(double vel, int state, boolean b_state){
        if (state != 1) {
            if (b_state) {
                back.setPosition(0.25);
                speed = 1.0;
                bstate = false;
            } else {
                back.setPosition(0.4);
                speed = 0.5;
                bstate = true;
            }
        }

        if (!complete){
            cooldown.reset();
        }

        if (arm.getCurrentPosition() >= -700 && state == 1){
            back.setPosition(0.6);
        }

        switch (state){
            case 1:
                switch (mTransferState){
                    case STATE_END:
                        newState(State.STATE_TRANSFER);
                        break;
                    case STATE_TRANSFER:
                        arm.setTargetPosition(-400);
                        arm.setPower(0.7);
                        complete = false;
                        if (Math.abs(arm.getCurrentPosition() + 400) < 25){
                            complete = true;
                            if (cooldown.time() >= 0.125) {
                                arm.setTargetPosition(-600);
                                arm.setPower(0.7);
                                complete = false;
                                cooldown.reset();
                                //sweeper.setPower(1.0);
                                //engage = true;
                                newState(State.STATE_CLEAR);
                            }
                        }
                        break;
                    case STATE_CLEAR:
                        if (Math.abs(arm.getCurrentPosition() + 600) < 25){
                            complete = true;
                            if (cooldown.time() >= 0.25) {
                                arm.setTargetPosition(arm.getCurrentPosition());
                                arm.setPower(0.0);
                                complete = false;
                                cooldown.reset();
                                //sweeper.setPower(1.0);
                                //engage = true;
                                newState(State.STATE_END);
                            }
                        }
                        break;
                }
                break;

            case 2:
                mTransferState = State.STATE_END;
                if (Math.abs(arm.getCurrentPosition() + 1450) < 25) {
                    complete = true;
                    if (bstate){
                        back.setPosition(0.5);
                    }
                    else{
                        back.setPosition(0.3);
                    }
                    if (cooldown.time() >= 1.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        cooldown.reset();
                    }
                    //sweeper.setPower(1.0);
                    //engage = true;
                }
                else{
                    arm.setTargetPosition(-1450);
                    arm.setPower(0.5);
                    complete = false;
                }
                break;
            case 3:
                mTransferState = State.STATE_END;
                if (Math.abs(arm.getCurrentPosition() + 700) < 25) {
                    complete = true;
                    if (cooldown.time() >= 1.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        cooldown.reset();
                    }
                    //sweeper.setPower(1.0);
                    //engage = true;
                }
                else{
                    arm.setTargetPosition(-700);
                    arm.setPower(0.7);
                    complete = false;
                }
                break;
        }
        if (vel != -2.0) {
            this.sweeper.setPower(-vel * speed);
        }
    }

    private void newState(State s){
        mTransferState = s;
    }

    public double getArmPos(){
        return arm.getCurrentPosition() - offset;
    }
    
    public void move(Gamepad g){
        //arm.setPower(g.right_stick_y * 0.5);

        //if (isPress(g.dpad_down)){
            if (isPress(g.dpad_down) && !g.dpad_left) {
                if (bstate) {
                    back.setPosition(0.3);
                    speed = 0.75;
                    bstate = false;
                } else {
                    back.setPosition(0.41);
                    if (voltage != null && false) {
                        speed = Range.clip((0.35 * 14.00) / voltage.getVoltage(), 0, 1);
                    }
                    else{
                        speed = 0.45;
                    }
                    bstate = true;
                }
            }
            //toggle.reset();
        //}
        if (g.y){
            if (Math.abs(arm.getCurrentPosition() + 550 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                if (mTransferState == State.STATE_END){
                    arm.setTargetPosition(-550 - offset);
                    arm.setPower(1.0);
                    arm_speed = 1.0;
                    complete = false;
                }
            }
            if (mTransferState == State.STATE_END) {
                newState(State.STATE_TRANSFER);
            }
            state = 1;
        }
        else if (g.a){
            if (Math.abs(arm.getCurrentPosition() + 1450 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1450 - offset);
                arm.setPower(-0.75);
                arm_speed = -0.75;
                complete = false;
            }
            mTransferState = State.STATE_END;
            state = 2;
        }
        /*else if (g.x){
            if (Math.abs(arm.getCurrentPosition() + 700 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-600 - offset);
                arm.setPower(-0.5);
                complete = false;
            }
            mTransferState = State.STATE_END;
            state = 3;
        }*/

        if (arm.getCurrentPosition() >= (-700 - offset) && state == 1){
            back.setPosition(0.4);
        }
        /*else{
            if (state == 2 && Math.abs(arm.getCurrentPosition() + 1450) > 25 && (back.getPosition() != 0.25 || back.getPosition() != 0.5)) {
                if (bstate) {
                    back.setPosition(0.25);
                    speed = 1.0;
                } else {
                    back.setPosition(0.5);
                    speed = 0.35;
                }
            }
        }*/

        if (!complete){
            cooldown.reset();
        }
        if (!g.b) {
            if (previous[2]){
                arm.setTargetPosition(prevPos);
                arm.setPower(arm_speed);
                previous[2] = false;
            }
            switch (state) {
                case 1:
                /*if (Math.abs(arm.getCurrentPosition() + 450) < 25){
                    complete = true;
                    if (cooldown.time() >= 1.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        cooldown.reset();
                        //sweeper.setPower(1.0);
                        //engage = true;
                    }
                }*/
                    switch (mTransferState) {
                        case STATE_TRANSFER:
                            if (Math.abs(arm.getCurrentPosition() + 550 + offset) < 25) {
                                complete = true;
                                if (cooldown.time() >= 0.75) {
                                    arm.setTargetPosition(-650 - offset);
                                    arm.setPower(1.0);
                                    complete = false;
                                    cooldown.reset();
                                    //sweeper.setPower(1.0);
                                    //engage = true;
                                    newState(State.STATE_CLEAR);
                                }
                            }
                            else{
                                arm.setTargetPosition(-550 - offset);
                                arm.setPower(1.0);
                                arm_speed = 1.0;
                                complete = false;
                            }
                            break;
                        case STATE_CLEAR:
                            if (Math.abs(arm.getCurrentPosition() + 650 + offset) < 25) {
                                complete = true;
                                if (cooldown.time() >= 0.25) {
                                    arm.setTargetPosition(arm.getCurrentPosition());
                                    arm.setPower(0.0);
                                    complete = false;
                                    cooldown.reset();
                                    //sweeper.setPower(1.0);
                                    //engage = true;
                                    newState(State.STATE_END);
                                }
                            }
                            break;
                    }
                    break;

                case 2:
                    if (Math.abs(arm.getCurrentPosition() + 1450 + offset) < 25) {
                        complete = true;
                        if (!g.dpad_left) {
                            if (bstate) {
                                back.setPosition(0.41);
                            } else {
                                back.setPosition(0.3);
                            }
                        }
                        if (cooldown.time() >= 1.0) {
                            arm.setTargetPosition(arm.getCurrentPosition());
                            arm.setPower(0.0);
                            complete = false;
                            cooldown.reset();
                        }
                        //sweeper.setPower(1.0);
                        //engage = true;
                    }
                    else{
                        arm.setTargetPosition(-1450 - offset);
                        arm.setPower(-0.75);
                        arm_speed = -0.75;
                        complete = false;
                    }
                    break;
                case 3:
                    if (Math.abs(arm.getCurrentPosition() + 600 + offset) < 25) {
                        complete = true;
                        if (cooldown.time() >= 1.0) {
                            arm.setTargetPosition(arm.getCurrentPosition());
                            arm.setPower(0.0);
                            complete = false;
                            cooldown.reset();
                        }
                        //sweeper.setPower(1.0);
                        //engage = true;
                    }
                    break;
            }
        }
        else{
            prevPos = arm.getTargetPosition();
            arm.setTargetPosition(-1000 - offset);
            arm.setPower(0.7);
            previous[2] = true;
        }
        /*else if (g.right_stick_y == 0.0){
            arm.setPower(0.0);
        }*/
        //if (engage){
            //sweeper.setPower(1.0);
        //}
        if (clearTime.time() >= 0.125){
            if (g.dpad_left && !g.dpad_down){
                back.setPosition(0.6);
                speed = 1.0;
            }
            else{
                if (state != 1) {
                    if (bstate) {
                        back.setPosition(0.41);
                        if (voltage != null && false) {
                            speed = Range.clip((0.35 * 14.00) / voltage.getVoltage(), 0, 1);
                        }
                        else{
                            speed = 0.45;
                        }
                    } else {
                        back.setPosition(0.3);
                        speed = 0.85;
                    }
                }
            }
            clearTime.reset();
        }
        if (g.right_trigger != 0) {
            sweeper.setPower(-g.right_trigger * speed);
        }
        else {
            sweeper.setPower(g.left_trigger * 0.7);
        }
    }

    public void move(double arm_power, double sweeper_power){
        arm.setPower(arm_power * 0.6);
        sweeper.setPower(-sweeper_power);
    }

    public void reset(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(1.0);
    }

    public void recalibrate(){
        offset = -1450;
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
