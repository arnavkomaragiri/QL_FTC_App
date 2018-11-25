package org.firstinspires.ftc.teamcode.wrapper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
    private DcMotor sweeper;
    private DcMotor arm;
    private Servo back;
    private int state = 0;
    private boolean bstate = true;
    private ElapsedTime cooldown = new ElapsedTime();
    private ElapsedTime toggle = new ElapsedTime();
    boolean complete = false;
    boolean engage = false;
    private double speed = 0.4;

    private State mTransferState = State.STATE_END;

    private enum State{
        STATE_TRANSFER,
        STATE_CLEAR,
        STATE_END
    }
    
    public Arm(DcMotor sweeper, DcMotor arm, Servo back){
        this.sweeper = sweeper;
        this.arm = arm;
        this.back = back;
        back.setPosition(0.5);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void setBack(Servo back){
        this.back = back;
    }

    public Servo getBack(){
        return this.back;
    }

    public boolean getBState(){
        return bstate;
    }

    public void move(double vel, int state, boolean b_state){
        if (b_state){
            back.setPosition(0.25);
            speed = 1.0;
            bstate = false;
        }
        else{
            back.setPosition(0.5);
            speed = 0.4;
            bstate = true;
        }

        if (!complete){
            cooldown.reset();
        }

        if (Math.abs(arm.getCurrentPosition() + 700) < 10 && state == 1){
            back.setPosition(0.6);
        }

        switch (state){
            case 1:
                if (Math.abs(arm.getCurrentPosition() + 270) < 25){
                    complete = true;
                    if (cooldown.time() >= 1.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        cooldown.reset();
                        //sweeper.setPower(1.0);
                        //engage = true;
                    }
                }
                else{
                    arm.setTargetPosition(-270);
                    arm.setPower(0.7);
                    complete = false;
                }
                break;

            case 2:
                if (Math.abs(arm.getCurrentPosition() + 1400) < 25) {
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
                    arm.setTargetPosition(-1400);
                    arm.setPower(0.5);
                    complete = false;
                }
                break;
            case 3:
                if (Math.abs(arm.getCurrentPosition() + 400) < 25) {
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
                    arm.setTargetPosition(-400);
                    arm.setPower(0.7);
                    complete = false;
                }
                break;
        }
        this.sweeper.setPower(-vel * speed);
    }

    public void newState(State s){
        mTransferState = s;
    }
    
    public void move(Gamepad g){
        //arm.setPower(g.right_stick_y * 0.5);

        if (g.dpad_down && toggle.time() >= 0.25){
            if (bstate){
                back.setPosition(0.25);
                speed = 1.0;
                bstate = false;
            }
            else{
                back.setPosition(0.5);
                speed = 0.4;
                bstate = true;
            }
            toggle.reset();
        }
        if (g.y){
            if (Math.abs(arm.getCurrentPosition() + 400) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                if (mTransferState == State.STATE_END){
                    arm.setTargetPosition(-400);
                    arm.setPower(0.7);
                    complete = false;
                }
            }
            if (mTransferState == State.STATE_END) {
                newState(State.STATE_TRANSFER);
            }
            state = 1;
        }
        else if (g.a){
            if (Math.abs(arm.getCurrentPosition() + 1450) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1450);
                arm.setPower(-0.5);
                complete = false;
            }
            mTransferState = State.STATE_END;
            state = 2;
        }
        else if (g.x){
            if (Math.abs(arm.getCurrentPosition() + 400) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-400);
                arm.setPower(-0.5);
                complete = false;
            }
            mTransferState = State.STATE_END;
            state = 3;
        }

        if (Math.abs(arm.getCurrentPosition() + 700) < 10 && state == 1){
            back.setPosition(0.6);
        }

        if (!complete){
            cooldown.reset();
        }

        switch (state){
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
                switch (mTransferState){
                    case STATE_TRANSFER:
                        if (Math.abs(arm.getCurrentPosition() + 400) < 25){
                            complete = true;
                            if (cooldown.time() >= 0.25) {
                                arm.setTargetPosition(-450);
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
                        if (Math.abs(arm.getCurrentPosition() + 450) < 25){
                            complete = true;
                            if (cooldown.time() >= 0.5) {
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
                break;
            case 3:
                if (Math.abs(arm.getCurrentPosition() + 400) < 25) {
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
        /*else if (g.right_stick_y == 0.0){
            arm.setPower(0.0);
        }*/
        //if (engage){
            //sweeper.setPower(1.0);
        //}
        if (g.right_trigger != 0) {
            sweeper.setPower(-g.right_trigger * speed);
        }
        else {
            sweeper.setPower(g.left_trigger);
        }
    }

    public void move(double arm_power, double sweeper_power){
        arm.setPower(arm_power * 0.6);
        sweeper.setPower(-sweeper_power);
    }
}
