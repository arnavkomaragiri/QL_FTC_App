package org.firstinspires.ftc.teamcode.wrapper;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.wrapper.motors.QL_Servo;

public class Arm {
    private DcMotor sweeper;
    private CRServo intake_left;
    private CRServo intake_right;
    private DcMotor arm;
    private QL_Servo back;
    private int state = 0;
    private int previous_state = 0;
    private int offset = 0;
    private boolean bstate = true; //todo: READ: BALL STATE
    private boolean alternate = false;
    private ElapsedTime cooldown = new ElapsedTime();
    private ElapsedTime toggle = new ElapsedTime();
    private ElapsedTime hold_time = new ElapsedTime();
    private boolean hold_first = true;
    private ElapsedTime clearTime = new ElapsedTime();
    boolean complete = false;
    boolean engage = false;
    private double speed = 0.5;
    private double arm_speed = 0.0;
    private int prevPos = 0;
    private boolean[] previous = {false, false, false};
    private boolean completed = true;
    private VoltageSensor voltage;
    private boolean exit = false;
    private boolean extended = false;

    private boolean slide_automated = false;
    private double error = 0.0;
    private int armPos = 0;

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
        else if (mTransferState == State.STATE_CLEAR){
            return 2;
        }
        else{
            return 0;
        }
    }
    
    public Arm(DcMotor sweeper, DcMotor arm, Servo back){
        this.sweeper = sweeper;
        this.arm = arm;
        this.back = new QL_Servo(back);
        back.setPosition(0.342);
        this.sweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Arm(DcMotor sweeper, DcMotor arm, Servo back, VoltageSensor voltage){
        this.sweeper = sweeper;
        this.sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.arm = arm;
        this.back = new QL_Servo(back);
        this.voltage = voltage;
        back.setPosition(0.342);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Arm(DcMotor sweeper, DcMotor arm, Servo back, VoltageSensor voltage, CRServo left, CRServo right){
        this.sweeper = sweeper;
        this.sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.arm = arm;
        this.back = new QL_Servo(back);
        this.voltage = voltage;
        this.intake_left = left;
        this.intake_right = right;
        intake_right.setDirection(CRServo.Direction.REVERSE);
        back.setPosition(0.342);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Arm(HardwareMap hardwareMap){
        this.sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.intake_left = hardwareMap.get(CRServo.class, "intake_left");
        this.intake_right = hardwareMap.get(CRServo.class, "intake_right");
        this.intake_right.setDirection(CRServo.Direction.REVERSE);
        this.back = new QL_Servo(hardwareMap.get(Servo.class, "back"));
    }

    public Arm(DcMotor sweeper, DcMotor arm, Servo back, boolean alt){
        this.sweeper = sweeper;
        this.arm = arm;
        this.back = new QL_Servo(back);
        back.setPosition(0.342);
        alternate = true;
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setExit(boolean e){
        exit = e;
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

    public void diagnose(){
        arm.setPower(1.0);
        back.setPosition(0.3);
    }

    public void diagnose2(){
        arm.setPower(0.5);
        back.setPosition(0.41);
    }

    public void move(double vel, int i_state, boolean b_state){

        //arm.setPower(g.right_stick_y * 0.5);

        //if (isPress(g.dpad_down)){
        if (!bstate) {
            back.setPosition((exit ? 0.43 : 0.342));
            speed = 0.9;
        } else {
            back.setPosition(0.342);
            if (false) {
                speed = Range.clip((0.35 * 14.00) / voltage.getVoltage(), 0, 1);
            }
            else{
                speed = 0.5;
            }
            bstate = true;
        }
        //toggle.reset();
        //}

        if (i_state == 1){
            if (Math.abs(arm.getCurrentPosition() + 550 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                if (mTransferState == State.STATE_END){
                    //arm.setTargetPosition(-500 - offset);
                    //arm.setPower(1.0);
                    //arm_speed = 1.0;
                    error = sweeper.getCurrentPosition();
                    armPos = arm.getCurrentPosition();
                    sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sweeper.setTargetPosition(-50);
                    sweeper.setPower(1.0);
                    slide_automated = true;
                    complete = false;
                    completed = false;
                }
            }
            if (mTransferState == State.STATE_END) {
                newState(State.STATE_TRANSFER);
            }
            state = 1;
        }
        else if (i_state == 2){
            if (Math.abs(arm.getCurrentPosition() + 1350 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1350 - offset);
                arm.setPower(-0.75);
                arm_speed = -0.75;
                complete = false;
                completed = false;
            }
            mTransferState = State.STATE_END;
            if (!extended) {
                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sweeper.setPower(0.0);
            }
            state = 2;
        }
        else if (i_state == 3){
            if (Math.abs(arm.getCurrentPosition() + 1100 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1100 - offset);
                arm.setPower(0.7);
                complete = false;
                completed = false;
            }
            mTransferState = State.STATE_END;
            if (!extended) {
                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sweeper.setPower(0.0);
            }
            state = 3;
        }
        else if (i_state == 4){
            if (Math.abs(arm.getCurrentPosition() + 600 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-600 - offset);
                arm.setPower(0.7);
                complete = false;
                completed = false;
            }
            mTransferState = State.STATE_END;
            if (!extended) {
                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sweeper.setPower(0.0);
            }
            state = 4;
        }

        if (arm.getCurrentPosition() >= (-700 - offset) && state == 1){
            back.setPosition(0.43);
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
                        if (Math.abs(arm.getCurrentPosition() + 550 + offset) < 25){// && Math.abs(sweeper.getCurrentPosition()) < 25) {
                            complete = true;
                            if (cooldown.time() >= 0.1){
                                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                sweeper.setPower(0.0);
                            }
                            if (cooldown.time() >= 0.75) {
                                arm.setTargetPosition(-600 - offset);
                                arm.setPower(1.0);
                                complete = false;
                                cooldown.reset();
                                slide_automated = false;
                                //sweeper.setPower(1.0);
                                //engage = true;
                                newState(State.STATE_CLEAR);
                            }
                        }
                        else{
                            if (sweeper.getCurrentPosition() >= 100) {
                                double gain = (armPos + 550) / error;
                                int target = (int) Math.round((gain * sweeper.getCurrentPosition()) - 550);
                                arm.setTargetPosition(target - offset);
                                arm.setPower(1.0);
                            }
                            else {
                                arm.setTargetPosition(-550 - offset);
                                arm.setPower(1.0);
                            }
                            arm_speed = 1.0;
                            complete = false;
                        }
                        break;
                    case STATE_CLEAR:
                        if (Math.abs(arm.getCurrentPosition() + 600 + offset) < 25) {
                            complete = true;
                            if (cooldown.time() >= 0.25) {
                                arm.setTargetPosition(arm.getCurrentPosition());
                                arm.setPower(0.0);
                                complete = false;
                                completed = true;
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
                if (Math.abs(arm.getCurrentPosition() + 1350 + offset) < 25) {
                    complete = true;
                    if (bstate) {
                        back.setPosition(0.342);
                    } else {
                        back.setPosition(0.42);
                    }
                    if (cooldown.time() >= 1.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        completed = true;
                        cooldown.reset();
                    }
                    //sweeper.setPower(1.0);
                    //engage = true;
                }
                else{
                    arm.setTargetPosition(-1350 - offset);
                    arm.setPower(-0.75);
                    arm_speed = -0.75;
                    complete = false;
                }
                break;
            case 3:
                if (Math.abs(arm.getCurrentPosition() + 1100 + offset) < 25) {
                    complete = true;
                    if (cooldown.time() >= 2.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        completed = true;
                        cooldown.reset();
                    }
                    //sweeper.setPower(1.0);
                    //engage = true;
                }
                break;
            case 4:
                if (Math.abs(arm.getCurrentPosition() + 600 + offset) < 25){
                    complete = true;
                    if (cooldown.time() >= 2.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        completed = true;
                        cooldown.reset();
                    }
                }
                break;
        }
        /*else if (g.right_stick_y == 0.0){
            arm.setPower(0.0);
        }*/
        //if (engage){
        //sweeper.setPower(1.0);
        //}
        if (!slide_automated && !extended){
            if (armTest()) {
                hold_time.reset();
                hold_first = true;
                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sweeper.setPower(0.0);
            }
            else{
                if (hold_time.time() >= 0.25 && !hold_first){
                    sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sweeper.setPower(0.0);
                }
                else if (hold_first){
                    sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sweeper.setTargetPosition(sweeper.getCurrentPosition());
                    sweeper.setPower(1.0);
                    hold_time.reset();
                    hold_first = false;
                }
            }
        }
        intake_left.setPower(vel);
        intake_right.setPower(vel);
        previous_state = state;
    }

    public boolean extend(){
        if (Math.abs(sweeper.getCurrentPosition() - 650) > 50) {
            sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sweeper.setTargetPosition(650);
            sweeper.setPower(1.0);
            extended = true;
            return false;
        }
        else{
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sweeper.setPower(0.0);
            extended = false;
            return true;
        }
    }

    public boolean extend(int target){
        if (Math.abs(sweeper.getCurrentPosition() - target) > 50) {
            sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sweeper.setTargetPosition(target);
            sweeper.setPower(1.0);
            extended = true;
            return false;
        }
        else{
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sweeper.setPower(0.0);
            extended = false;
            return true;
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
            if (isPress(g.dpad_right) && !g.dpad_left) {
                if (bstate) {
                    back.setPosition(0.342);
                    speed = 0.9;
                    bstate = false;
                } else {
                    back.setPosition(0.342);
                    if (false) {
                        speed = Range.clip((0.35 * 14.00) / voltage.getVoltage(), 0, 1);
                    }
                    else{
                        speed = 0.5;
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
                    //arm.setTargetPosition(-500 - offset);
                    //arm.setPower(1.0);
                    //arm_speed = 1.0;
                    error = sweeper.getCurrentPosition();
                    armPos = arm.getCurrentPosition();
                    sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sweeper.setTargetPosition(-50);
                    sweeper.setPower(1.0);
                    slide_automated = true;
                    complete = false;
                    completed = false;
                }
            }
            if (mTransferState == State.STATE_END) {
                newState(State.STATE_TRANSFER);
            }
            state = 1;
        }
        else if (g.a){
            if (Math.abs(arm.getCurrentPosition() + 1350 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1350 - offset);
                arm.setPower(-0.75);
                arm_speed = -0.75;
                complete = false;
                completed = false;
            }
            mTransferState = State.STATE_END;
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sweeper.setPower(0.0);
            state = 2;
        }
        else if (g.x){
            if (Math.abs(arm.getCurrentPosition() + 1100 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1100 - offset);
                arm.setPower(0.7);
                complete = false;
                completed = false;
            }
            mTransferState = State.STATE_END;
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sweeper.setPower(0.0);
            state = 3;
        }

        if (arm.getCurrentPosition() >= (-700 - offset) && state == 1){
            back.setPosition(0.43);
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
                            if (Math.abs(arm.getCurrentPosition() + 550 + offset) < 25){// && Math.abs(sweeper.getCurrentPosition()) < 25) {
                                complete = true;
                                if (cooldown.time() >= 0.1){
                                    sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    sweeper.setPower(0.0);
                                }
                                if (cooldown.time() >= 0.75) {
                                    arm.setTargetPosition(-600 - offset);
                                    arm.setPower(1.0);
                                    complete = false;
                                    cooldown.reset();
                                    slide_automated = false;
                                    //sweeper.setPower(1.0);
                                    //engage = true;
                                    newState(State.STATE_CLEAR);
                                }
                            }
                            else{
                                if (sweeper.getCurrentPosition() >= 100) {
                                    double gain = (armPos + 550) / error;
                                    int target = (int) Math.round((gain * sweeper.getCurrentPosition()) - 550);
                                    arm.setTargetPosition(target - offset);
                                    arm.setPower(1.0);
                                }
                                else {
                                    arm.setTargetPosition(-550 - offset);
                                    arm.setPower(1.0);
                                }
                                arm_speed = 1.0;
                                complete = false;
                            }
                            break;
                        case STATE_CLEAR:
                            if (Math.abs(arm.getCurrentPosition() + 600 + offset) < 25) {
                                complete = true;
                                if (cooldown.time() >= 0.25) {
                                    arm.setTargetPosition(arm.getCurrentPosition());
                                    arm.setPower(0.0);
                                    complete = false;
                                    completed = true;
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
                    if (Math.abs(arm.getCurrentPosition() + 1350 + offset) < 25) {
                        complete = true;
                        if (!g.dpad_left) {
                            if (bstate) {
                                back.setPosition(0.342);
                            } else {
                                back.setPosition(0.42);
                            }
                        }
                        if (cooldown.time() >= 1.0) {
                            arm.setTargetPosition(arm.getCurrentPosition());
                            arm.setPower(0.0);
                            complete = false;
                            completed = true;
                            cooldown.reset();
                        }
                        //sweeper.setPower(1.0);
                        //engage = true;
                    }
                    else{
                        arm.setTargetPosition(-1350 - offset);
                        arm.setPower(-0.75);
                        arm_speed = -0.75;
                        complete = false;
                    }
                    break;
                case 3:
                    if (Math.abs(arm.getCurrentPosition() + 1100 + offset) < 25) {
                        complete = true;
                        if (cooldown.time() >= 2.0) {
                            arm.setTargetPosition(arm.getCurrentPosition());
                            arm.setPower(0.0);
                            complete = false;
                            completed = true;
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
        if (!slide_automated){
            if (Math.abs(g.right_stick_y) != 0.0 || armTest()) {
                hold_time.reset();
                hold_first = true;
                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sweeper.setPower(g.right_stick_y);
            }
            else{
                if (hold_time.time() >= 0.25 && !hold_first){
                    sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sweeper.setPower(0.0);
                }
                else if (hold_first){
                    sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sweeper.setTargetPosition(sweeper.getCurrentPosition());
                    sweeper.setPower(1.0);
                    hold_time.reset();
                    hold_first = false;
                }
            }
        }
        if (clearTime.time() >= 0.125){
            if (g.dpad_left && !g.dpad_down){
                //back.setPosition(0.6);
                speed = 1.0;
            }
            else{
                if (state != 1) {
                    if (bstate) {
                        back.setPosition(0.342);
                        if (false) {
                            speed = Range.clip((0.35 * 14.00) / voltage.getVoltage(), 0, 1);
                        }
                        else{
                            speed = 0.5;
                        }
                    } else {
                        back.setPosition(0.342);
                        speed = 0.9;
                    }
                }
            }
            clearTime.reset();
        }
        intake_left.setPower(g.right_trigger - g.left_trigger);
        intake_right.setPower(g.right_trigger - g.left_trigger);
        previous_state = state;
    }

    public void move(Gamepad g, Servo filter, boolean fState){
        //arm.setPower(g.right_stick_y * 0.5);

        //if (isPress(g.dpad_down)){
        if (isPress(g.dpad_right) && !g.dpad_left) {
            if (bstate) {
                back.setPosition(0.342);
                speed = 0.9;
                bstate = false;
            } else {
                back.setPosition(0.342);
                if (false) {
                    speed = Range.clip((0.35 * 14.00) / voltage.getVoltage(), 0, 1);
                }
                else{
                    speed = 0.5;
                }
                bstate = true;
            }
        }
        //toggle.reset();
        //}
        if (g.y){
            if (Math.abs(arm.getCurrentPosition() + 500 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                if (mTransferState == State.STATE_END){
                    //arm.setTargetPosition(-500 - offset);
                    //arm.setPower(1.0);
                    //arm_speed = 1.0;
                    error = 400;
                    armPos = arm.getCurrentPosition();
                    sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sweeper.setTargetPosition(-50);
                    sweeper.setPower(1.0);
                    slide_automated = true;
                    complete = false;
                    completed = false;
                }
            }
            if (mTransferState == State.STATE_END) {
                newState(State.STATE_TRANSFER);
            }
            state = 1;
        }
        else if (g.a){
            if (Math.abs(arm.getCurrentPosition() + 1350 + offset) < 60){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1350 - offset);
                arm.setPower(-0.75);
                arm_speed = -0.75;
                complete = false;
                completed = false;
            }
            mTransferState = State.STATE_END;
            filter.setPosition(0.786);
            back.setPosition(0.342);
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sweeper.setPower(g.right_stick_y);
            state = 2;
        }
        else if (g.x){
            if (Math.abs(arm.getCurrentPosition() + 1000 + offset) < 10){
                //arm.setTargetPosition(arm.getCurrentPosition());
                //arm.setPower(0.0);
                complete = true;
            }
            else {
                arm.setTargetPosition(-1000 - offset);
                arm.setPower(0.7);
                complete = false;
                completed = false;
            }
            mTransferState = State.STATE_END;
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sweeper.setPower(g.right_stick_y);
            state = 3;
        }

        if (mTransferState == State.STATE_CLEAR && arm.getCurrentPosition() < (-510 - offset) && (state == 1 || state == 0)){
            if (fState){
                filter.setPosition(0.434);
            }
        }

        if (arm.getCurrentPosition() >= (-600 - offset) && state == 1){
            back.setPosition(0.6);
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
                        if (Math.abs(arm.getCurrentPosition() + 500 + offset) < 25){// && Math.abs(sweeper.getCurrentPosition()) < 25) {
                            complete = true;
                            if (cooldown.time() >= 0.1){
                                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                sweeper.setPower(g.right_stick_y);
                            }
                            if (cooldown.time() >= 1.0) {
                                arm.setTargetPosition(-600 - offset);
                                arm.setPower(1.0);
                                complete = false;
                                cooldown.reset();
                                slide_automated = false;
                                //sweeper.setPower(1.0);
                                //engage = true;
                                newState(State.STATE_CLEAR);
                            }
                        }
                        else{
                            if (sweeper.getCurrentPosition() >= 100) {
                                double gain = (armPos + 500) / error;
                                int target = (int) Math.round((gain * sweeper.getCurrentPosition()) - 500);
                                if (target < -1100){
                                    target = -1100;
                                }
                                arm.setTargetPosition(target - offset);
                                arm.setPower(1.0);
                            }
                            else {
                                arm.setTargetPosition(-500 - offset);
                                arm.setPower(1.0);
                            }
                            arm_speed = 1.0;
                            complete = false;
                        }
                        break;
                    case STATE_CLEAR:
                        if (Math.abs(arm.getCurrentPosition() + 600 + offset) < 25) {
                            complete = true;
                            if (fState){
                                filter.setPosition(0.487);
                            }
                            if (cooldown.time() >= 0.25) {
                                arm.setTargetPosition(arm.getCurrentPosition());
                                arm.setPower(0.0);
                                complete = false;
                                completed = true;
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
                if (Math.abs(arm.getCurrentPosition() + 1350 + offset) < 60) {
                    complete = true;
                    if (!g.dpad_left) {
                        if (bstate) {
                            back.setPosition(0.342);
                        } else {
                            back.setPosition(0.342);
                        }
                    }
                    if (cooldown.time() >= 0.1){
                        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sweeper.setPower(g.right_stick_y);
                    }
                    if (cooldown.time() >= 1.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        completed = true;
                        cooldown.reset();
                    }
                    //sweeper.setPower(1.0);
                    //engage = true;
                }
                else{
                    arm.setTargetPosition(-1350 - offset);
                    arm.setPower(-0.75);
                    arm_speed = -0.75;
                    complete = false;
                }
                break;
            case 3:
                if (Math.abs(arm.getCurrentPosition() + 1000 + offset) < 60) {
                    complete = true;
                    back.setPosition(0.342);
                    if (cooldown.time() >= 0.1){
                        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sweeper.setPower(g.right_stick_y);
                    }
                    if (cooldown.time() >= 2.0) {
                        arm.setTargetPosition(arm.getCurrentPosition());
                        arm.setPower(0.0);
                        complete = false;
                        completed = true;
                        cooldown.reset();
                    }
                    //sweeper.setPower(1.0);
                    //engage = true;
                }
                else{
                    if (!complete) {
                        back.setPosition(0.224);
                    }
                }
                break;
        }
        /*else if (g.right_stick_y == 0.0){
            arm.setPower(0.0);
        }*/
        //if (engage){
        //sweeper.setPower(1.0);
        //}
        if (!slide_automated){
            if (Math.abs(g.right_stick_y) != 0.0 || armTest()) {
                hold_time.reset();
                hold_first = true;
                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sweeper.setPower(g.right_stick_y);
            }
            else{
                if (hold_time.time() >= 0.25 && !hold_first){
                    sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sweeper.setPower(g.right_stick_y);
                }
                else if (hold_first){
                    sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sweeper.setTargetPosition(sweeper.getCurrentPosition());
                    sweeper.setPower(1.0);
                    hold_time.reset();
                    hold_first = false;
                }
            }
        }
        if (clearTime.time() >= 0.125){
            if (g.dpad_left && !g.dpad_down){
                //back.setPosition(0.6);
                speed = 1.0;
            }
            else{
                if (state != 1 && state != 3) {
                    if (bstate) {
                        back.setPosition(0.342);
                        if (false) {
                            speed = Range.clip((0.35 * 14.00) / voltage.getVoltage(), 0, 1);
                        }
                        else{
                            speed = 0.5;
                        }
                    } else {
                        back.setPosition(0.342);
                        speed = 0.9;
                    }
                }
            }
            clearTime.reset();
        }
        intake_left.setPower(g.right_trigger - g.left_trigger);
        intake_right.setPower(g.right_trigger - g.left_trigger);
        previous_state = state;
    }

    public boolean armTest(){
        if (completed){
            if (previous_state == state){
                return true;
            }
            return false;
        }
        return false;
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
        offset = -arm.getCurrentPosition() - 1450;
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
