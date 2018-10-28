package org.firstinspires.ftc.teamcode.wrapper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    private DcMotor sweeper;
    private DcMotor arm;
    private int state = 0;
    
    public Arm(DcMotor sweeper, DcMotor arm){
        this.sweeper = sweeper;
        this.arm = arm;
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
    
    public void move(Gamepad g){
        //arm.setPower(g.right_stick_y * 0.5);
        if (g.y){
            if (Math.abs(arm.getCurrentPosition() + 270) < 10){
                arm.setTargetPosition(arm.getCurrentPosition());
                arm.setPower(0.0);
            }
            else {
                arm.setTargetPosition(-270);
                arm.setPower(0.7);
            }
            state = 1;
        }
        else if (g.a){
            if (Math.abs(arm.getCurrentPosition() + 1350) < 10){
                arm.setTargetPosition(arm.getCurrentPosition());
                arm.setPower(0.0);
            }
            else {
                arm.setTargetPosition(-1350);
                arm.setPower(-0.5);
            }
            state = 2;
        }

        switch (state){
            case 1:
                if (Math.abs(arm.getCurrentPosition() + 270) < 10){
                    arm.setTargetPosition(arm.getCurrentPosition());
                    arm.setPower(0.0);
                }
                break;

            case 2:
                if (Math.abs(arm.getCurrentPosition() + 1350) < 25){
                    arm.setTargetPosition(arm.getCurrentPosition());
                    arm.setPower(0.0);
                }
                break;
        }
        /*else if (g.right_stick_y == 0.0){
            arm.setPower(0.0);
        }*/
        if (g.right_trigger != 0) {
            sweeper.setPower(-g.right_trigger);
        }
        else{
            sweeper.setPower(g.left_trigger);
        }
    }

    public void move(double arm_power, double sweeper_power){
        arm.setPower(arm_power * 0.6);
        sweeper.setPower(-sweeper_power);
    }
}
