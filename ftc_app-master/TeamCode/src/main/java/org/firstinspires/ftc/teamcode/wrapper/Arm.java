package org.firstinspires.ftc.teamcode.wrapper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    private DcMotor sweeper;
    private DcMotor arm;
    
    public Arm(DcMotor sweeper, DcMotor arm){
        this.sweeper = sweeper;
        this.arm = arm;
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            arm.setPower(0.7);
        }
        else if (g.a){
            arm.setPower(-0.5);
        }
        else if (g.right_stick_y == 0.0){
            arm.setPower(0.0);
        }
        sweeper.setPower(-g.right_trigger);
    }

    public void move(double arm_power, double sweeper_power){
        arm.setPower(arm_power * 0.6);
        sweeper.setPower(-sweeper_power);
    }
}
