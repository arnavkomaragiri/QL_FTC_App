package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.control.Pid;

public class Mecanum_Drive {
    private final double ticksPerRevolution = 1000;  // Get for your motor and gearing.
    private double prevTime;  // The last time loop() was called.
    private int prevLeftEncoderPosition;   // Encoder tick at last call to loop().
    private int prevRightEncoderPosition;  // Encoder tick at last call to loop().

    private final double drivePidKp = 1;     // Tuning variable for PID.
    private final double drivePidTi = 1.0;   // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = 105;  // Limit to max speed.
    private final double driveOutMax = 1.0;  // Motor output limited to 100%.

    DcMotor up_left;
    DcMotor up_right;
    DcMotor back_left;
    DcMotor back_right;

    private Pid leftDrive = null;
    private Pid rightDrive = null;

    Mecanum_Drive(DcMotor names[]){
        up_left = names[0];
        up_right = names[1];
        back_left = names[2];
        back_right = names[3];
    }

    public void drive(double r, double angle, double rightX){
        double robotAngle = angle - (Math.PI / 4);

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        up_left.setPower(v1);
        up_right.setPower(v2);
        back_left.setPower(v3);
        back_right.setPower(v4);
    }
}
