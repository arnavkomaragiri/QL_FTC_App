package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.control.Pid;
import org.firstinspires.ftc.teamcode.control.Vector2d;

@TeleOp(name="PID Test", group="Teleop")
public class PID_Test extends OpMode{
    private final double ticksPerRevolution = 1680;  // Get for your motor and gearing.
    private double prevTime;  // The last time loop() was called.
    private int prevLeftEncoderPosition;   // Encoder tick at last call to loop().
    private int prevRightEncoderPosition;  // Encoder tick at last call to loop().

    private State mRobotState;
    private final double drivePidKp = 1;     // Tuning variable for PID.
    private final double drivePidTi = 1.0;   // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = 105;  // Limit to max speed.
    private final double driveOutMax = 1.0;  // Motor output limited to 100%.

    private ElapsedTime mStateTime = new ElapsedTime();
    private ElapsedTime mTestTime = new ElapsedTime();
    private ElapsedTime mExitTime = new ElapsedTime();

    final String QTAG= "QL8719";

    DcMotor up_left;
    DcMotor up_right;
    DcMotor back_left;
    DcMotor back_right;

    double prevpos = 0;
    double prevpos2 = 0;
    double prevpos3 = 0;
    double prevpos4 = 0;

    double kp, ti, td = 0;
    boolean p = true, i = false, d = false;

    boolean first = false;


    int mode = 1;

    Pid control = new Pid(drivePidKp, drivePidTi, drivePidTd,
            -drivePidIntMax, drivePidIntMax,
            -driveOutMax, driveOutMax);

    private Pid leftDrive = null;
    private Pid rightDrive = null;

    private enum State{
        STATE_RECORD,
        STATE_RUN
    }

    public void init(){
        up_left = hardwareMap.dcMotor.get("up_left");
        up_right = hardwareMap.dcMotor.get("up_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");

        //up_left.setDirection(DcMotor.Direction.REVERSE);
        //back_left.setDirection(DcMotor.Direction.REVERSE);

        up_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mStateTime.reset();

        prevLeftEncoderPosition = up_left.getCurrentPosition();
        prevRightEncoderPosition = up_right.getCurrentPosition();

        leftDrive = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax,
                -driveOutMax, driveOutMax);
        rightDrive = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax,
                -driveOutMax, driveOutMax);
    }

    public void loop(){
        double deltaTime = time - prevTime;
        double leftSpeed = (up_left.getCurrentPosition() - prevLeftEncoderPosition) /
                deltaTime;
        double rightSpeed = (up_right.getCurrentPosition() - prevRightEncoderPosition) /
                deltaTime;
        Vector2d wheelVelocities = new Vector2d(leftSpeed, rightSpeed);
        // Track last loop() values.
        prevTime = time;
        prevLeftEncoderPosition = up_left.getCurrentPosition();
        prevRightEncoderPosition = up_right.getCurrentPosition();

        double left = 0;
        double right = 0;

        left = leftDrive.update(wheelVelocities.getX(), leftSpeed,
                deltaTime);
        right = rightDrive.update(wheelVelocities.getY(), rightSpeed,
                deltaTime);
        // Clamp motor powers.
        Vector2d motorPower = new Vector2d(left, right);
        motorPower = clampPowers(motorPower);
        left = motorPower.getX();
        right = motorPower.getY();

        up_left.setPower(left);
        up_right.setPower(right);
        back_left.setPower(left);
        back_right.setPower(right);
    }


    private boolean run_test(double desired, double mytime){
        boolean result = false;
        double actual = 0.0;
        double deltatime = 0.0;
        double power = 0.0;
        double time = 0.0;
        mExitTime.reset();
        if (mTestTime.time() % (1/512) != 0) {
            actual = getspeed(deltatime);
            telemetry.addData("Speed:", actual);
            power = control.update(desired, actual, deltatime);
            power = Range.clip(power, -1.0, 1.0);
            logMessage("Power:", Double.toString(power));

            up_left.setPower(power);
            up_right.setPower(power);
            back_left.setPower(power);
            back_right.setPower(power);
            if (Math.abs(desired - actual) <= 5.0) {
                time = mExitTime.time();
                if (time >= 1.0) {
                    result = true;
                    up_left.setPower(0.0);
                    up_right.setPower(0.0);
                    back_left.setPower(0.0);
                    back_right.setPower(0.0);
                }
            } else {
                mExitTime.reset();
            }
        }

        mytime = (mTestTime.time() - 1.0);
        return result;
    }
    private double getspeed(double delta){
        double deltaTime = 1.0 / 512.0;
        double avgspeed = 0;

        delta = deltaTime;

        sleep(Math.round(deltaTime * 1000));
        avgspeed = ((up_left.getCurrentPosition() + up_right.getCurrentPosition() + back_left.getCurrentPosition() + back_right.getCurrentPosition()) - (prevpos + prevpos2 + prevpos3 + prevpos4)) / (4 * deltaTime);
        avgspeed /= 1120;
        avgspeed *= 60;

        prevpos = up_left.getCurrentPosition();
        prevpos2 = up_right.getCurrentPosition();
        prevpos3 = back_left.getCurrentPosition();
        prevpos4 = back_right.getCurrentPosition();
        return avgspeed;
    }

    private void logMessage( String sMsgHeader, String sMsg)
    {
        telemetry.addData(sMsgHeader, sMsg);
        RobotLog.ii(QTAG, getRuntime()+ "%s :: %s", sMsgHeader, sMsg);
    }

    public static void sleep(long sleepTime)
    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
                sleepTime = wakeupTime - System.currentTimeMillis();
            }
        }
    }   //sleep

    private Vector2d clampPowers(Vector2d vec){
        double x = 0.0, y = 0.0;
        x = vec.getX();
        y = vec.getY();
        x = Range.clip(x, -1.0, 1.0);
        y = Range.clip(y, -1.0, 1.0);
        Vector2d out = new Vector2d(x, y);
        return out;
    }
}
