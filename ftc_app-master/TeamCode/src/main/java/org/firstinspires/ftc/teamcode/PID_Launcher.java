package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.control.Pid;

@TeleOp(name="PID Launcher", group="Teleop")
public class PID_Launcher extends OpMode{
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
    double time = 0.0;

    boolean first = false;


    int mode = 1;

    Pid control = new Pid(drivePidKp, drivePidTi, drivePidTd,
            -drivePidIntMax, drivePidIntMax,
            -driveOutMax, driveOutMax);

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

        mStateTime.reset();
        newState(State.STATE_RECORD);
    }

    public void loop(){
        switch (mRobotState){
            case STATE_RECORD:
                telemetry.addData("State:", "STATE_RECORD");
                kp = control.getKp();
                ti = control.getTi();
                td = control.getTd();

                if (gamepad1.dpad_left){
                    if (p){
                        p = false;
                        d = true;
                    }
                    else if (i){
                        i = false;
                        p = true;
                    }
                    else if (d){
                        d = false;
                        i = true;
                    }
                    mode--;
                    if (mode < 1){
                        mode = 3;
                    }
                }
                if (gamepad1.dpad_right){
                    if (p){
                        p = false;
                        i = true;
                    }
                    else if (i){
                        i = false;
                        d = true;
                    }
                    else if (d){
                        d = false;
                        p = true;
                    }
                    mode++;
                    if (mode > 3){
                        mode = 1;
                    }
                }
                if (gamepad1.dpad_up){
                    if (p){
                        kp += 0.0001;
                    }
                    else if (i){
                        ti += 0.0001;
                    }
                    else if (d){
                        td += 0.0001;
                    }
                }
                if (gamepad1.dpad_down){
                    if (p){
                        kp -= 0.0001;
                    }
                    else if (i){
                        ti -= 0.0001;
                    }
                    else if (d){
                        td -= 0.0001;
                    }
                }
                control.tune(kp, ti, td);
                if (gamepad1.b){
                    newState(State.STATE_RUN);
                }
                break;
            case STATE_RUN:
                telemetry.addData("State:", "STATE_RUN");
                if (!first){
                    mTestTime.reset();
                    first = true;
                }
                if (run_test(50, time)){
                    newState(State.STATE_RECORD);
                    first = false;
                }
                break;
        }
        telemetry.addData("P:", kp);
        telemetry.addData("I:", ti);
        telemetry.addData("D:", td);
        telemetry.addData("Mode:", mode);
        telemetry.addData("Time:", time);
    }

    private void newState (PID_Launcher.State newState){
        // Reset State time and change to next state
        mStateTime.reset();
        mRobotState = newState;
    }

    private boolean run_test(double desired, double mytime){
        boolean result = false;
        double actual = 0.0;
        double deltatime = 0.0;
        double power = 0.0;
        double time = 0.0;
        mExitTime.reset();
        if (mTestTime.time() % 0.008125 != 0) {
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
        avgspeed /= 1680;
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
}
