package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.RobotLiveData;
import org.firstinspires.ftc.teamcode.control.RobotLiveSend;

@TeleOp(name = "data_test", group = "Robot-Live")
public class test_data extends OpMode{

    RobotLiveData data;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        data = RobotLiveSend.createNewRun("http://192.168.1.12");

        data.addStringData("Test", "Data works");

        RobotLiveSend.send(data, "http://192.168.1.12");
    }
}