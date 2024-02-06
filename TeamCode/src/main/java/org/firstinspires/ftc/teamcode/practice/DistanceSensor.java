package org.firstinspires.ftc.teamcode.practice;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms.PbSensor;

@Config
@Autonomous
public class DistanceSensor extends OpMode{

    PbSensor pbSensor = new PbSensor();

    public static int redvalue = 30;


    @Override
    public void init() {
        pbSensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("d", "d");
    }

}


