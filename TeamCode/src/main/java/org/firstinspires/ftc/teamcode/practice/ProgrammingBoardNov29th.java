package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import javax.sql.StatementEvent;

public class ProgrammingBoardNov29th {
    private DcMotor motor;
    private DigitalChannel Digital;
    private Servo servo;

    public void init(HardwareMap hwmap) {
        motor = hwmap.get(DcMotor.class, "DcMotor");
        motor.setMode();

        Digital = hwmap.get(DigitalChannel.class, "Digital Channel");


        servo = hwmap.get(Servo.class, "Servo");



    }

}
