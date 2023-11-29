package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard7_2;
@TeleOp()
public class MotorOpMode7_1 extends OpMode{
    ProgrammingBoard7_2 board = new ProgrammingBoard7_2();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {
        board.setMotorSpeed(0.5);
        telemetry.addData("Motor rotation", board.motorRotation());
    }
}
