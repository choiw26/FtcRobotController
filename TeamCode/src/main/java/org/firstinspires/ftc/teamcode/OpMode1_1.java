package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;
@TeleOp()
public class OpMode1_1 extends OpMode{
    ProgrammingBoard pb = new ProgrammingBoard();
    @Override
    public void init() {
        pb.init(hardwareMap);
    }
    @Override
    public void loop() {
        telemetry.addData("The number of revelation is ", pb.NumberOfRev());
        if (gamepad1.a) {
            pb.setMotor(1);
        }
    }
}
