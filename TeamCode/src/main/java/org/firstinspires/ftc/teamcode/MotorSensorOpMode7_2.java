package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard7_2;

@TeleOp()
public class MotorSensorOpMode7_2 extends OpMode{
    ProgrammingBoard7_2 pb = new ProgrammingBoard7_2();
    @Override
    public void init() {
        pb.init(hardwareMap);
    }

    @Override
    public void loop() {
        pb.setMotorSpeed(0.5);
        telemetry.addData("The number of rotation is ", pb.motorRotation());
    }
}
