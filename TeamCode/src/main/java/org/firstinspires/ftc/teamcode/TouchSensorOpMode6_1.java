package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard6_2;

@TeleOp
public class TouchSensorOpMode6_1 extends OpMode{
    ProgrammingBoard6_2 pb = new ProgrammingBoard6_2();
    @Override
    public void init() {
        pb.init(hardwareMap);
    }

    @Override
    public void loop() {
        String touchsensor = "No values";
        if (pb.getAtmosphericPressure() == true) {
            touchsensor = "Yes value";
        }
        telemetry.addData("The state is ", touchsensor);
    }

}
