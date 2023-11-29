package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard7_2;
@TeleOp
public class OpMode7_3 extends OpMode{
    ProgrammingBoard7_2 pb = new ProgrammingBoard7_2();
    @Override
    public void init() {
        pb.init(hardwareMap);
    }
    @Override
    public void loop() {
        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotangle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        if (gamepad1.a) {
            pb.setMotorSpeed(speed);
        }

    }
}
