package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.practice.ProgrammingBoardNov29th;

@TeleOp
public class OpModeNov29th extends OpMode{
    ProgrammingBoardNov29th pb = new ProgrammingBoardNov29th();
    public void init() {
        pb.init(hardwareMap);
    }

    @Override
    public void loop() {
        double motorSpeed = gamepad1.right_trigger + gamepad1.left_trigger;

        telemetry.addData("Pot Angle", pb.getPotAngle());

        telemetry.addData("Amount Red", pb.getamountRed());
        telemetry.addData("Distance (CM)", pb.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (IN)", pb.getDistance(DistanceUnit.INCH));

        telemetry.addData("Our Heading", pb.getHeading(AngleUnit.DEGREES));

    }
}
