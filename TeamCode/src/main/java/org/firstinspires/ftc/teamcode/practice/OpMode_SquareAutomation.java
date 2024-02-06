// Created December 1st
//Making a robot move until it meets a wall (distance 30cm)
//Then the Robot turns 90 degrees to its right
//Then continues the cycle
//Uses ProgrammingBoard_SquareAutomation for Programming Board.

package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.practice.mechanisms.ProgrammingBoard_SquareAutomation_2;

@Autonomous
public class OpMode_SquareAutomation extends OpMode{
    enum State{
        MOVE,
        SPIN

    }
    ProgrammingBoard_SquareAutomation_2 board = new ProgrammingBoard_SquareAutomation_2();
    // Importing from Programming Board, set a variable board to import methods from the other Program
    IMU imu; // IMU for the ROTATION
    State state = State.MOVE;
    double lastTime;
    boolean autoMove;

    double speed = 0.0;
    @Override
    public void init() {
        board.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
        // Initialize the IMU
    }

    public void start() {
        state = State.MOVE;
        resetRuntime();
        lastTime = getRuntime();
    }

    @Override
    public void loop () {
        telemetry.addData("State", state);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in State", getRuntime() - lastTime); //to voever
        telemetry.addData("Distance left (CM)", board.getDistance1(DistanceUnit.CM));

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("Angle Spun", botHeading); // The angel of rotation.

        switch (state) {
            case MOVE:
                board.setAllMotor(0.3);
                if (board.getDistance1(DistanceUnit.CM) <= 100.0 ||
                        board.getDistance2(DistanceUnit.CM) <= 100.0) {
                    // if robot sees a wall less than 30cm away, stop and turn
                    board.setPower(0.0, 0.0, 0.0, 0.0);
                    state = State.SPIN;
                    imu.resetYaw();
                }
                break;
            case SPIN:
                board.setPower(-0.2, 0.2, -0.2, 0.2);
                if (Math.abs(botHeading) >= Math.PI / 2 || (board.getDistance1(DistanceUnit.CM)
                        >= 150.0 && board.getDistance1(DistanceUnit.CM) >= 150.0)) {
                    state = State.MOVE;
                }
        }
    }
}
