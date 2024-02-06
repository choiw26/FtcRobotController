//Created December 5th
//Program that makes the robot move in a square with a period of 12 seconds
//Programming Board as ProgrammingBoard_MecanumWheelsAutomation
//Doesn't work

package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.practice.mechanisms.ProgrammingBoard_mecanum_wheels;

@Autonomous
public class MecanumWheelsAutomation1 extends OpMode{

    ProgrammingBoard_mecanum_wheels board = new ProgrammingBoard_mecanum_wheels();
    double lastTime;
    boolean autoMove;

    double speed = 0.0;
    @Override
    public void init() {
        board.init(hardwareMap);
    }

    public void start() {
        resetRuntime();
        lastTime = getRuntime();
        speed = 0.0;
        //autoMove = True;
    }

    @Override
    public void loop () {

        /*if (!gamepad1.x) {
            autoMove = !autoMove;
            break //automated movement is true if x is held down -- Solely for testing.
        }
        if autoMove {

        }*/



        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in State", getRuntime() - lastTime);

        double Time = getRuntime()- lastTime;
        if ((Time % 4.0) <= 1.0) {
            board.setAllMotor(0.5);

        } else if (((Time % 4.0) <= 2.0) && ((Time % 4.0) > 1.0)) {
            board.setPower(0.5,-0.5,-0.5,0.5);//left

        } else if (((Time % 4.0) > 2.0) && ((Time % 4.0) <= 3.0)) {
            board.setAllMotor(-0.5);

        }else if ((Time % 4.0) > 3.0) {
            board.setPower(-0.5,0.5,0.5,-0.5);//right
        }
    }
}
