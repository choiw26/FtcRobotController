package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.practice.mechanisms.ProgrammingBoard_MecanumWheelsAutomation;

@Autonomous
public class MecanumWheelsAutomation extends OpMode{
    enum State{
        FRONT,
        BACK
    }
    ProgrammingBoard_MecanumWheelsAutomation board = new ProgrammingBoard_MecanumWheelsAutomation();
    State state = State.FRONT;
    double lastTime;
    boolean autoMove;

    double speed = 0.0;
    @Override
    public void init() {
        board.init(hardwareMap);
    }

    public void start() {
        state = State.FRONT;
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

        telemetry.addData("State", state);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in State", getRuntime() - lastTime);

        switch (state) {
            case FRONT:
                if ((getRuntime() % 6.0) <= 3.0) {
                    board.setAllMotor(0.5);
                } else {
                    state = State.BACK;
                }
            case BACK:
                if ((getRuntime() % 6.0) > 3.0) {
                    board.setAllMotor(-0.5);
                } else {
                    state = State.FRONT;
                }
        }


    }
}
