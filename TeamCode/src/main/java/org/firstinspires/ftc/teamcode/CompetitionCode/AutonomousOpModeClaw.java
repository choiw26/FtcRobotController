package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms.PbClaw;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;


@Autonomous
public class AutonomousOpModeClaw extends LinearOpMode{
    @Override
    public void runOpMode() {
    //Define all the Programming Boards
    PbClaw pbGrab = new PbClaw();
    pbGrab.init(hardwareMap);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    ElapsedTime runtime = new ElapsedTime();






    int SpikeMark = 0;


    Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-36,-63, Math.toRadians(90)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-36,-12, Math.toRadians(0)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(36,-12, Math.toRadians(-90)), Math.toRadians(-90))
            .splineToSplineHeading(new Pose2d(36,-36, Math.toRadians(0)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(48,-36, Math.toRadians(0)), Math.toRadians(0))
            //

            //
            .build();

    Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(48,-36, Math.toRadians(0)), Math.toRadians(0))
            .strafeLeft(40)
            .build();




        waitForStart();
        runtime.reset();
        if(isStopRequested()) return;

        //
        pbGrab.RotateGrab();
        pbGrab.ClawClose();
        //

        //
        pbGrab.RotatePlace();
        pbGrab.ViperSlideSetPosition(10, 0.5);
        while (pbGrab.viperSlideLeft.isBusy() && pbGrab.viperSlideRight.isBusy()) { }

        pbGrab.ClawOpen();
        //
        //
        pbGrab.ClawClose();

        pbGrab.ViperSlideSetPosition(0, 0.5);
        while (pbGrab.viperSlideLeft.isBusy() && pbGrab.viperSlideRight.isBusy()) { }
        pbGrab.RotateGrab();

        //
        while (opModeIsActive()) {
            double clawServoPosition = pbGrab.ClawServo.getPosition();
            telemetry.addData("Claw Servo Position : ", clawServoPosition);
            //
            double rotateServoPosition = pbGrab.RotateServo.getPosition();
            telemetry.addData("Rotate Servo Position : ", rotateServoPosition);

            telemetry.update();
        }
    }
        /*
    --> Autonomous driving is based on movement of the four wheel motors.
    The goals for Autonomous for programmers were Localization and Adjusting error

    Initially we used robot oriented running which I created.
    However it had too much errors, and often crashed.

    So we shifted to field centric movement, so the robot moves based on the coordinate of the game field
    To further increase our precision, we used a PIDF controller to adjust the position

    the movement of linear slide and the rotate motor is based on encoder reading.
    We used getPosition and setPosition method to increase precision against gravity.
    */


}


