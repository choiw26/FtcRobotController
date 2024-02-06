package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48180821614297, 52.48180821614297,
                        Math.toRadians(138.0064856897222), Math.toRadians(138.0064856897222)
                        , 16.97)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36,-66, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-36,0, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(54,-39, Math.toRadians(0)), Math.toRadians(0))
                                //

                                //
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
