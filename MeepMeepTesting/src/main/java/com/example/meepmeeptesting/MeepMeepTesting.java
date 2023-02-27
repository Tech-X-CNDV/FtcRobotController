package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                                .forward(55)
                                .back(6)
                                .lineToSplineHeading(new Pose2d(26.5, -2.5, Math.toRadians(135)))
                                .back(5)
                                .splineToSplineHeading(new Pose2d(55, -12, Math.toRadians(-0)),0)
                                .forward(10)
                                .back(10)
                                .splineToSplineHeading(new Pose2d(26.5, -2.5, Math.toRadians(135)), Math.toRadians(135))
                                /*
                                .forward(55)
                                .back(6)
                                .lineToSplineHeading(new Pose2d(26.5, -2.5, Math.toRadians(135)))
                                .back(5)
                                .lineToSplineHeading(new Pose2d(-40, -11.5, Math.toRadians(180)))
                                //.splineToSplineHeading(new Pose2d(-55, -12, Math.toRadians(180)),0)
                                .forward(24)
                                .back(20)
                                .lineToSplineHeading(new Pose2d(-30.5, -6.5, Math.toRadians(45)))

                                 */
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}