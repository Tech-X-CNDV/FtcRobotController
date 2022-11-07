package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                                .forward(10) //dupa vine scanare Signal
                                .splineTo(new Vector2d(27,-4.5), Math.toRadians(135)) // lasat con
                                .back(10)
                                .turn(Math.toRadians(-45))
                                /*.forward(26) //prindere con nou
                                .back(12)
                                .turn(Math.toRadians(-90)) //de aici merge pentru cap #2
                                .forward(5)//lasare con nou
                                .back(5)*/
                                //pt caz 1
                                /*.back(47)
                                .turn(Math.toRadians(-45))*/
                                //pt caz 2
                                /*.back(25)
                                .turn(Math.toRadians(-135))*/
                                //pt caz 3

                                //.strafeRight(20)
                                //.forward(28)
                               // .splineTo(new Vector2d(10,-34), Math.toRadians(180))


                                .build()


                                //dupa asta vine parcarea prin strafing
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}