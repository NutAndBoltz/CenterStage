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
                        drive.trajectorySequenceBuilder(new Pose2d(9, -63, Math.toRadians(90)))
                                .forward(20)
                                .lineToLinearHeading(new Pose2d(17, -36, Math.toRadians(45)))
                                .lineToLinearHeading(new Pose2d(9, -43, Math.toRadians(90)))
                                .strafeRight(25)
                                .lineToLinearHeading(new Pose2d(50, -33, Math.toRadians(0)))
                                //.splineTo(new Vector2d(50, -33), Math.toRadians(-90))
                                //.setReversed(true)
                                //.splineToLinearHeading(new Pose2d(50, -33, Math.toRadians(90)), Math.toRadians(0))
                                //.setReversed(false)
                                //.strafeRight(33)
                                .addTemporalMarker(() -> {
                                    //servo.setPosition(0)
                                }) // Lower servo
                                .waitSeconds(3)
                                .addTemporalMarker(() -> {
                                    //servo.setPosition(1)
                                }) // Raise servo
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}