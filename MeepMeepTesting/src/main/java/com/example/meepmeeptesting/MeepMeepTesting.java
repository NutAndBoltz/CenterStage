package com.example.meepmeeptesting;

import  com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                                .forward(20)
                                .lineToLinearHeading(new Pose2d(27, 8, Math.toRadians(45)))
                                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                                .strafeRight(25)
                                .lineToLinearHeading(new Pose2d(48, -38, Math.toRadians(-90)))
                                .addTemporalMarker(() -> {
                                    //pixelPlacer.setPosition(.7);
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