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
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(20)
                                .lineToLinearHeading(new Pose2d(27, -8, Math.toRadians(-45)))
                                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                                .back(20)
                                .strafeLeft(50)
                                .forward(20)
                                .lineToLinearHeading(new Pose2d(12.5, 36, Math.toRadians(90)))
                                .addTemporalMarker(() -> {
                                    //pixelPlacer.setPosition(PIXEL_PLACEMENT_END_POSITION);
                                }) // move servo to place pixel
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {
                                    //pixelPlacer.setPosition(PIXEL_PLACEMENT_START_POSITION);
                                }) // return servo to original position
                                .back(5)
                                .strafeLeft(15)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
//im sorry, i shouldn't have broke out like that.