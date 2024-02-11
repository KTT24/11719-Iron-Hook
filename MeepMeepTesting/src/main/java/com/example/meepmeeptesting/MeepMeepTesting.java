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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-37,-57, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(-33,-33, Math.toRadians(270)))
                        .addDisplacementMarker(() -> {
//                            // This marker runs after the first splineTo()
//                            telemetry.addLine("Place Purple Pixel");
//                            purplePixel.setPosition(1);
                            // Run your action in here
                        })
                        .waitSeconds(0.5)
                        .lineTo( new Vector2d(-37,-33))
                                .lineTo(new Vector2d(-37,-10))
                        .lineTo(new Vector2d(60, -10))
                        .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}