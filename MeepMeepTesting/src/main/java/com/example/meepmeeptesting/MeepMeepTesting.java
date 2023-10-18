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
                .setConstraints(26037.71398745077, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-61, -61, Math.toRadians(45)))
                                /* =================================================== */
                                /*                    TRAJECTORY 1a                    */
                                /*      Head to the back wall from starting spot       */
                                /* =================================================== */
                                .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(0)))
                                .forward(17)
//                                .turn(Math.toRadians(-90))
//                                .forward(100)

                                /* =================================================== */
                                /*                    TRAJECTORY 1b                    */
                                /*      Head to the back wall from starting spot       */
                                /* =================================================== */
                                /*.forward(10)
                                .turn(Math.toRadians(45))
                                .forward(50)
                                .turn(Math.toRadians(-90))
                                .forward(90)
                                .turn(Math.toRadians(-90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(12)*/
                                // .getPixel() .getPixel() Is a custom method that will be created in the future
                                // .placePixel(double riseHeight) Is a custom method that will be created in the future

                                /* =================================================== */
                                /*                     TRAJECTORY 2                    */
                                /*             Hanging from the stage frame            */
                                /* =================================================== */
                                /*.strafeTo(new Vector2d(-34, -34))
                                .turn(Math.toRadians(-45))
                                .lineTo(new Vector2d(-10, -34))*/

                                // Build the trajectory
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
