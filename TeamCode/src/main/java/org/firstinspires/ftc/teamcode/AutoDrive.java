/* Authors: Kutter Thornton */
/* Creation Date: 09/06/2023 */
/* Last Edited: 09/06/2023 */
/* Description: This is a test autonomous program that will drive the robot forward 40 inches twice. */



package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Robot: Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                // .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .forward(40)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                // .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .forward(40)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

        if(isStopRequested()) return;

    }


}
