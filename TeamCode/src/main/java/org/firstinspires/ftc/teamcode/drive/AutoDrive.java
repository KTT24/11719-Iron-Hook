package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoDrive", group = "Testing OpModes")
@Config
public class AutoDrive extends LinearOpMode {

    public static double MAX_VELOCITY = 45.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Creates the drive object form the SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // -46,-70 8.5
        // Sets the starting position of the robot
        Pose2d startPose = new Pose2d(-37.5, -62, 45);
        drive.setPoseEstimate(startPose);


        telemetry.addData("Starting Position", "%3f, %3f", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
        // Sets the direction of the motors
        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("leftRear").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("rightRear").setDirection(DcMotor.Direction.FORWARD);

        // Creates a trajectory for moving to stop 1
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, -35, 0), SampleMecanumDrive.getVelocityConstraint(MAX_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-37,-37,0))
                .forward(20)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .forward(30)
                .build();


        // Waits for the start button to be pressed
        waitForStart();

        // telemetry.addData("Current Pose", drive.getPoseEstimate());
        telemetry.addData("Current Position", "%7f, %7f", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
        telemetry.update();


        // Checks if the stop button has been pressed
        if(isStopRequested()) {
            return;
        }

        // Follows the trajectory
        drive.followTrajectory(traj2);
        sleep(1000);
        drive.turn(Math.toRadians(-55));
        sleep(1000);
        drive.followTrajectory(traj3);




        //drive.followTrajectory(tra1b);

    }
}
