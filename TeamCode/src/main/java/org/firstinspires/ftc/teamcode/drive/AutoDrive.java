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

    // public static double TURN = -50.0; Unused variable
    public static double SLIDE_SPEED = 0.5;

    public static double STARTING_X = -37.5;
    public static double STARTING_Y = -33;

    public static double ENDING_X = -35;
    public static double ENDING_Y = -35;
    public static int TRAJECTORY_TO_FOLLOW = 1;

    /*

    ================ Deprecated ================

    public void initOpMode() throws InterruptedException {
        // Prints the status of the robot which is "Initializing"
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        sleep(5000);

        // Prints the status of the robot which is "Initialized" and "Waiting for Start"
        telemetry.addData("Status", "Initialized", "Waiting for Start");
        telemetry.update();
    }

   ================ Deprecated ================


     */

    @Override
    public void runOpMode() throws InterruptedException {

        // Creates the drive object form the SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Sets the direction of the motors
        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("leftRear").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("rightRear").setDirection(DcMotor.Direction.FORWARD);

        // Sets the starting position of the robot
        Pose2d startPose = new Pose2d(-37, -62, 45);
        drive.setPoseEstimate(startPose);

        // Prints the starting position of the robot to the driver station
        telemetry.addData("Starting Position", "%3f, %3f", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());

        // Trajectory 1
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-37, -62, 45))
                .lineToLinearHeading(new Pose2d(-34, -34, 0), SampleMecanumDrive.getVelocityConstraint(MAX_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Trajectory 2
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-35, -35, 0))
                // .lineToLinearHeading(new Pose2d(STARTING_X, STARTING_Y, 45), SampleMecanumDrive.getVelocityConstraint(MAX_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(58,-60,0), SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(150), Math.toRadians(150)),SampleMecanumDrive.getAccelerationConstraint(26037.71398745077))
                .build();

        // Trajectory 3 TODO: FIX THIS NOW
        //        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(58,-60,0))
        //                .lineToLinearHeading(new Pose2d(58,-60,0), SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(150), Math.toRadians(150)),SampleMecanumDrive.getAccelerationConstraint(26037.71398745077))
        //                .build();

        // Waits for the start button to be pressed
        waitForStart();



        // Updates the telemetry with the current position of the robot
        telemetry.addData("Current Position", "%4f, %4f", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());

        // Updates the telemetry
        telemetry.update();

        drive.followTrajectory(traj1);

        // Checks if the stop button has been pressed
        // If it has, the robot will stop moving
        if(isStopRequested()) {
            return;
        }
    }
}
