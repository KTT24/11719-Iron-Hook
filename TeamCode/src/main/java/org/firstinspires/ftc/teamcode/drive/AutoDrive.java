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

    public static double DISTANCE1 = 1;
    public static double DISTANCE2 = 1;




    @Override
    public void runOpMode() throws InterruptedException {

        // Creates the drive object form the SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Sets the starting position of the robot
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Sets the direction of the motors
        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("leftRear").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("rightRear").setDirection(DcMotor.Direction.FORWARD);

        // Sets the power of the motors
//        hardwareMap.dcMotor.get("leftFront").setPower(POWER);
//        hardwareMap.dcMotor.get("rightFront").setPower(POWER);
//        hardwareMap.dcMotor.get("leftRear").setPower(POWER);
//        hardwareMap.dcMotor.get("rightRear").setPower(POWER);


        // Creates a trajectory for the robot to follow
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE1)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .back(DISTANCE2)
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
        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(traj2);

    }
}
