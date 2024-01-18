package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LeftDrive", group = "Testing OpModes")
public class LeftDrive extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor slide = null;
    private DcMotor intake = null;
    private CRServo trapDoor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Creates the drive object form the SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightFront");
        slide = hardwareMap.get(DcMotor.class, "slide");
        intake = hardwareMap.get(DcMotor.class, "intake");



        waitForStart();

        Pose2d startPose = new Pose2d(-62,35, Math.toRadians(0));

        // Drive forward for 1 inch
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-57,35, Math.toRadians(0)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(0 ,35, Math.toRadians(0)))
                .build();




        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(-65));
        drive.followTrajectory(traj3);


    }
}
