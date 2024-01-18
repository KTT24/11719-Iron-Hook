package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "RightDrive", group = "Testing OpModes")
public class RightDrive extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotorEx slide = null;
    private DcMotor intake = null;
    private Servo trapDoor = null;

    private OpenCvCamera camera;
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;


    @Override
    public void runOpMode() throws InterruptedException {

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(camera, 120);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);

        // Creates the drive object form the SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightFront");
        //slide = hardwareMap.get(DcMotorEx.class, "slide");
        //intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize servos
        //trapDoor = hardwareMap.get(Servo.class, "box");

        // TODO: reverse any motors using DcMotor.setDirection()
        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("leftRear").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightFront").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightRear").setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        // Sets the starting position of the robot
        Pose2d startPose = new Pose2d(-35,60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, 35, Math.toRadians(0)))
                .build();

//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .lineToSplineHeading(new Pose2d(35, -35, Math.toRadians(0)))
//                .addDisplacementMarker(85, () -> {
//                    telemetry.addLine("Time to place pixels");
//                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slide.setTargetPosition(3000);
//                    trapDoor.setPosition(0);
//                })
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .lineToSplineHeading(new Pose2d(40, -60, Math.toRadians(90)))
//                .build();

        // Tell the robot to drive to the specified position
        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);

        telemetry.addLine("Done");
        telemetry.update();

        if (isStopRequested()) return;
    }
}
