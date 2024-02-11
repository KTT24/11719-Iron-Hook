package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "Blue Inside", group = "Blue Autonomous")
public class BlueInside extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx slide = null;
    private DcMotor intake = null;
    private Servo trapDoor = null;
    private Servo purplePixel = null;
    private OpenCvCamera camera;

    private CRServo intaker = null;

    // Camera Values
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

//    public void intakeOne() {
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setPower(-1); // Make the motor spin at full power
//        try {
//            intaker.setPower(-1);
//            Thread.sleep(7500); // Pause execution for 1/4 (kind)  of a second (250 milliseconds)
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        intake.setPower(0.0); // Stop the motor by setting the power to 0
//        intaker.setPower(0.0);
//
//    }

    ObjectDetectionBlue.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {


        // Creates the camera objects and pipeline objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ObjectDetectionBlue.SkystoneDeterminationPipeline();
        camera.setPipeline(pipeline);




        // Stars the camera object and displays it in FTC Dashboard
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 120);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // Set telemetry interval
        //telemetry.setMsTransmissionInterval(50);

        // Creates the drive object form the SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightFront");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        intake = hardwareMap.get(DcMotor.class, "intake");
        Servo purplePixel = hardwareMap.get(Servo.class, "purple_pixel");
        intaker = hardwareMap.get(CRServo.class, "intaker");


        // Initialize servos
        trapDoor = hardwareMap.get(Servo.class, "box");

        // Close trapdoor
        trapDoor.setPosition(.295);

        while (opModeInInit() && !isStarted()) {
            telemetry.addData(" ----------- Analysis ----------- \n", pipeline.getAnalysis());
            telemetry.update();
        }




        // =======================================================================================
        //                               TODO: REDO RIGHT PIXEL
        //========================================================================================


        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(new Pose2d(14, 63.00, Math.toRadians(270)))
                .lineTo(new Vector2d(14,35))
                .lineTo(new Vector2d(10,34))
                .addDisplacementMarker(() -> {
                    purplePixel.setPosition(1.25);
                    sleep(500);
                })
                .lineToLinearHeading(new Pose2d(23,34, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(55 , 34, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    slide.setTargetPosition(3000); // Set target position
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode to run to position
                    slide.setPower(1); // Slide up
                    sleep(3000);
                    slide.setPower(0);
                    sleep(1000);
                    trapDoor.setPosition(0.8); // Open trapdoor
                    sleep(500);
                    trapDoor.setPosition(0.275); // Close trapdoor
                    sleep(1000);
                    slide.setTargetPosition(0); // Reset slide
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    sleep(2800);
                })
                .lineTo(new Vector2d(49, 34))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(47,58, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(58,58, Math.toRadians(270)))
                .build();

        // =======================================================================================
        //                                   CENTER PIXEL
        //========================================================================================

        TrajectorySequence centerPixel = drive.trajectorySequenceBuilder(new Pose2d(14, 63.00, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(14,33, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    purplePixel.setPosition(1.25);
                    sleep(500);
                })
                .lineTo(new Vector2d(14,37))
                .lineToLinearHeading(new Pose2d(23,36, Math.toRadians(-5)))
                .lineToLinearHeading(new Pose2d(56 , 38, Math.toRadians(-2)))
                .addDisplacementMarker(() -> {
                    slide.setTargetPosition(3000); // Set target position
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode to run to position
                    slide.setPower(1); // Slide up
                    sleep(3000);
                    slide.setPower(0);
                    sleep(1000);
                    trapDoor.setPosition(0.8); // Open trapdoor
                    sleep(500);
                    trapDoor.setPosition(0.275); // Close trapdoor
                    sleep(1000);
                    slide.setTargetPosition(0); // Reset slide
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    sleep(2800);
                })
                .lineTo(new Vector2d(49, 34))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(47,58, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(58,58, Math.toRadians(270)))
                .build();

        // =======================================================================================
        //                                     LEFT PIXEL
        //========================================================================================

        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(new Pose2d(14, 63.00, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(16,34, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    purplePixel.setPosition(1.25);
                    sleep(500);
                })
                .lineTo(new Vector2d(14,37))
                .lineToLinearHeading(new Pose2d(14,63, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56 , 42, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    slide.setTargetPosition(3000); // Set target position
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode to run to position
                    slide.setPower(1); // Slide up
                    sleep(3000);
                    slide.setPower(0);
                    sleep(1000);
                    trapDoor.setPosition(0.8); // Open trapdoor
                    sleep(500);
                    trapDoor.setPosition(0.275); // Close trapdoor
                    sleep(1000);
                    slide.setTargetPosition(0); // Reset slide
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    sleep(2800);
                })
                .lineTo(new Vector2d(49, 34))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(47,61, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(58,61, Math.toRadians(270)))
                .build();

        // =======================================================================================

        waitForStart();

        // =======================================================================================
        //                              LEFT, RIGHT CENTER SWITCH CASE
        //                               TODO: RE-ENABLE SWITCH CASE
        // =======================================================================================

        // intakeOne();

        switch (pipeline.getAnalysis()) {
            case LEFT:
                camera.closeCameraDevice();
                telemetry.update();
                drive.setPoseEstimate(leftPixel.start());
                drive.followTrajectorySequence(leftPixel);
                break;
            case CENTER:
                camera.closeCameraDevice();
                drive.setPoseEstimate(centerPixel.start());
                drive.followTrajectorySequence(centerPixel);
                break;
            case RIGHT:
                camera.closeCameraDevice();
                drive.setPoseEstimate(rightPixel.start());
                drive.followTrajectorySequence(rightPixel);
                break;
        }

        // =======================================================================================
        if (isStopRequested()) return;
    }



}


