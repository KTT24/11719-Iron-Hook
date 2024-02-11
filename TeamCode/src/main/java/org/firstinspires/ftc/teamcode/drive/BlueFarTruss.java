package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "Blue Far Truss", group = "Final OpModes")
public class BlueFarTruss extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx slide = null;
    private DcMotor intake = null;
    private Servo trapDoor = null;
    private Servo purplePixel = null;
    private OpenCvCamera camera;

    private Encoder intakeEncoder = null;

    // Camera Values
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    ObjectDetectionBlue.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {


        // Creates the camera objects and pipeline objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ObjectDetectionBlue.SkystoneDeterminationPipeline();



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
        telemetry.setMsTransmissionInterval(50);

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

        // Initialize servos
        trapDoor = hardwareMap.get(Servo.class, "box");

        // Close trapdoor TODO: GET THE CLOSE POSITION FROM ROBERT
        trapDoor.setPosition(0.295);

        // Lock/Hold the purple pixel in position
        // purplePixel.setPosition(0.75);

// Displays the analysis result
        while (opModeInInit() && !isStarted()) {
            telemetry.addData(" ----------- Analysis ----------- \n", pipeline.getAnalysis());
            telemetry.update();
        }

        // =======================================================================================
        //                               TODO: REDO RIGHT PIXEL
        //========================================================================================

        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(new Pose2d(-39, 63.00, Math.toRadians(270)))
                .splineTo(new Vector2d(-40, 34.89), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    // Do some shit
                    purplePixel.setPosition(1.25);

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-35, 34.89, Math.toRadians(270)))
                .lineTo(new Vector2d(-34.75, 59))
                .lineTo(new Vector2d(12,59))
                .splineToLinearHeading(new Pose2d(37,37, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // April tag shit

                })
                .lineToLinearHeading(new Pose2d(47,29, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // Place yellow pxiel
                })
                .lineTo(new Vector2d(47,13))
                .lineTo(new Vector2d(58,13))
                .build();

        // =======================================================================================
        //                                   CENTER PIXEL
        //========================================================================================

        TrajectorySequence centerPixel = drive.trajectorySequenceBuilder(new Pose2d(-37.40, 63.00, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(-35.75, 33.89, Math.toRadians(180)), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    telemetry.addLine("Place Purple Pixel");
                    // Run your action in here!
                    purplePixel.setPosition(1);
                })
                .waitSeconds(0.5)
                //.lineToLinearHeading(new Pose2d(-46, 36.89, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-59, 11.53, Math.toRadians(0.00)), Math.toRadians(180.00))
                //.lineTo(new Vector2d(-59, 11))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    telemetry.addLine("Pick Up one white pixel");
                    // Run your action in here!

                })
                .waitSeconds(0.5)
                .lineTo(new Vector2d(23,11))
                .lineToLinearHeading(new Pose2d(47, 35, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    telemetry.addLine("Place the white pixel");
                    // Run your action in here!
                    slide.setTargetPosition(3000); // Set target position
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode to run to position
                    slide.setPower(1); // Slide up
                    sleep(1500);
                    trapDoor.setPosition(1.75); // Open trapdoor
                    sleep(1500);
                    trapDoor.setPosition(0); // Close trapdoor
                    slide.setTargetPosition(0); // Reset slide
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(43, 11, Math.toRadians(90)))
                .lineTo(new Vector2d(56, 0))
                .build();

        // =======================================================================================
        //                                     LEFT PIXEL
        //========================================================================================

        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(new Pose2d(-37.40, 63.00, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(-37, 32, Math.toRadians(270)), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    telemetry.addLine("Place Purple Pixel");
                    // Run your action in here!
                    purplePixel.setPosition(1);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-32, 32, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-32, 11.25, Math.toRadians(270)))

                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    telemetry.addLine("Pick Up one white pixel");
                    intake.setPower(-.5); // Starts intake
                })
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    telemetry.addLine("Pick Up one white pixel");
                })
                .waitSeconds(0.5)
                .lineTo(new Vector2d(23,11))
                .lineToLinearHeading(new Pose2d(47, 28, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    telemetry.addLine("Place the white pixel");
                    // Run your action in here!
                    slide.setTargetPosition(3000); // Set target position
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode to run to position
                    slide.setPower(1); // Slide up
                    sleep(1500);
                    trapDoor.setPosition(1.75); // Open trapdoor
                    sleep(1500);
                    trapDoor.setPosition(0); // Close trapdoor
                    slide.setTargetPosition(0); // Reset slide
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(47, 11, Math.toRadians(90)))
                .lineTo(new Vector2d(58, 7))
                .build();

        // =======================================================================================



        // TEMPORARY CODE FOR TESTING
//        drive.setPoseEstimate(leftPixel.start());
//        sleep(1000);
//        drive.followTrajectorySequence(leftPixel);

        // Wait for start button to be pressed
        waitForStart();

        // =======================================================================================
        //                              LEFT, RIGHT CENTER SWITCH CASE
        //                               TODO: RE-ENABLE SWITCH CASE
        // =======================================================================================

        switch (pipeline.getAnalysis()) {
            case LEFT:
                camera.closeCameraDevice();
                telemetry.update();
//                drive.setPoseEstimate(leftPixel.start());
//                drive.followTrajectorySequence(leftPixel);
                break;
            case CENTER:
                camera.closeCameraDevice();
//                drive.setPoseEstimate(centerPixel.start());
//                drive.followTrajectorySequence(centerPixel);
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


