package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Inside", group = "Red OpModes")
public class RedInside extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotorEx slide = null;
    private DcMotorEx intake = null;
    private Servo trapDoor = null;
    private Servo purplePixel = null;

    private OpenCvCamera camera;
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    ObjectDetectionRed.SkystoneDeterminationPipeline pipeline;

    public void intakeOne() {
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime intakeTime = new ElapsedTime();
        intake.setPower(-1); // Make the motor spin at full power
        try {
            Thread.sleep(333); // Pause execution for 1/4 of a second (250 milliseconds)
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        intake.setPower(0.0); // Stop the motor by setting the power to 0

    }

    @Override
    public void runOpMode() throws InterruptedException {


        // Creates the camera objects and pipeline objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ObjectDetectionRed.SkystoneDeterminationPipeline();
        camera.setPipeline(pipeline);

        // Stars the camera object and displays it in FTC Dashboard
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 120);
            }

            @Override
            public void onError(int errorCode) {

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
        rightRear = hardwareMap.get(DcMotor.class, "rightFront");

        // Initialize motors for intake and slide
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Initialize servos
        trapDoor = hardwareMap.get(Servo.class, "box");
        purplePixel =hardwareMap.get(Servo.class, "purple_pixel");

        // Create the runtime object
        ElapsedTime runtime = new ElapsedTime();

        hardwareMap.dcMotor.get("intake").setDirection(DcMotorSimple.Direction.FORWARD);

        // Close trapdoor
        trapDoor.setPosition(.295);

        // Displays the analysis result
        while (opModeInInit() && !isStarted()) {
            telemetry.addData(" ----------- Analysis ----------- \n", pipeline.getAnalysis());
            telemetry.update();
        }

        // =======================================================================================
        //                               REDO RIGHT PIXEL
        //=======================================================================================

        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(new Pose2d(14, -63.00, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(15,-33, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                            telemetry.addLine("Place Purple Pixel");
                            // Run your action in here
                            purplePixel.setPosition(1);
                            sleep(500);
                })
                .lineTo(new Vector2d(11,-33))
                .lineTo(new Vector2d(14, -58))
                .lineToLinearHeading(new Pose2d(35,-58, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(53.5 , -40, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
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
                .waitSeconds(0.5)
                .lineTo(new Vector2d(34,-36))
//                .lineToLinearHeading(new Pose2d(43, -11, Math.toRadians(90)))
                .lineTo(new Vector2d(34, -59))
                .lineToLinearHeading(new Pose2d(58,-59, Math.toRadians(90)))
                .build();

        // =======================================================================================
        //                                   CENTER PIXEL
        //========================================================================================

        TrajectorySequence centerPixel = drive.trajectorySequenceBuilder(new Pose2d(14, -63, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(14,-31, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    purplePixel.setPosition(1.25);
                    sleep(500);
                })
                .lineTo(new Vector2d(14,-34))
                .lineToLinearHeading(new Pose2d(23,-36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56 , -38, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    slide.setTargetPosition(3000); // Set target position
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode to run to position
                    slide.setPower(1); // Slide up
                    sleep(3000);
                    slide.setPower(0);
                    sleep(1000);
                    trapDoor.setPosition(0.8); // Open trapdoor
                    sleep(1000);
                    trapDoor.setPosition(0.275); // Close trapdoor
                    sleep(1000);
                    slide.setTargetPosition(0); // Reset slide
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    sleep(2800);
                })
                .lineTo(new Vector2d(49, -34))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(47,-58, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58,-58, Math.toRadians(90)))
                .build();
        // =======================================================================================
        //                                     LEFT PIXEL
        //========================================================================================

        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(new Pose2d(14, -63.00, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(11,-34, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                            purplePixel.setPosition(1.25);
                            sleep(500);
                })
                .lineTo(new Vector2d(14,-34))
                .lineToLinearHeading(new Pose2d(14,-58, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(53.5, -31, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                            slide.setTargetPosition(3000); // Set target position
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode to run to position
                            slide.setPower(1); // Slide up
                            sleep(3000);
                            slide.setPower(0);
                            sleep(3000);
                            trapDoor.setPosition(0.8); // Open trapdoor
                            sleep(500);
                            trapDoor.setPosition(0.275); // Close trapdoor
                            sleep(1000);
                            slide.setTargetPosition(0); // Reset slide
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide.setPower(1);
                            sleep(2800);
                })
                .lineTo(new Vector2d(49, -34))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(49,-58, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(58,-58, Math.toRadians(90)))
                .build();

        // =======================================================================================
        //                              LEFT, RIGHT CENTER SWITCH CASE
        // =======================================================================================

        waitForStart();

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


