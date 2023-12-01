package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Autonomous(name = "AutoDriveRight", group = "Testing OpModes")
@Config
public class AutoDriveRight extends LinearOpMode {

    // Motor variable global declarations
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor slide = null;
    private DcMotor intake = null;
    private CRServo trapDoor = null;

    public static Double TURN = 80.0;

    private TfodProcessor tfod;

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    SkystoneDetector detector = new SkystoneDetector(width);


    OpenCvCamera camera;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    Encoder leftEncoder, rightEncoder, frontEncoder;

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }


    /**
     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

    @Override
    public void runOpMode() throws InterruptedException {

        // Initializes the AprilTag processor
        initAprilTag();

        // Creates the drive object form the SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightFront");
        slide = hardwareMap.get(DcMotor.class, "slide");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // TODO: reverse any motors using DcMotor.setDirection()
        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("leftRear").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightFront").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightRear").setDirection(DcMotor.Direction.FORWARD);

        /*
        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("leftRear").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("rightFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("rightRear").setDirection(DcMotor.Direction.REVERSE);
         */

        // Sets motors power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "actuator"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_Encoder"));


        /*========================================================================================================
         *
         *                  Object Detection
         *
         *========================================================================================================*/

        // Createwebport
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //open camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        // Add pipline to the camera
        camera.setPipeline(detector);

        SkystoneDetector.SkystoneLocation location = detector.getLocation();

        if (location != SkystoneDetector.SkystoneLocation.NONE) {
            telemetry.addLine("FOUND");
        } else {
            telemetry.addLine("NOT FOUND");
        }

        /*========================================================================================================*/

        // Sets Slide Motor run mode
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }


        /*========================================================================================================*/
        // Sets the starting position of the robot at (-35,-62) with a heading of 0 deg
        Pose2d startPose = new Pose2d(-35,62, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        // Prints the starting position of the robot to the driver station
        telemetry.addData("Starting Position", "%3f, %3f", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());

        // Trajectory 1
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, 35, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(45,3.3859947795680827, 15),SampleMecanumDrive.getAccelerationConstraint(45))
//                .addTemporalMarker(2.50, () -> {
//                    // This marker runs two seconds into the trajectory
//                    intake.setPower(1);
//                    // Run your action in here!
//                })
                .build();

        // Trajectory 2 TODO: Test for accuracy and fix cords
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-35, -35, 0))
                // Line to the second stopping point (Right beghind the of the stage frame)
                .lineToLinearHeading(new Pose2d(44,-35,0), SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(150), Math.toRadians(150)),SampleMecanumDrive.getAccelerationConstraint(26037.71398745077))
                .build();



        // Waits for the start button to be pressed
        waitForStart();

        // Updates the telemetry with the current position of the robot

        telemetry.addData("Current X", leftEncoder.getCurrentPosition());
        telemetry.addData("Current Y", frontEncoder.getCurrentPosition());
//        telemetry.addData("Current Heading", "%4f", drive.getExternalHeading() + "°");

        // Updates the telemetry
        telemetry.update();

        // Follow
        //drive.followTrajectory(traj1);

        visionPortal.close();

        // Checks if the stop button has been pressed
        // If it has, the robot will stop moving
        if(isStopRequested()) {
            return;
        }
    }
}
