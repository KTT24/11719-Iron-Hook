package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;




@TeleOp(name="TELEOP 2023 FINAL")
    public class Teleop extends OpMode {
    /* Declare OpMode members. */
    private double slowM = .7;
    private DcMotor lf = null;
    private DcMotor lr = null;
    private DcMotor rf = null;
    private DcMotor rr = null;
    private DcMotor intake = null;
    private DcMotor slide = null;
    //private CRServo door = null;
    private CRServo box = null;

    //public static final double MID_SERVO   =  0.5 ;
    //public static final double CLAW_SPEED  = 0.02 ;

    //declares rumble things
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime boxtime = new ElapsedTime();


    BNO055IMU imu = null;
    double lastZ = 0;
    int zTurns = 0;
    double face = 0;

    double armSpeed = .85;
    double clawOffset = 0;


    @Override
    public void init()
    {

        telemetry.addData("Status", "Initialized");
        lf = hardwareMap.get(DcMotor.class, "leftFront");
        lr = hardwareMap.get(DcMotor.class, "leftRear");
        rf = hardwareMap.get(DcMotor.class, "rightRear");
        rr = hardwareMap.get(DcMotor.class, "rightFront");
        slide = hardwareMap.get(DcMotor.class, "slide");
        intake = hardwareMap.get(DcMotor.class, "intake");
        //door = hardwareMap.get(CRServo.class, "box");


        box = hardwareMap.get(CRServo.class, "box");

        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("leftRear").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("rightFront").setDirection(DcMotor.Direction.FORWARD);
        hardwareMap.dcMotor.get("rightRear").setDirection(DcMotor.Direction.FORWARD);

        // hardwareMap.dcMotor.get("intake").setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }




    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();





    }


    public void loop() {

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        lf.setPower(leftFrontPower);
        rf.setPower(rightFrontPower);
        lr.setPower(leftBackPower);
        rr.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        // telemetry.addData("POS", "%4.2f", door.getPosition());
        telemetry.update();

        // Set intake speed to gamepad 2 left trigger
        intake.setPower(gamepad2.left_trigger);



        if (gamepad2.dpad_up == true) {
            slide.setTargetPosition(-3000);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        } else if (gamepad2.dpad_down == true) {
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);

        }



        // Use gamepad left & right Bumpers to open and close the claw
        if(gamepad2.a == true)
        {
            slide.setTargetPosition(500);
        }
        if (gamepad2.b == true )
        {
            box.setPower(1);
        } else {

            box.setPower(0);
        }
        // else if (gamepad1.left_bumper)
        //     door.setPower(1);

        // Move both servos to new position.  Assume servos are mirror image of each other.




        // intake.setPower(-gamepad1.right_trigger);

    }


    public double heading()

    {
        double z = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //telemetry.addData("lastZ", lastZ);
        //telemetry.addData("zTurns", zTurns);
        telemetry.update();
        if (z < -140 && lastZ > 140) zTurns++;
        if (z > 140 && lastZ < -140) zTurns--;
        lastZ = z;
        return z + zTurns*360;
    }

    public double nearestHeading(double current, double target)
    {
        return 360 * Math.round((current-target)/360) + target;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }
}


