package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

import java.util.List;

@Autonomous(name = "Back Board Right Side V1")
public class _20231121_Back_Board_Right_Side_Parking extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor rightOdometryPod = null;
    private DcMotor leftOdometryPod = null;
    private DcMotor backOdometryPod = null;

    int loop = 0;

    double speed = 0.50;
    double distanceToTargetForDecceleration = 0;

    int rightOdometryPodTicks = 0;
    int leftOdometryPodTicks = 0;
    int backOdometryPodTicks = 0;

    double rightOdometryPodInches = 0;
    double leftOdometryPodInches =  0;
    double backOdometryPodInches = 0;

    double robotHeading = 0;

    List<LynxModule> allHubs = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motorFL");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motorBL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motorFR");
        backRightMotor = hardwareMap.get(DcMotor.class, "motorBR");

        rightOdometryPod = hardwareMap.get(DcMotor.class, "motorFR");
        leftOdometryPod = hardwareMap.get(DcMotor.class, "motorFL");
        backOdometryPod = hardwareMap.get(DcMotor.class, "motorBR");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        resetOdometryPods();

        waitForStart();

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if (opModeIsActive()) {
            moveForward(speed, 24);
            telemetry.addData("Measured Distance at End: ", calculateOdometryInches(rightOdometryPod.getCurrentPosition()));
            telemetry.update();
            sleep(5000);
        }
    }

    public void resetOdometryPods() {
        rightOdometryPod.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometryPod.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdometryPod.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOdometryPod.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backOdometryPod.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backOdometryPod.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double calculateOdometryInches(int odometryTickMeasurement) {
        double odometryTickMeasurementDecimal = Double.valueOf(odometryTickMeasurement);
        double odometryInchMeasurement = (odometryTickMeasurementDecimal/2000) * (2 * Math.PI * 0.944882); //1.889764
        return odometryInchMeasurement;
    }

    public void SetFrontLeftDriveDirection(String direction) {
        if(direction == "forward") {
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction == "backward") {
            frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void SetFrontRightDriveDirection(String direction) {
        if(direction == "forward") {
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == "backward") {
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void SetBackLeftDriveDirection(String direction) {
        if(direction == "forward") {
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction == "backward") {
            backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void SetBackRightDriveDirection(String direction) {
        if(direction == "forward") {
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == "backward") {
            backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public void update() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        rightOdometryPodTicks = rightOdometryPod.getCurrentPosition();
        leftOdometryPodTicks = leftOdometryPod.getCurrentPosition();
        backOdometryPodTicks = backOdometryPod.getCurrentPosition();

        rightOdometryPodInches = calculateOdometryInches(rightOdometryPodTicks);
        leftOdometryPodInches= calculateOdometryInches(leftOdometryPodTicks);
        backOdometryPodInches = calculateOdometryInches(backOdometryPodTicks);

        loop = loop+1;
    }
    //Drive Functions
    public void moveForward(double speed, int inches) {

        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("forward");

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);

        loop = 0;

        while (true) {

            update();

            telemetry.addData("Loop: ", loop);

            telemetry.addData("Right Odometry Pod Measurement: ", "%d ticks", rightOdometryPodTicks);
            telemetry.addData("Left Odometry Pod Measurement: ", "%d ticks", leftOdometryPodTicks);
            telemetry.addData("Back Odometry Pod Measurement: ", "%d ticks", backOdometryPodTicks);

            telemetry.addData("Right Odometry Pod Inch: ", "%f inches", rightOdometryPodInches);
            telemetry.addData("Left Odometry Pod Inch: ", "%f inches", leftOdometryPodInches);
            telemetry.addData("Back Odometry Pod Inch: ", "%f inches", backOdometryPodInches);

            double odometryReadingAverage = (rightOdometryPodInches + leftOdometryPodInches)/2;

            telemetry.addData("Odometry Reading Average: ", odometryReadingAverage);
            telemetry.addData("Target Inches: ", inches);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Distance To Target For Decceleration: ", distanceToTargetForDecceleration);

            telemetry.update();


            if (odometryReadingAverage < (inches-distanceToTargetForDecceleration)) {

            } else {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                sleep(15000);
                break;
            }

        }
    }
}
