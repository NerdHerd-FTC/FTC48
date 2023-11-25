package org.firstinspires.ftc.teamcode.Component_Testing_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

@TeleOp
public class _2023111701_Odometry_Pod_Test_Code extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor rightOdometryPod = null;
    private DcMotor leftOdometryPod = null;
    private DcMotor backOdometryPod = null;

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

        double speed = 1;

        int rightOdometryPodTicks = 0;
        int leftOdometryPodTicks = 0;
        int backOdometryPodTicks = 0;

        double rightOdometryPodInches = 0;
        double leftOdometryPodInches =  0;
        double backOdometryPodInches = 0;

        double robotHeading = 0;

        resetOdometryPods();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            rightOdometryPodTicks = rightOdometryPod.getCurrentPosition();
            leftOdometryPodTicks = leftOdometryPod.getCurrentPosition();
            backOdometryPodTicks = backOdometryPod.getCurrentPosition();

            rightOdometryPodInches = calculateOdometryInches(rightOdometryPodTicks);
            leftOdometryPodInches= calculateOdometryInches(leftOdometryPodTicks);
            backOdometryPodInches = calculateOdometryInches(backOdometryPodTicks);

            robotHeading = (rightOdometryPodInches-leftOdometryPodInches/12);

            telemetry.addData("Right Odometry Pod Measurement: ", "%d ticks", rightOdometryPodTicks);
            telemetry.addData("Left Odometry Pod Measurement: ", "%d ticks", leftOdometryPodTicks);
            telemetry.addData("Back Odometry Pod Measurement: ", "%d ticks", backOdometryPodTicks);

            telemetry.addData("Right Odometry Pod Inch: ", "%f inches", rightOdometryPodInches);
            telemetry.addData("Left Odometry Pod Inch: ", "%f inches", leftOdometryPodInches);
            telemetry.addData("Back Odometry Pod Inch: ", "%f inches", backOdometryPodInches);

            telemetry.addData("Yaw/Heading", robotHeading);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * speed);
            backLeftMotor.setPower(backLeftPower * speed);
            frontRightMotor.setPower(frontRightPower * speed);
            backRightMotor.setPower(backRightPower * speed);

            telemetry.update();
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
        double odometryInchMeasurement = (odometryTickMeasurementDecimal/2000) * (2 * Math.PI * 0.944882);
        return odometryInchMeasurement;
    }
}
