package org.firstinspires.ftc.teamcode.developingTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drivebase w/ Claw+Lift+Drone Test Code")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class TeleOpWithClawLiftDrone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");
        Servo clawServo = hardwareMap.servo.get("claw");

        // ADDED CODE - sets up lift motor and drone servo
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        Servo droneServo = hardwareMap.servo.get("drone");

        // Reverse the right side motors. Flip if goes backward.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ADDED CODE - sets directions of motor and servos (just in case)
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        droneServo.setDirection(Servo.Direction.FORWARD);

        clawServo.setDirection(Servo.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // ADDED CODE - sets servo's range and default position beforehand
        // WILL LIKELY NEED TO BE CHANGED AFTER TESTING
        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.85);

        clawServo.scaleRange(0, 0.85);
        clawServo.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // ADDED CODE - creates variables for right and left trigger values
            double rTrigger = gamepad1.right_trigger;
            double lTrigger = gamepad1.left_trigger;
            double droneServoPosition = droneServo.getPosition();
            double clawServoPosition = clawServo.getPosition();
            float clawPos = 1f;

            // ADDED CODE - holding down right trigger makes lift move up
            // and left trigger makes lift move down
            // depending on how much you press
            // while you aren't pressing anything, sets the motor power to zero
            if (rTrigger > 0) {
                lift.setPower(rTrigger);
            }else if (lTrigger > 0) {
                lift.setPower(-lTrigger);
            }else {
                lift.setPower(0);
            }

            // ADDED CODE - pressing button A moves servo to launch the drone and then reset launcher position
            // pressing button Y opens the claw and then closes it
            if (gamepad1.a) {
                droneServo.setPosition(1);
                sleep(1500);
                droneServo.setPosition(0.85);
            }

            if (gamepad1.y) {
                clawServo.setPosition(0);
                sleep(2000);
                clawServo.setPosition(1);
            }

            if (gamepad1.right_bumper && clawPos <= 0.9 && clawPos >= 0) {
                clawPos += 0.1;
                clawServo.setPosition(clawPos);
            } else if (gamepad1.left_bumper && clawPos <= 1 && clawPos <=0.1) {
                clawPos -= 0.1;
                clawServo.setPosition(clawPos);
            }

            // ADDED CODE - sends info about current servo position to driver station
            telemetry.addData("Servo Position: ", droneServoPosition);
            telemetry.update();

            telemetry.addData("Claw Position: ", clawServoPosition);
            telemetry.update();

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.x) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

        }
    }
}

