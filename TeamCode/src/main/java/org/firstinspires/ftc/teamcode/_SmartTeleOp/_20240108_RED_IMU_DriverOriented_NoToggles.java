package org.firstinspires.ftc.teamcode._SmartTeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.DroneLauncherInstance;

@TeleOp(name = "0 RED IMU Driver Oriented - No Toggles")

public class _20240108_RED_IMU_DriverOriented_NoToggles extends LinearOpMode {

    private int Arm_Adjustment_Value = 50;

    private double Driving_Speed = 0.85;
    double armSpeed = 0.15;
    @Override
    public void runOpMode() throws InterruptedException {
        ArmInstance Arm = new ArmInstance();
        ClawInstance Claw = new ClawInstance();
        DroneLauncherInstance DroneLauncher = new DroneLauncherInstance();

        Arm.initializeArm(hardwareMap);
        Claw.initializeClaw(hardwareMap);
        DroneLauncher.initializeDroneLauncher(hardwareMap);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB backward
        imu.initialize(parameters);

        boolean armDown = true;

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = Math.round(((y + x + rx) / denominator));
            double backLeftPower = Math.round(((y - x + rx) / denominator));
            double frontRightPower = Math.round(((y - x - rx) / denominator));
            double backRightPower = Math.round(((y + x - rx) / denominator));

            frontLeftMotor.setPower(frontLeftPower * Driving_Speed);
            backLeftMotor.setPower(backLeftPower * Driving_Speed);
            frontRightMotor.setPower(frontRightPower * Driving_Speed);
            backRightMotor.setPower(backRightPower * Driving_Speed);

            //MecanumDrivebase.setDrivePower(new Pose2d(x, x, RobotHeading));

            if (gamepad1.y) {
                Claw.Actuate_Claw_Top_Finger("toggle");
            }
            if (gamepad1.a) {
                Claw.Actuate_Claw_Bottom_Finger("toggle");
            }

            if (gamepad1.right_bumper) {
                DroneLauncher.launchDrone();
            }

            if ((Arm.Arm_Motor.getCurrentPosition() > 530)) {
                Arm.setArmPosTo(525, 0.1);
            }
            if (Arm.Arm_Motor.getCurrentPosition() < 5) {
                Arm.setArmPosTo(5, 0.1);
            }

            //Smart TeleOp

            /*if (gamepad1.a){
                armSpeed = 0.2;
            }
            else if(gamepad1.b){
                armSpeed = 0.15;
            }
             */

            if (gamepad1.b) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Arm.setArmPosTo(525, armSpeed);
            }

            if (gamepad1.right_trigger > 0) {
                Claw.Actuate_Claw_Bottom_Finger("open");
                sleep(850);

                Driving_Speed = 0.25;
                backLeftMotor.setPower(Driving_Speed);
                backRightMotor.setPower(-Driving_Speed);
                frontLeftMotor.setPower(-Driving_Speed);
                frontRightMotor.setPower(Driving_Speed);
                sleep(800);

                Driving_Speed = 0.85;
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);

                Claw.Actuate_Claw_Top_Finger("open");
            }
// genshin uid: 642041765
// add me pls !!
            if (gamepad1.x) {
                armDown = false;
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Arm.setArmPosTo(100, 0.15);
            }

            if (gamepad1.left_trigger > 0) {
                Driving_Speed = 0.1;
                armDown = true;
                Claw.Actuate_Claw_Bottom_Finger("open");
                Claw.Actuate_Claw_Top_Finger("open");
                sleep(700);
                Arm.setArmPosTo(5, 0.15);
                while (Arm.Arm_Motor.isBusy()) {}
                backLeftMotor.setPower(-Driving_Speed);
                backRightMotor.setPower(-Driving_Speed);
                frontLeftMotor.setPower(-Driving_Speed);
                frontRightMotor.setPower(-Driving_Speed);

                sleep(350);

                Arm.setArmPosTo(5, 0.15);

                while (!Arm.Arm_Motor.isBusy()) {
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    break;
                }

                Driving_Speed = 0.85;
            }

            if (gamepad1.dpad_up) {
                Driving_Speed = 0.1;
                backLeftMotor.setPower(-Driving_Speed);
                backRightMotor.setPower(-Driving_Speed);
                frontLeftMotor.setPower(-Driving_Speed);
                frontRightMotor.setPower(-Driving_Speed);

                sleep(1000);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);

                Driving_Speed = 0.85;
            }

            if (!Arm.Arm_Motor.isBusy() && armDown) {
                armDown = false;
                sleep(50);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(150);
                Arm.setArmPosTo(100, 0.15);
            }

            Arm.setArmPosTo(Arm.getCurrentArmPos(), armSpeed);

            telemetry.addData("Arm is busy: ", Arm.Arm_Motor.isBusy());
            telemetry.addData("Arm Position: ", Arm.Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm Target Position: ", Arm.Arm_Motor.getTargetPosition());
            telemetry.addData("Top Claw Position: ", Claw.Claw_Top_Finger.getPosition());
            telemetry.addData("Bottom Claw Position: ", Claw.Claw_Bottom_Finger.getPosition());
            telemetry.addData("Drone Launcher Position: ", DroneLauncher.DroneLauncherServo.getPosition());
            telemetry.addLine("");
            telemetry.addLine("Genshin UID: 642041765");
            telemetry.update();
        }
    }
}
