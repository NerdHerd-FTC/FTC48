package org.firstinspires.ftc.teamcode.Component_Testing_Code;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

import java.util.List;

@TeleOp
public class _20231122_Odometry_Pod_Roll_Test extends LinearOpMode {

    private DcMotor rightOdometryPod = null;
    private DcMotor leftOdometryPod = null;
    private DcMotor backOdometryPod = null;

    @Override
    public void runOpMode() throws InterruptedException {

        rightOdometryPod = hardwareMap.get(DcMotor.class, "motorFR");
        leftOdometryPod = hardwareMap.get(DcMotor.class, "motorFL");
        backOdometryPod = hardwareMap.get(DcMotor.class, "motorBR");



        int rightOdometryPodTicks = 0;
        int leftOdometryPodTicks = 0;
        int backOdometryPodTicks = 0;

        double rightOdometryPodInches = 0;
        double leftOdometryPodInches =  0;
        double backOdometryPodInches = 0;

        resetOdometryPods();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            rightOdometryPodTicks = rightOdometryPod.getCurrentPosition();
            leftOdometryPodTicks = leftOdometryPod.getCurrentPosition();
            backOdometryPodTicks = backOdometryPod.getCurrentPosition();

            rightOdometryPodInches = calculateOdometryInches(rightOdometryPodTicks);
            leftOdometryPodInches= calculateOdometryInches(leftOdometryPodTicks);
            backOdometryPodInches = calculateOdometryInches(backOdometryPodTicks);

            telemetry.addData("Right Odometry Pod Measurement: ", "%d ticks", rightOdometryPodTicks);
            telemetry.addData("Left Odometry Pod Measurement: ", "%d ticks", leftOdometryPodTicks);
            telemetry.addData("Back Odometry Pod Measurement: ", "%d ticks", backOdometryPodTicks);

            telemetry.addData("Right Odometry Pod Inch: ", "%f inches", rightOdometryPodInches);
            telemetry.addData("Left Odometry Pod Inch: ", "%f inches", leftOdometryPodInches);
            telemetry.addData("Back Odometry Pod Inch: ", "%f inches", backOdometryPodInches);

            telemetry.update();
        }
    }

    public void resetOdometryPods() {
        rightOdometryPod.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometryPod.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        leftOdometryPod.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOdometryPod.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        backOdometryPod.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backOdometryPod.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

    }

    public double calculateOdometryInches(int odometryTickMeasurement) {
        double odometryTickMeasurementDecimal = Double.valueOf(odometryTickMeasurement);
        double odometryInchMeasurement = (odometryTickMeasurementDecimal/2000) * (2 * Math.PI * 0.944882);
        return odometryInchMeasurement;
    }
}
