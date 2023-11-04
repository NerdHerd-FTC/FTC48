/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Meet_01.Autonomous.Developing_Code.Autonomous_Place_Pixel_At_Spike_Zone_With_Tensorflow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Optional;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Autonomous Code For Spike Zone & Parking Forward Left")
public class _20231028_Kavi_Gupta_Tensorflow_Autonomous_Test extends LinearOpMode {

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private DcMotor lift = null;

    private double Ticks_Per_Inch = 45.2763982107824;

    private double Ticks_Per_Rotational_Degree = 12.1111111111;

    private double baseAutonomousSpeed = 0.75;

    private int baseRestTime = 750;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;



    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBR");
        lift = hardwareMap.get(DcMotor.class, "arm");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);


        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoders();

        initTfod();

        waitForStart();


        if (opModeIsActive()) {
            moveArm(0.3, 375);
            moveForward(22);
            sleep(2000);
            if (detectProps()) {
                telemetry.addLine("Forward Prop Detected");
                telemetry.update();
                moveForward(13);
                moveBackward(10);
                moveRight(40);
            } else {
                moveRight(6);
                rotateCounterClockwise(80);
                telemetry.addLine("Left Prop Detected");
                telemetry.update();
                sleep(2000);
                 if (detectProps()) {
                     moveForward(16);
                     moveBackward(50);
                 } else {
                     rotateClockwise(180);
                     moveBackward(5);
                     sleep(2000);
                     if (detectProps()) {
                         telemetry.addLine("Right Prop Detected");
                         telemetry.update();
                         moveForward(13);
                         moveBackward(15);
                         moveRight(20);
                         moveForward(45);
                     }
                 }
            }






        }
    }

    //"Building Blocks" Code

        //Code to create flexible drives

    public void moveArm(double speed, int ticks) {
        lift.setPower(speed);
        lift.setTargetPosition(ticks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {}
    }

    public void SetFrontLeftDriveDirection(String direction) {
        if(direction == "forward") {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction == "backward") {
            leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void SetFrontRightDriveDirection(String direction) {
        if(direction == "forward") {
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == "backward") {
            rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void SetBackLeftDriveDirection(String direction) {
        if(direction == "forward") {
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction == "backward") {
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void SetBackRightDriveDirection(String direction) {
        if(direction == "forward") {
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == "backward") {
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void runLeftFrontDrive(double speed, int ticks) {
        leftFrontDrive.setTargetPosition(ticks);
        leftFrontDrive.setPower(speed);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runRightFrontDrive(double speed, int ticks) {
        rightFrontDrive.setTargetPosition(ticks);
        rightFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runLeftBackDrive(double speed, int ticks) {
        leftBackDrive.setTargetPosition(ticks);
        leftBackDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runRightBackDrive(double speed, int ticks) {
        rightBackDrive.setTargetPosition(ticks);
        rightBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int GetAverageEncoderPositions() {
        int Average_Ticks = (leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 4;
        return Average_Ticks;
    }

        //Easy Encoder Reset
    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

        //Drive Functions
    public void moveForward(double speed, int inches) {

        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("forward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void moveForward(int inches) {
        moveForward(baseAutonomousSpeed, inches);
    }

    public void moveBackward(double speed, int inches) {

        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontLeftDriveDirection("backward");
        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");
        SetBackRightDriveDirection("backward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void moveBackward(int inches) {
        moveBackward(baseAutonomousSpeed, inches);
    }


    public void moveLeft(double speed, int inches) {
        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontLeftDriveDirection("backward");
        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("backward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void moveLeft(int inches) {
       moveLeft(baseAutonomousSpeed, inches);
    }

    public void moveRight(double speed, int inches) {
        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");
        SetBackRightDriveDirection("forward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void moveRight(int inches) {
        moveRight(baseAutonomousSpeed, inches);
    }

    public void rotateClockwise(double speed, double degrees) {
        resetEncoders();


        int Rounded_Encoder_Ticks = calculateTicksForRotationalMovement(degrees);

        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("backward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void rotateClockwise(double degrees) {
        rotateClockwise(baseAutonomousSpeed, degrees);
    }


    public void rotateCounterClockwise(double speed, double degrees) {
        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForRotationalMovement(degrees);

        SetFrontLeftDriveDirection("backward");
        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("backward");
        SetBackRightDriveDirection("forward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void rotateCounterClockwise(double degrees) {
        rotateCounterClockwise(baseAutonomousSpeed, degrees);
    }

    public void strafeForwardLeft(double speed, double inches) {
        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void strafeForwardLeft(double inches) {
        strafeForwardLeft(baseAutonomousSpeed, inches);
    }

    public void strafeForwardRight(double speed, double inches) {
        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontLeftDriveDirection("forward");
        SetBackRightDriveDirection("forward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void strafeForwardRight(double inches) {
        strafeForwardRight(baseAutonomousSpeed, inches);
    }

    public void strafeBackwardLeft(double speed, double inches) {
        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontLeftDriveDirection("backward");
        SetBackRightDriveDirection("backward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void strafeBackwardLeft(double inches) {
        strafeBackwardLeft(baseAutonomousSpeed, inches);
    }

    public void strafeBackwardRight(double speed, double inches) {
        resetEncoders();

        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        sleep(baseRestTime);
    }
    public void strafeBackwardRight(double inches) {
        strafeBackwardRight(baseAutonomousSpeed, inches);
    }


    public int calculateTicksForLateralMovement(int inches) {
        int Current_Encoder_Ticks = GetAverageEncoderPositions();

        double Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Rounded_Encoder_Ticks;
    }

    public int calculateTicksForDiagonalStrafingMovement(double inches) {
        double Calculated_Ticks_For_Diagonal_Strafing = Math.sqrt((inches*inches)/2);
        int Rounded_Ticks_For_Diagonal_Strafing = (int)Math.round(Calculated_Ticks_For_Diagonal_Strafing);
        return Rounded_Ticks_For_Diagonal_Strafing;
    }

    public int calculateTicksForRotationalMovement(double degrees) {
        double Calculated_Ticks_For_Rotational_Movement = Ticks_Per_Rotational_Degree * degrees;
        int Rounded_Ticks_For_Rotational_Movement = (int)Math.round(Calculated_Ticks_For_Rotational_Movement);
        return Rounded_Ticks_For_Rotational_Movement;
    }
    public void motionTelemetry() {
        telemetry.addData("Front Left Target Position",leftFrontDrive.getTargetPosition());
        telemetry.addData("Front Left Current Position Position",leftFrontDrive.getCurrentPosition());

        telemetry.addData("Front Right Target Position",rightFrontDrive.getTargetPosition());
        telemetry.addData("Front Right Current Position Position",rightFrontDrive.getCurrentPosition());

        telemetry.addData("Back Left Target Position",leftBackDrive.getTargetPosition());
        telemetry.addData("Back Left Current Position Position",leftBackDrive.getCurrentPosition());

        telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
        telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

        telemetry.update();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

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
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    //Tensorflow
    public boolean detectProps() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        boolean propDetected = Boolean.parseBoolean(null);
        if (currentRecognitions.isEmpty()) {
            propDetected = false;
        } else if (currentRecognitions.isEmpty() != true) {
            propDetected = true;
        }
        telemetry.update();
        return  propDetected;

    }   // end method telemetryTfod()


}   // end class
