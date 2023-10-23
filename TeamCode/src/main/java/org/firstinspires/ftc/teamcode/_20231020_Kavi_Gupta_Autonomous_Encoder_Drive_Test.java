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

package org.firstinspires.ftc.teamcode;

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

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Encoder Drive Test 4 Lateral + CW 3K Steps Rev.2")
public class _20231020_Kavi_Gupta_Autonomous_Encoder_Drive_Test extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private double Ticks_Per_Inch = 45.2763982107824;

    private int leftFrontDriveTickTracker = 0;
    private int rightFrontDriveTickTracker = 0;
    private int leftBackDriveTickTracker = 0;
    private int rightBackDriveTickTracker = 0;

    private boolean ended = false;




    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motorFL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFR");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "motorBL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBR");

        ResetEncoders();

        waitForStart();


        while (opModeIsActive()) {
            if (moveForward(0.5, 10)) {
                telemetry.addLine("Ended Confirmed");
                telemetry.update();
                moveBackward(0.5,10);
            };
            //moveBackward(0.5, 10);
            //moveLeft(0.5, 10);
            //moveRight(0.5, 10);
            //rotateClockwise(0.5, 3000);
            //moveBackward(0.5, 25);
        }
    }


    public boolean isNotActive() {
        if (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            return false;
        } else {
            return true;
        }
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

    public int GetAverageEncoderPositions() {
        int Average_Ticks = (leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 4;
        return Average_Ticks;
    }
    public void ResetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Reset");
        telemetry.update();
    }

    public boolean moveForward(double speed, double inches) {

        int leftFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches); //2000
        int rightFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int leftBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);


        int leftFrontDriveCurrentTicks = leftFrontDrive.getCurrentPosition();
        int rightFrontDriveCurrentTicks = rightFrontDrive.getCurrentPosition();
        int leftBackDriveCurrentTicks = leftBackDrive.getCurrentPosition();
        int rightBackDriveCurrentTicks = rightBackDrive.getCurrentPosition();

        int leftFrontDriveTargetTicks =  leftFrontDriveNecessaryTicks;
        int rightFrontDriveTargetTicks = rightFrontDriveNecessaryTicks;
        int leftBackDriveTargetTicks = leftBackDriveNecessaryTicks;
        int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;




        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("forward");

        telemetry.addData("Left Front Necessary Ticks", leftFrontDriveNecessaryTicks);
        telemetry.addData("Right Front Necessary Ticks", rightFrontDriveNecessaryTicks);
        telemetry.addData("Left Back Necessary Ticks", leftBackDriveNecessaryTicks);
        telemetry.addData("Right Back Necessary Ticks", rightBackDriveNecessaryTicks);

        leftFrontDrive.setTargetPosition(leftFrontDriveTargetTicks);
        leftFrontDrive.setPower(speed);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setTargetPosition(rightFrontDriveTargetTicks);
        rightFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(leftBackDriveTargetTicks);
        leftBackDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(rightBackDriveTargetTicks);
        rightBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Subtraction: ", Math.abs(leftFrontDriveCurrentTicks - leftFrontDriveTargetTicks));
        motionTelemetry();

        if (Math.abs(leftFrontDriveCurrentTicks - leftFrontDriveTargetTicks) <= 10 && leftFrontDriveCurrentTicks - leftFrontDriveTargetTicks >= -10) {
            telemetry.addLine("Forward Ended");
            ResetEncoders();
            return true;
        } else {
            telemetry.addLine("Moving Forward");
            return false;
        }
    }

    public boolean moveBackward(double speed, double inches) {

        int leftFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches); //2000
        int rightFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int leftBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);


        int leftFrontDriveCurrentTicks = leftFrontDrive.getCurrentPosition();
        int rightFrontDriveCurrentTicks = rightFrontDrive.getCurrentPosition();
        int leftBackDriveCurrentTicks = leftBackDrive.getCurrentPosition();
        int rightBackDriveCurrentTicks = rightBackDrive.getCurrentPosition();

        int leftFrontDriveTargetTicks =  leftFrontDriveNecessaryTicks;
        int rightFrontDriveTargetTicks = rightFrontDriveNecessaryTicks;
        int leftBackDriveTargetTicks = leftBackDriveNecessaryTicks;
        int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;




        SetFrontLeftDriveDirection("backward");
        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");
        SetBackRightDriveDirection("backward");

        telemetry.addLine("Moving Backward");
        telemetry.addData("Left Front Necessary Ticks", leftFrontDriveNecessaryTicks);
        telemetry.addData("Right Front Necessary Ticks", rightFrontDriveNecessaryTicks);
        telemetry.addData("Left Back Necessary Ticks", leftBackDriveNecessaryTicks);
        telemetry.addData("Right Back Necessary Ticks", rightBackDriveNecessaryTicks);

        leftFrontDrive.setTargetPosition(leftFrontDriveTargetTicks);
        leftFrontDrive.setPower(speed);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setTargetPosition(rightFrontDriveTargetTicks);
        rightFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(leftBackDriveTargetTicks);
        leftBackDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(rightBackDriveTargetTicks);
        rightBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motionTelemetry();

        if (leftFrontDriveCurrentTicks - leftFrontDriveTargetTicks <= 5 && leftFrontDriveCurrentTicks - leftFrontDriveTargetTicks >= -5) {
            telemetry.addLine("Backward Ended");
            ResetEncoders();
            return true;
        } else {
            telemetry.addLine("Moving Backward");
            return false;
        }
    }

    public void moveLeft(double speed, double inches) {

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontLeftDriveDirection("backward");
        SetBackRightDriveDirection("backward");

        leftFrontDrive.setTargetPosition(Rounded_Encoder_Ticks);
        leftFrontDrive.setPower(speed);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(Rounded_Encoder_Ticks);
        rightBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motionTelemetry();
    }

    public void moveRight(double speed, double inches) {

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");


        rightFrontDrive.setTargetPosition(Rounded_Encoder_Ticks);
        rightFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(Rounded_Encoder_Ticks);
        leftBackDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motionTelemetry();
    }

    public void rotateClockwise(double speed, double steps) {
        int Current_Encoder_Ticks = GetAverageEncoderPositions();

        ResetEncoders();

        double Calculated_Encoder_Ticks = (steps * Ticks_Per_Inch) + Current_Encoder_Ticks;
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);

        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("backward");

        leftFrontDrive.setTargetPosition(Rounded_Encoder_Ticks);
        leftFrontDrive.setPower(speed);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setTargetPosition(Rounded_Encoder_Ticks);
        rightFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(Rounded_Encoder_Ticks);
        leftBackDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(Rounded_Encoder_Ticks);
        rightBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motionTelemetry();
    }

    public int calculateTicksForLateralMovement(double inches) {
        double Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Rounded_Encoder_Ticks;
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

}   // end class
