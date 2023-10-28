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

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Full Encoder Test 10/27")
public class _20231027_Kavi_Gupta_Autonomous_Encoder_Drive_Test extends LinearOpMode {

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private int Ticks_Per_Inch = 45;

    private double Ticks_Per_Rotational_Degree = 50;

    private double baseAutonomousSpeed = 0.75;



    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBR");

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        if (opModeIsActive()) {
            int Rounded_Encoder_Ticks = 1000; //calculateTicksForLateralMovement(250);

            runDrives(0.5, 1000, 1000, 1000, 1000);
        }
    }

    //"Building Blocks" Code

        //Code to create flexible drives

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

    public void runDrives(double speed, int frontLeftTicks, int frontRightTicks, int backLeftTicks, int backRightTicks) {
        leftFrontDrive.setTargetPosition(frontLeftTicks);
        rightFrontDrive.setTargetPosition(frontRightTicks);
        leftBackDrive.setTargetPosition(backLeftTicks);
        rightBackDrive.setTargetPosition(backRightTicks);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        leftFrontDrive.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        rightFrontDrive.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        leftBackDrive.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        rightBackDrive.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        telemetry.addData("Left Front Target Position", leftFrontDrive.getTargetPosition());
        telemetry.addData("Left Front Current Position", leftFrontDrive.getCurrentPosition());

        telemetry.addData("Right Front Target Position", rightFrontDrive.getTargetPosition());
        telemetry.addData("Right Front Current Position", rightFrontDrive.getCurrentPosition());

        telemetry.addData("Left Back Target Position", leftBackDrive.getTargetPosition());
        telemetry.addData("Left Back Current Position", leftBackDrive.getCurrentPosition());

        telemetry.addData("Right Back Target Position", rightBackDrive.getTargetPosition());
        telemetry.addData("Right Back Current Position", rightBackDrive.getCurrentPosition());

        telemetry.update();



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

        //resetEncoders();
    }

    public void moveBackward(double speed, int inches) {

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

        resetEncoders();
    }

    public void moveLeft(double speed, int inches) {

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontLeftDriveDirection("backward");
        SetBackRightDriveDirection("backward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        resetEncoders();
    }

    public void moveRight(double speed, int inches) {

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();
    }

    public void rotateClockwise(double speed, double degrees) {

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

        resetEncoders();
    }

    public void rotateCounterClockwise(double speed, double degrees) {

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

        resetEncoders();
    }

    public void strafeForwardLeft(double speed, double inches) {
        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        resetEncoders();
    }

    public void strafeForwardRight(double speed, double inches) {
        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontLeftDriveDirection("forward");
        SetBackRightDriveDirection("forward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        resetEncoders();
    }

    public void strafeBackwardLeft(double speed, double inches) {
        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontLeftDriveDirection("backward");
        SetBackRightDriveDirection("backward");

        runLeftFrontDrive(speed, Rounded_Encoder_Ticks);

        runRightBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        resetEncoders();
    }

    public void strafeBackwardRight(double speed, double inches) {
        int Rounded_Encoder_Ticks = calculateTicksForDiagonalStrafingMovement(inches);

        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");

        runRightFrontDrive(speed, Rounded_Encoder_Ticks);

        runLeftBackDrive(speed, Rounded_Encoder_Ticks);

        motionTelemetry();

        resetEncoders();
    }

    public int calculateTicksForLateralMovement(int inches) {
        int Current_Encoder_Ticks = GetAverageEncoderPositions();

        int Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Calculated_Encoder_Ticks;
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

}   // end class
