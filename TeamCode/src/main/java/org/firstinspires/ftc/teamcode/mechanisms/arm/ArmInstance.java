package org.firstinspires.ftc.teamcode.mechanisms.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmInstance {

    private double Arm_Motor_Power = 0.2;
    private HardwareMap hardwareMap;
    public int currentArmPos;


    public DcMotor Arm_Motor;

    public void initializeArm(HardwareMap hardwareMap) {
        Arm_Motor = hardwareMap.get(DcMotor.class, "Arm_Motor");
        Arm_Motor.setDirection(DcMotor.Direction.REVERSE);
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setTargetPosition(15);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveArmBy(int Ticks_To_Move_Arm_Encoder_Position_By) {
        int Arm_Motor_Encoder_Current_Position = Arm_Motor.getCurrentPosition();

        int Arm_Motor_Encoder_Target_Position = Arm_Motor_Encoder_Current_Position + Ticks_To_Move_Arm_Encoder_Position_By;
        /*if (Arm_Motor_Encoder_Target_Position <= 5) {
            Arm_Motor_Encoder_Target_Position = 5;
        }
        if (Arm_Motor_Encoder_Target_Position >= 900) {
            Arm_Motor_Encoder_Target_Position = 900;
        }

         */
        Arm_Motor.setTargetPosition(Arm_Motor_Encoder_Target_Position);
        Arm_Motor.setPower(Arm_Motor_Power);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveArmTo(int Desired_Arm_Position) {
        Arm_Motor.setTargetPosition(Desired_Arm_Position);
        Arm_Motor.setPower(Arm_Motor_Power);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setArmMotorPower(double power) {
        Arm_Motor_Power = power;
    }
    public void setArmPosTo(int Ticks_To_Move_Arm_To, double power) {
        int Arm_Motor_Encoder_Target_Position = Ticks_To_Move_Arm_To;
        Arm_Motor.setTargetPosition(Arm_Motor_Encoder_Target_Position);
        Arm_Motor.setPower(power);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentArmPos = Arm_Motor_Encoder_Target_Position;
    }

    public void RunWithoutEncoder(){Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
    public int getCurrentArmPos() {
        return currentArmPos;
    }
}