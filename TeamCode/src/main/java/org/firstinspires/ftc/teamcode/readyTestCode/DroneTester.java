package org.firstinspires.ftc.teamcode.readyTestCode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.DroneLauncherInstance;
@TeleOp
public class DroneTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo DroneLauncherServo = hardwareMap.get(Servo.class, "Drone_Launcher_Servo");
        DroneLauncherServo.setDirection(Servo.Direction.FORWARD);
        DroneLauncherServo.scaleRange(0, 1);
        DroneLauncherServo.setPosition(0.3);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                DroneLauncherServo.setPosition(1);
                sleep(1500);
                DroneLauncherServo.setPosition(0.3);
            }
            telemetry.addData("Drone Launcher Position: ", DroneLauncherServo.getPosition());
            telemetry.update();
        }
    }
}
