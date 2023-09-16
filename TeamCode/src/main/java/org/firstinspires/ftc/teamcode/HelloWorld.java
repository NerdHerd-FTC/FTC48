package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hello World", group = "Robot")
public class HelloWorld extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Hello World!");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            boolean buttonA = false;
            if (gamepad1.a) {
                telemetry.addData("Game Pad 1, A Button: ", buttonA);
            }
        }
    }
}
