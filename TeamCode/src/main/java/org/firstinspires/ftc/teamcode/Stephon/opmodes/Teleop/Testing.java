package org.firstinspires.ftc.teamcode.Stephon.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Testing", group = "Stephon")
public class Testing extends LinearOpMode {

    Servo stopper;

    @Override
    public void runOpMode() throws InterruptedException {

        stopper = hardwareMap.get(Servo.class, "stopper");
        double position = 0;

        waitForStart();

        while (opModeIsActive()) {
            // Clamp position to valid servo range [0, 1]
            if (-gamepad1.right_stick_y > 0.5) {
                position = Math.min(1.0, position + 0.01);  // Slower increment + clamping
            }
            if (-gamepad1.right_stick_y < -0.5) {
                position = Math.max(0.0, position - 0.01);  // Slower decrement + clamping
            }

            stopper.setPosition(position);
            telemetry.addLine("Position: " + position);
            telemetry.update();
        }
    }
}
