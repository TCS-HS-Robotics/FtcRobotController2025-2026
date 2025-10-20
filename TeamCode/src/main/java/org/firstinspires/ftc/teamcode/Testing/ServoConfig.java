package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoConfig extends LinearOpMode {

    private Servo rightTilt, leftTilt;

    private double rightPos = 0, leftPos = 0;
    private long lastUpdateTime = 0;  // keeps track of time since last movement
    private static final long UPDATE_DELAY_MS = 150; // adjust this delay (smaller = faster)

    @Override
    public void runOpMode() throws InterruptedException {
        leftTilt = hardwareMap.get(Servo.class, "lefttilt");
        rightTilt = hardwareMap.get(Servo.class, "righttilt");

        rightTilt.setDirection(Servo.Direction.REVERSE);
        leftTilt.setPosition(leftPos);
        rightTilt.setPosition(rightPos);

        waitForStart();

        while (opModeIsActive()) {

            // check how long since last update
            long now = System.currentTimeMillis();

            // only update every UPDATE_DELAY_MS milliseconds
            if (now - lastUpdateTime > UPDATE_DELAY_MS) {

                // right servo controls (gamepad1)
                if (gamepad1.dpad_up) {
                    rightPos += 0.01;
                } else if (gamepad1.dpad_down) {
                    rightPos -= 0.01;
                }

                // left servo controls (gamepad2)
                if (gamepad2.dpad_up) {
                    leftPos += 0.01;
                } else if (gamepad2.dpad_down) {
                    leftPos -= 0.01;
                }

                // clamp values between 0 and 1
                rightPos = Math.min(Math.max(rightPos, 0), 1);
                leftPos = Math.min(Math.max(leftPos, 0), 1);

                // update servos
                rightTilt.setPosition(rightPos);
                leftTilt.setPosition(leftPos);

                // update timer
                lastUpdateTime = now;
            }

            telemetry.addData("Right Servo Position", rightPos);
            telemetry.addData("Left Servo Position", leftPos);
            telemetry.update();
        }
    }
}
