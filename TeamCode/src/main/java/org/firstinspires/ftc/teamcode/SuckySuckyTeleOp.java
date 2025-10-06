package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="SuckySuckyTeleOp", group="Robot")
public class SuckySuckyTeleOp extends LinearOpMode {


    private double speed = 0.5;
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {

        // Hardware mapping
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        // Reverse left side motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Activate Speed Mode
            if (gamepad1.right_bumper) { // Activate Fast Mode
                speed = 0.9;
            } else if (gamepad1.left_bumper) { // Activate Slow Mode
                speed = 0.5;
            }

            // Tank Drive
            if (-gamepad1.left_stick_y > 0.5) {
                leftFront.setPower(speed);
                leftBack.setPower(speed);
            } else if (-gamepad1.left_stick_y < -0.5) {
                leftFront.setPower(-speed);
                leftBack.setPower(-speed);
            } else {
                leftFront.setPower(0.0);
                leftBack.setPower(0.0);
            }

            if (-gamepad1.right_stick_y > 0.5) {
                rightFront.setPower(speed);
                rightBack.setPower(speed);
            } else if (-gamepad1.right_stick_y < -0.5) {
                rightFront.setPower(-speed);
                rightBack.setPower(-speed);
            } else {
                rightFront.setPower(0.0);
                rightBack.setPower(0.0);
            }

            // Strafing
            if (gamepad1.left_trigger > 0.5) { // Strafe Right
                leftFront.setPower(speed);
                leftBack.setPower(-speed);
                rightFront.setPower(-speed);
                rightBack.setPower(speed);
            }
            if (gamepad1.right_trigger > 0.5) { // Strafe Left
                leftFront.setPower(-speed);
                leftBack.setPower(speed);
                rightFront.setPower(speed);
                rightBack.setPower(-speed);
            }

        }
    }
}
