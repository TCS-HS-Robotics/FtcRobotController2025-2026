package org.firstinspires.ftc.teamcode.SuckySucky;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpoint.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="SuckySuckyTeleOp", group="Robot")
public class SuckySuckyTeleOp extends LinearOpMode {


    private double speed = 0.5;
    private DcMotor leftFront, leftBack, rightFront, rightBack, shootLeft, shootRight;
    private GoBildaPinpointDriver pinpoint;
    private Servo rightTilt, leftTilt;


    @Override
    public void runOpMode() {

        // --- Hardware mapping ---

        // Drive Train Init
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        // Shooter Init
        shootLeft = hardwareMap.get(DcMotor.class, "shootleft");
        shootRight = hardwareMap.get(DcMotor.class, "shootright");

        // Reverse left side motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        shootLeft.setDirection(DcMotor.Direction.REVERSE);

        // Pinpoint Init
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        // Pinpoint info
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
        telemetry.update();

        // Arm Tilt Init
        leftTilt = hardwareMap.get(Servo.class, "lefttilt");
        rightTilt = hardwareMap.get(Servo.class, "righttilt");

        rightTilt.setDirection(Servo.Direction.REVERSE);
        leftTilt.setPosition(0.2);
        rightTilt.setPosition(0.11);

        sleep(5000);

        leftTilt.setPosition(0.2 + 0.5);
        rightTilt.setPosition(0.11 + 0.4);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            pinpoint.update();

            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", pinpoint.getVelX(DistanceUnit.MM), pinpoint.getVelY(DistanceUnit.MM), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            // Activate Speed Mode
            changeSpeedMode(gamepad1);

            // --- Driving ---

            // Tank Drive
//            tankSideDrive(gamepad1.right_stick_y, rightFront, rightBack, speed); // Right Drive
//            tankSideDrive(gamepad1.left_stick_y, leftFront, leftBack, speed); // Left Drive
//            dPadDrive(gamepad1, leftFront, leftBack, rightFront, rightBack);

            driveControl(gamepad1);

            // Strafing
            if (gamepad1.right_trigger > 0.5) { // Strafe Right
                strafeRight(leftFront, leftBack, rightFront, rightBack);
            }
            if (gamepad1.left_trigger > 0.5) { // Strafe Left
                strafeLeft(leftFront, leftBack, rightFront, rightBack);
            }


            // --- Shooter ---

            if (gamepad2.right_trigger > 0.5) { // Outtake
                shootLeft.setPower(-0.5);
                shootRight.setPower(-0.5);
            } else if (gamepad2.left_trigger > 0.5) { // Intake
                shootLeft.setPower(0.2);
                shootRight.setPower(0.2);
            } else {
                shootLeft.setPower(0.0);
                shootRight.setPower(0.0);
            }





            telemetry.update();

        }



    }

    public void driveControl(Gamepad gamepad) {
        boolean isMoving = false;

        // --- Joystick Tank Drive ---
        if (-gamepad.left_stick_y > 0.5) { // Left stick forward
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            isMoving = true;
        } else if (-gamepad.left_stick_y < -0.5) { // Left stick backward
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            isMoving = true;
        }

        if (-gamepad.right_stick_y > 0.5) { // Right stick forward
            rightFront.setPower(speed);
            rightBack.setPower(speed);
            isMoving = true;
        } else if (-gamepad.right_stick_y < -0.5) { // Right stick backward
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
            isMoving = true;
        }

        // --- DPad Drive Overrides ---
        if (gamepad.dpad_up) {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
            isMoving = true;
        } else if (gamepad.dpad_down) {
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
            isMoving = true;
        } else if (gamepad.dpad_left) {
            strafeLeft(leftFront, leftBack, rightFront, rightBack);
            isMoving = true;
        } else if (gamepad.dpad_right) {
            strafeRight(leftFront, leftBack, rightFront, rightBack);
            isMoving = true;
        }

        // --- If nothing pressed, stop ---
        if (!isMoving) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }

//    public void tankSideDrive(float gamepad_stick_y, DcMotor sideFront, DcMotor sideBack, double speed) {
//        if (-gamepad_stick_y > 0.5) {
//            sideFront.setPower(speed);
//            sideBack.setPower(speed);
//        } else if (-gamepad_stick_y < -0.5) {
//            sideFront.setPower(-speed);
//            sideBack.setPower(-speed);
//        } else {
//            sideFront.setPower(0.0);
//            sideBack.setPower(0.0);
//        }
//    }
//
//    public void dPadDrive(Gamepad gamepad, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
//        if (gamepad.dpad_up) {
//            leftFront.setPower(speed);
//            leftBack.setPower(speed);
//            rightFront.setPower(speed);
//            rightBack.setPower(speed);
//        } else if (gamepad.dpad_down) {
//            leftFront.setPower(-speed);
//            leftBack.setPower(-speed);
//            rightFront.setPower(-speed);
//            rightBack.setPower(-speed);
//        } else if (gamepad.dpad_left) {
//            strafeLeft(leftFront, leftBack, rightFront, rightBack);
//        } else if (gamepad.dpad_right) {
//            strafeRight(leftFront, leftBack, rightFront, rightBack);
//        } else {
//            leftFront.setPower(0.0);
//            leftBack.setPower(0.0);
//            rightFront.setPower(0.0);
//            rightBack.setPower(0.0);
//        }
//    }

    public void strafeRight(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(speed);
    }

    public void strafeLeft(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
        leftFront.setPower(-speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(-speed);
    }

    public void changeSpeedMode(Gamepad gamepad) {
        if (gamepad.right_bumper) { // Activate Fast Mode
            speed = 0.9;
        } else if (gamepad.left_bumper) { // Activate Slow Mode
            speed = 0.5;
        }
    }

}
