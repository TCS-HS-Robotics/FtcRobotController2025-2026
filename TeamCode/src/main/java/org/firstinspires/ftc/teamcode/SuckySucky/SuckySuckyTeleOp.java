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

import java.io.File;
import java.io.IOException;
import java.util.Locale;
import java.util.Scanner;

@TeleOp(name="SuckySuckyTeleOp", group="Robot")
public class SuckySuckyTeleOp extends LinearOpMode {


    private double speed = 0.5;
    private DcMotor leftFront, leftBack, rightFront, rightBack, shootLeft, shootRight;
    private GoBildaPinpointDriver pinpoint;
    private Servo rightTilt, leftTilt;

    // Shooter Specifics
    private double[] servo_coeffs = {-0.0321039824, 0.7489592892, -0.0689888076};//loadCoefficients();
    private double[] servo_mins = {0.2, 0.11};//loadMins();
    private double leftTiltMin = servo_mins[0];
    private double rightTiltMin = servo_mins[1];

    private double[] servo_maxs = {0.636, 0.416};
    private double leftTiltMax = servo_maxs[0];
    private double rightTiltMax = servo_maxs[1];

    private double leftTiltPos = leftTiltMin;
    private double rightTiltPos = rightTiltMin;

    private double[] shootingPos1 = {0.394, 0.252};


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
        leftTilt.setPosition(leftTiltMin);
        rightTilt.setPosition(rightTiltMin);

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

            // Shooter Angle
            handleShooterAngle(gamepad2);


            telemetry.addLine("A: " + servo_coeffs[0] + " B: " + servo_coeffs[1] + " C: " + servo_coeffs[2]);
            telemetry.addLine(String.format(Locale.US, "Servos Pos: (%.3f, %.3f)", leftTiltPos, rightTiltPos));

            telemetry.update();

        }



    }

    // --- Driving Methods ---
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


    // --- Shooter Methods ---

    private void goTo(double[] position) {
        leftTiltPos = position[0];
        rightTiltPos = position[1];
    }

    private double _clampLeft(double val) {
        return Math.max(leftTiltMin, Math.min(leftTiltMax, val));
    }

    private double _clampRight(double val) {
        return Math.max(rightTiltMin, Math.min(rightTiltMax, val));
    }

    /**
     * Calculate right servo position using quadratic coefficients.
     *
     * @param leftPos Current left servo position
     * @param coeffs Array [a, b, c] of quadratic coefficients
     * @return Predicted right servo position
     */
    public double _calcRightTiltPos(double leftPos, double[] coeffs) {
        return coeffs[0] + coeffs[1] * leftPos + coeffs[2] * leftPos * leftPos;
    }

    public void handleShooterAngle(Gamepad gamepad) {

        if (gamepad.a) {
            leftTiltPos = leftTiltMin;
            rightTiltPos = rightTiltMin;
        }

        if (gamepad.b) {
            goTo(shootingPos1);
        }

        if (-gamepad.left_stick_y > 0.5) {
            leftTiltPos += 0.002;
            rightTiltPos = _calcRightTiltPos(leftTiltPos, servo_coeffs);
        }
        if (-gamepad.left_stick_y < -0.5) {
            leftTiltPos -= 0.002;
            rightTiltPos = _calcRightTiltPos(leftTiltPos, servo_coeffs);
        }

        leftTiltPos = _clampLeft(leftTiltPos);
        rightTiltPos = _clampRight(rightTiltPos);

        leftTilt.setPosition(leftTiltPos);
        rightTilt.setPosition(rightTiltPos);
    }


    /**
     * Load coefficients from a file.
     *
     * @return Array [a, b, c] of coefficients
     */
    public static double[] loadCoefficients() {
        double[] coeffs = new double[3];
        try (Scanner scanner = new Scanner(new File("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SuckySucky/ServoHelp/CalibrationCoeffs.txt"))) {
            for (int i = 0; i < 3; i++) {
                if (scanner.hasNextDouble()) {
                    coeffs[i] = scanner.nextDouble();
                } else {
                    throw new IOException("File does not contain enough numbers");
                }
            }
        } catch (IOException e) {
            System.out.println("Error loading coefficients: " + e.getMessage());
        }
        return coeffs;
    }

    /**
     * Load coefficients from a file.
     *
     * @return Array [a, b, c] of coefficients
     */
    public static double[] loadMins() {
        double[] mins = new double[2];
        try (Scanner scanner = new Scanner(new File("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SuckySucky/ServoHelp/CalibrationCoeffs.txt"))) {
            for (int i = 0; i < 2; i++) {
                if (scanner.hasNextDouble()) {
                    mins[i] = scanner.nextDouble();
                } else {
                    throw new IOException("File does not contain enough numbers");
                }
            }
        } catch (IOException e) {
            System.out.println("Error loading coefficients: " + e.getMessage());
        }
        return mins;
    }

}
