package org.firstinspires.ftc.teamcode.Stephon.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain extends Hardware {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private double speed = 0.5;

    public Drivetrain(HardwareMap hw, Telemetry telemetry, Gamepad gamepad) {

        super(telemetry, gamepad);

        this.leftFront = hw.get(DcMotor.class, "leftfront");
        this.rightFront = hw.get(DcMotor.class, "rightfront");
        this.leftBack = hw.get(DcMotor.class, "leftback");
        this.rightBack = hw.get(DcMotor.class, "rightback");

        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void driveControl() {
        this.changeSpeedMode();

        // D-Pad overrides everything
        if (gamepad.dpad_up) {
            this.leftFront.setPower(speed);
            this.leftBack.setPower(speed);
            this.rightFront.setPower(speed);
            this.rightBack.setPower(speed);
        } else if (gamepad.dpad_down) {
            this.leftFront.setPower(-speed);
            this.leftBack.setPower(-speed);
            this.rightFront.setPower(-speed);
            this.rightBack.setPower(-speed);
        } else if (gamepad.dpad_left) {
            this._strafeLeft();
        } else if (gamepad.dpad_right) {
            this._strafeRight();
        }
        // Triggers override joysticks (but not D-pad)
        else if (gamepad.right_trigger > 0.5) {
            this._strafeRight();
        } else if (gamepad.left_trigger > 0.5) {
            this._strafeLeft();
        }
        // Joystick tank drive - BOTH sides handled independently with constant speed
        else {
            double leftPower = 0;
            double rightPower = 0;

            // Left joystick controls left side at constant speed
            if (-gamepad.left_stick_y > 0.5) {
                leftPower = speed;
            } else if (-gamepad.left_stick_y < -0.5) {
                leftPower = -speed;
            }

            // Right joystick controls right side at constant speed
            if (-gamepad.right_stick_y > 0.5) {
                rightPower = speed;
            } else if (-gamepad.right_stick_y < -0.5) {
                rightPower = -speed;
            }

            this.leftFront.setPower(leftPower);
            this.leftBack.setPower(leftPower);
            this.rightFront.setPower(rightPower);
            this.rightBack.setPower(rightPower);
        }
    }

    public void changeSpeedMode() {
        if (gamepad.right_bumper) { // Activate Fast Mode
            speed = 0.9;
        } else if (gamepad.left_bumper) { // Activate Slow Mode
            speed = 0.5;
        }
    }

    public void driveStraight(double speed) {
        this.leftFront.setPower(speed);
        this.leftBack.setPower(speed);
        this.rightFront.setPower(speed);
        this.rightBack.setPower(speed);
    }

    public void driveBackward(double speed) {
        this.leftFront.setPower(-speed);
        this.leftBack.setPower(-speed);
        this.rightFront.setPower(-speed);
        this.rightBack.setPower(-speed);
    }

    public void turnRight(double speed) {
        this.leftFront.setPower(speed);
        this.leftBack.setPower(speed);
        this.rightFront.setPower(-speed);
        this.rightBack.setPower(-speed);
    }

    public void turnLeft(double speed) {
        this.leftFront.setPower(-speed);
        this.leftBack.setPower(-speed);
        this.rightFront.setPower(speed);
        this.rightBack.setPower(speed);
    }

    public void stop() {
        this.leftFront.setPower(0);
        this.leftBack.setPower(0);
        this.rightFront.setPower(0);
        this.rightBack.setPower(0);
    }


    // --- Helper methods ---

    public void _strafeRight() {
        this.leftFront.setPower(speed);
        this.leftBack.setPower(-speed);
        this.rightFront.setPower(-speed);
        this.rightBack.setPower(speed);
    }

    public void _strafeLeft() {
        this.leftFront.setPower(-speed);
        this.leftBack.setPower(speed);
        this.rightFront.setPower(speed);
        this.rightBack.setPower(-speed);
    }

    public void _strafeRight(double speed) {
        this.leftFront.setPower(speed);
        this.leftBack.setPower(-speed);
        this.rightFront.setPower(-speed);
        this.rightBack.setPower(speed);
    }

    public void _strafeLeft(double speed) {
        this.leftFront.setPower(-speed);
        this.leftBack.setPower(speed);
        this.rightFront.setPower(speed);
        this.rightBack.setPower(-speed);
    }




}