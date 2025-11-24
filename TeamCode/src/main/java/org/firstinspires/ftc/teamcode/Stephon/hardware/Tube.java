package org.firstinspires.ftc.teamcode.Stephon.hardware;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tube extends Hardware {

    private CRServo bottomLeft, topLeft, topRight, bottomRight;

    private Servo stopper;
    private final double STOPPER_OPEN_POS = 0.02;
    private final double STOPPER_CLOSE_POS = 0.19;

    public Tube(HardwareMap hw, Telemetry telemetry, Gamepad gamepad) {

        super(telemetry, gamepad);

        this.bottomLeft = hw.get(CRServo.class, "bottomleft");
        this.topLeft = hw.get(CRServo.class, "topleft");
        this.bottomRight = hw.get(CRServo.class, "bottomright");
        this.topRight = hw.get(CRServo.class, "topright");
        this.stopper = hw.get(Servo.class, "stopper");

        this.bottomLeft.setDirection(CRServo.Direction.REVERSE);
        this.topLeft.setDirection(CRServo.Direction.REVERSE);

        //this.bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void handleIntakeOuttake() {
        if (gamepad.right_bumper) {
            outTake();
        } else if (gamepad.left_bumper || gamepad.left_trigger > 0.5) {
            inTake();
        } else {
            stop();
        }
    }

    public void inTake() {
        bottomLeft.setPower(1.0);
        topLeft.setPower(1.0);
        bottomRight.setPower(1.0);
        topRight.setPower(1.0);
    }

    public void outTake() {
        bottomLeft.setPower(-1.0);
        topLeft.setPower(-1.0);
        bottomRight.setPower(-1.0);
        topRight.setPower(-1.0);
    }

    public void stop() {
        bottomLeft.setPower(0.0);
        topLeft.setPower(0.0);
        bottomRight.setPower(0.0);
        topRight.setPower(0.0);
    }

    public void handleStopper() {
        if (gamepad.b) {
            openStopper();
        } else if (gamepad.a || gamepad.y || gamepad.left_stick_y > 0.5 || gamepad.left_stick_y < -0.5 || gamepad.right_stick_y > 0.5) {
            closeStopper();
        }
    }

    public void openStopper() {
        stopper.setPosition(STOPPER_OPEN_POS);
    }

    public void closeStopper() {
        stopper.setPosition(STOPPER_CLOSE_POS);
    }

}
