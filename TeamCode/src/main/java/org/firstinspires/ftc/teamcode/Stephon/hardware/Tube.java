package org.firstinspires.ftc.teamcode.Stephon.hardware;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tube extends Hardware {

    private CRServo bottomLeft, topLeft, topRight, bottomRight;

    public Tube(HardwareMap hw, Telemetry telemetry, Gamepad gamepad) {

        super(telemetry, gamepad);

        this.bottomLeft = hw.get(CRServo.class, "bottomleft");
        this.topLeft = hw.get(CRServo.class, "topleft");
        this.bottomRight = hw.get(CRServo.class, "bottomright");
        this.topRight = hw.get(CRServo.class, "topright");

        this.bottomLeft.setDirection(CRServo.Direction.REVERSE);
        this.topLeft.setDirection(CRServo.Direction.REVERSE);

    }

    public void inTake() {
        //bottomLeft.setPower(1.0);
        topLeft.setPower(1.0);
        bottomRight.setPower(1.0);
        topRight.setPower(1.0);
    }

    public void outTake() {
        //bottomLeft.setPower(-1.0);
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

}
