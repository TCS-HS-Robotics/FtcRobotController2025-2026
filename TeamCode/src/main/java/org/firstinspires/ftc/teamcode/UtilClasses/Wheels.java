package org.firstinspires.ftc.teamcode.UtilClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Wheels {
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;


    public Wheels(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;

    }

    public void moveForward() {
        this.leftFront.setPower(0.3);
        this.leftBack.setPower(0.3);
        this.rightFront.setPower(0.3);
        this.rightBack.setPower(0.3);
    }

    public void stop() {
        this.leftFront.setPower(0.0);
        this.leftBack.setPower(0.0);
        this.rightFront.setPower(0.0);
        this.rightBack.setPower(0.0);

    }

    public void turnRight() {
        this.leftFront.setPower(0.3);
        this.leftBack.setPower(0.3);
        this.rightFront.setPower(-0.3);
        this.rightBack.setPower(-0.3);
    }

    public void turnLeft() {
        this.leftFront.setPower(-0.3);
        this.leftBack.setPower(-0.3);
        this.rightFront.setPower(0.3);
        this.rightBack.setPower(0.3);
    }
}
