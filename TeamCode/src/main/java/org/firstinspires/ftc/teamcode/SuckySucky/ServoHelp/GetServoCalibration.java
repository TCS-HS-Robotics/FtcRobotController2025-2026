package org.firstinspires.ftc.teamcode.SuckySucky.ServoHelp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * OpMode for calibrating servos.
 *
 * Features:
 * - Collect calibration points manually.
 * - Compute a quadratic relationship between the left and right servos.
 * - Live motion mode: move left servo with joystick, right servo follows automatically.
 * - Save calibration coefficients to a file for use in main TeleOp.
 */
@TeleOp(name="GetServoCalibration", group="Robot")
public class GetServoCalibration extends LinearOpMode {

    private Servo rightTilt, leftTilt;

    private double rightPos = 0, leftPos = 0;            // Current servo positions
    private long lastUpdateTime = 0;                     // Timer for update delay
    private static final long UPDATE_DELAY_MS = 150;     // Delay between manual servo updates

    private List<Double> leftData = new ArrayList<>();   // Collected left servo positions
    private List<Double> rightData = new ArrayList<>();  // Collected right servo positions
    private double[] coeffs = null;                      // Quadratic coefficients a, b, c

    private List<String> pointsSaved = new ArrayList<>();

    private boolean liveMotion = false;                  // Live motion toggle

    private static final String COEFF_FILE = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SuckySucky/ServoHelp/CalibrationCoeffs.txt"; // File to save/load coefficients

    private double leftMin, rightMin;
    private static final String MIN_FILE = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SuckySucky/ServoHelp/MinimumServoValues.txt";

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servos from hardware map
        leftTilt = hardwareMap.get(Servo.class, "lefttilt");
        rightTilt = hardwareMap.get(Servo.class, "righttilt");

        rightTilt.setDirection(Servo.Direction.REVERSE);  // Reverse right servo direction
        leftTilt.setPosition(leftPos);
        rightTilt.setPosition(rightPos);

        waitForStart();

        while (opModeIsActive()) {

            // Toggle live motion mode with Y button
            if (gamepad1.y) {
                liveMotion = !liveMotion;
                sleep(300); // debounce
                telemetry.addLine(liveMotion ? "Live Motion Activated." : "Live Motion Deactivated.");
            }

            double deadzone = 0.1; // Joystick deadzone

            // --- Live motion mode ---
            if (liveMotion) {
                if (coeffs == null) {
                    telemetry.addLine("Calculate coefficients first (press X).");
                    liveMotion = false;
                } else {
                    // Move left servo with left joystick
                    if (Math.abs(gamepad1.left_stick_y) > deadzone) {
                        leftPos -= 0.02 * gamepad1.left_stick_y;  // negative = up
                        leftPos = clamp(leftPos);
                        rightPos = calcRightPos(leftPos, coeffs);
                        rightPos = clamp(rightPos);

                        leftTilt.setPosition(leftPos);
                        rightTilt.setPosition(rightPos);
                    }
                }
            }
            // --- Manual calibration mode ---
            else {
                long now = System.currentTimeMillis();
                if (now - lastUpdateTime > UPDATE_DELAY_MS) {

                    // Right servo manual control
                    if (-gamepad1.left_stick_y > 0.5) {
                        rightPos += 0.01;
                        rightPos = clamp(rightPos);
                    }
                    if (-gamepad1.left_stick_y < -0.5) {
                        rightPos -= 0.01;
                        rightPos = clamp(rightPos);
                    }

                    // Left servo manual control
                    if (-gamepad1.right_stick_y > 0.5) {
                        leftPos += 0.01;
                        leftPos = clamp(leftPos);
                    }
                    if (-gamepad1.right_stick_y < -0.5) {
                        leftPos -= 0.01;
                        leftPos = clamp(leftPos);
                    }
                    rightTilt.setPosition(rightPos);
                    leftTilt.setPosition(leftPos);


                    // Save current positions as a calibration point
                    if (gamepad1.a) {
                        savePoint(leftPos, rightPos);
                        telemetry.addLine("Saved Point: (" + leftPos + ", " + rightPos + ")");
                        sleep(150); // debounce
                    }

                    // Save current positions as minimum point
                    if (gamepad1.b) {
                        leftMin = leftPos;
                        rightMin = rightPos;
                        saveMinPosition(leftMin, rightMin, MIN_FILE);
                        telemetry.addLine("Minimum positions saved!");
                        sleep(300); // debounce
                    }

                    // Calculate quadratic coefficients from saved points
                    if (gamepad1.x) {
                        if (leftData.size() >= 3) {
                            coeffs = quadraticFit(leftData, rightData);
                            telemetry.addLine(String.format(Locale.US, "Coefficients: a=%.4f, b=%.4f, c=%.4f",
                                    coeffs[0], coeffs[1], coeffs[2]));
                        } else {
                            telemetry.addLine("Not enough points to calculate coefficients (min 3).");
                        }
                        sleep(300); // debounce
                    }

                    lastUpdateTime = now;
                }
            }

            // Save coefficients with both bumpers
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                if (coeffs != null) { // Make sure coefficients exist
                    saveCoefficients(coeffs, COEFF_FILE);
                    telemetry.addLine("Coefficients saved to file!");
                } else {
                    telemetry.addLine("No coefficients to save.");
                }
                sleep(300); // debounce so it doesn’t trigger multiple times
            }

            // Update telemetry every loop
            telemetry.addData("Left Servo", leftPos);
            telemetry.addData("Right Servo", rightPos);
            telemetry.addData("Live Motion", liveMotion);

            displaySavedPoints(telemetry);
            telemetry.update();
        }
    }

    private void savePoint(double left, double right) {
        leftData.add(left);
        rightData.add(right);
        pointsSaved.add(String.format(Locale.US, "(%.3f, %.3f)", left, right));
    }

    private void displaySavedPoints(Telemetry telemetry) {
        if (pointsSaved.isEmpty()) {
            return;
        }
        telemetry.addLine("Saved Points:");
        for (String point : pointsSaved) {
            telemetry.addLine("  " + point);
        }
    }



    /**
     * Clamp a servo position between 0 and 1.
     *
     * @param val Position value
     * @return Clamped value in [0,1]
     */
    private double clamp(double val) {
        return Math.max(0, Math.min(1, val));
    }

    /**
     * Calculate right servo position using quadratic coefficients.
     *
     * @param leftPos Current left servo position
     * @param coeffs Array [a, b, c] of quadratic coefficients
     * @return Predicted right servo position
     */
    public static double calcRightPos(double leftPos, double[] coeffs) {
        return coeffs[0] + coeffs[1] * leftPos + coeffs[2] * leftPos * leftPos;
    }

    /**
     * Save coefficients to a file.
     *
     * @param coeffs Array [a, b, c]
     * @param filename File path to save
     */
    public static void saveCoefficients(double[] coeffs, String filename) {
        try (PrintWriter writer = new PrintWriter(new File(filename))) {
            writer.printf("%.6f%n", coeffs[0]);
            writer.printf("%.6f%n", coeffs[1]);
            writer.printf("%.6f%n", coeffs[2]);
            writer.flush();
        } catch (IOException e) {
            System.out.println("Error saving coefficients: " + e.getMessage());
        }
    }

    /**
     * Save the servo positions when servos are at minimum positions.
     *
     * @param left Left servo position
     * @param right Right servo position
     * @param filename File path
     */
    public static void saveMinPosition(double left, double right, String filename) {
        try (PrintWriter writer = new PrintWriter(new File(filename))) {
            writer.printf("%.6f%n", left);
            writer.printf("%.6f%n", right);
            writer.flush();
        } catch (IOException e) {
            System.out.println("Error saving parallel position: " + e.getMessage());
        }
    }

    /**
     * Compute quadratic best-fit curve for servo positions.
     * Right servo ≈ a + b*left + c*(left^2)
     *
     * @param x Left servo positions
     * @param y Right servo positions
     * @return Array [a, b, c] coefficients
     */
    public static double[] quadraticFit(List<Double> x, List<Double> y) {
        int n = x.size();
        if (n < 3) throw new IllegalArgumentException("Need at least 3 points for quadratic fit");

        double sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumY = 0, sumXY = 0, sumX2Y = 0;

        for (int i = 0; i < n; i++) {
            double xi = x.get(i);
            double yi = y.get(i);
            double xi2 = xi * xi;

            sumX += xi;
            sumX2 += xi2;
            sumX3 += xi2 * xi;
            sumX4 += xi2 * xi2;
            sumY += yi;
            sumXY += xi * yi;
            sumX2Y += xi2 * yi;
        }

        double[][] A = {
                {n, sumX, sumX2},
                {sumX, sumX2, sumX3},
                {sumX2, sumX3, sumX4}
        };

        double[] B = {sumY, sumXY, sumX2Y};

        return solve3x3(A, B);
    }

    /**
     * Solve 3x3 linear system using Gaussian elimination.
     *
     * @param A Coefficient matrix (3x3)
     * @param B Right-hand side vector
     * @return Solution vector [a, b, c]
     */
    public static double[] solve3x3(double[][] A, double[] B) {
        int n = 3;

        // Forward elimination
        for (int i = 0; i < n; i++) {
            double pivot = A[i][i];
            for (int j = i; j < n; j++) A[i][j] /= pivot;
            B[i] /= pivot;

            for (int k = i + 1; k < n; k++) {
                double factor = A[k][i];
                for (int j = i; j < n; j++) A[k][j] -= factor * A[i][j];
                B[k] -= factor * B[i];
            }
        }

        // Back substitution
        for (int i = n - 1; i >= 0; i--) {
            for (int k = i - 1; k >= 0; k--) {
                double factor = A[k][i];
                B[k] -= factor * B[i];
            }
        }

        return B;
    }
}
