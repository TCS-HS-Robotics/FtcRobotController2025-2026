package com.example.cli_tools;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class ServoRelationshipCLI {

    public static void main(String[] args) {

        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();

        int count = 0;

        Scanner input = new Scanner(System.in);

        System.out.println("Please enter 7 different servo points when servos are parallel.");
        while (count < 7) {
            System.out.print("  Enter x: ");
            x.add(input.nextDouble());

            System.out.print("  Enter y: ");
            y.add(input.nextDouble());
            System.out.println();
            count++;
        }

        System.out.println("X Positions: " + x);
        System.out.println("Y Positions: " + y);

        double[] coeffs = quadraticFit(x, y);
        System.out.println("A: " + coeffs[0]);
        System.out.println("B: " + coeffs[1]);
        System.out.println("C: " + coeffs[2]);

        saveCoefficients(coeffs, "cli-tools/src/main/java/com/example/cli_tools/CalibrationCoeffs.txt");

    }


    /**
     * Computes a quadratic best-fit curve (least squares regression)
     * that maps one servo's position (left) to another (right).
     *
     * It finds coefficients a, b, c such that:
     *      right ≈ a + b*left + c*(left^2)
     *
     * You can then use these coefficients to predict the right servo
     * position for any given left servo position.
     *
     * Requires at least 3 calibration points.
     *
     * @param x list of left servo positions (input)
     * @param y list of right servo positions (output)
     * @return an array [a, b, c] for the quadratic relationship
     */
    public static double[] quadraticFit(List<Double> x, List<Double> y) {
        int n = x.size();
        if (n < 3) {
            throw new IllegalArgumentException("Need at least 5 points for quadratic fit");
        }

        // --- Step 1: Compute all the necessary summations ---
        // These correspond to the normal equations for least-squares fitting.
        double sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumY = 0, sumXY = 0, sumX2Y = 0;

        for (int i = 0; i < n; i++) {
            double xi = x.get(i);
            double yi = y.get(i);
            double xi2 = xi * xi;

            // Build up all the summations used in the linear system
            sumX   += xi;          // Σx
            sumX2  += xi2;         // Σx²
            sumX3  += xi2 * xi;    // Σx³
            sumX4  += xi2 * xi2;   // Σx⁴
            sumY   += yi;          // Σy
            sumXY  += xi * yi;     // Σx·y
            sumX2Y += xi2 * yi;    // Σx²·y
        }

        // --- Step 2: Build the normal equation system ---
        // [ [n, Σx, Σx² ],     [Σy    ]
        //   [Σx, Σx², Σx³],  *  [Σx·y  ]
        //   [Σx², Σx³, Σx⁴] ]    [Σx²·y]
        double[][] A = {
                {n,    sumX,  sumX2},
                {sumX, sumX2, sumX3},
                {sumX2, sumX3, sumX4}
        };

        double[] B = {sumY, sumXY, sumX2Y};

        // --- Step 3: Solve for [a, b, c] ---
        // We'll use Gaussian elimination on the 3x3 system.
        return solve3x3(A, B);
    }

    /**
     * Solves a 3x3 linear system A·x = B using Gaussian elimination.
     *
     * This is a lightweight solver — good enough for small calibration systems.
     *
     * @param A 3x3 coefficient matrix (modified in place)
     * @param B right-hand side vector (modified to become the solution)
     * @return array [a, b, c] — the solution to A·x = B
     */
    public static double[] solve3x3(double[][] A, double[] B) {
        int n = 3;

        // --- Forward elimination ---
        for (int i = 0; i < n; i++) {
            // Normalize the pivot row (make diagonal = 1)
            double pivot = A[i][i];
            for (int j = i; j < n; j++) {
                A[i][j] /= pivot;
            }
            B[i] /= pivot;

            // Eliminate below the pivot
            for (int k = i + 1; k < n; k++) {
                double factor = A[k][i];
                for (int j = i; j < n; j++) {
                    A[k][j] -= factor * A[i][j];
                }
                B[k] -= factor * B[i];
            }
        }

        // --- Back substitution ---
        for (int i = n - 1; i >= 0; i--) {
            for (int k = i - 1; k >= 0; k--) {
                double factor = A[k][i];
                A[k][i] -= factor * A[i][i]; // (which is 1)
                B[k] -= factor * B[i];
            }
        }

        return B;  // [a, b, c]
    }

    public static void saveCoefficients(double[] coeffs, String filename) {
        try (PrintWriter writer = new PrintWriter(new File(filename))) {
            // Write each coefficient on one line
            writer.printf("%.10f%n", coeffs[0]);
            writer.printf("%.10f%n", coeffs[1]);
            writer.printf("%.10f%n", coeffs[2]);
            System.out.println("Coefficients saved to " + filename);
        } catch (IOException e) {
            System.out.println("Error saving coefficients: " + e.getMessage());
        }
    }


}
