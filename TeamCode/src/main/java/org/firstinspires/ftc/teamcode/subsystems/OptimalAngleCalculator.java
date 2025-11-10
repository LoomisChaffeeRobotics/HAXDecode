package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Arrays;

public class OptimalAngleCalculator {
    public double calculateOptimalAngle(double current, double target) {
        double[] angs = {
                target,
                target + 360,
                target - 360
        };

        double[] diffs = new double[5];
        for (int i = 0; i < angs.length; i++) {
            diffs[i] = Math.abs(angs[i] - current);
        }

        int index = 0;
        double min = diffs[index];
        for (int i = 1; i < diffs.length; i++) {
            if (diffs[i] <=min) {
                min = diffs[i];
                index = i;
            }
        }

        return angs[index];
    }
}
