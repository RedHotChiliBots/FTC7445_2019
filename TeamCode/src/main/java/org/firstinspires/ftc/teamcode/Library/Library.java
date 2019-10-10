package org.firstinspires.ftc.teamcode.Library;

import java.util.Arrays;
import java.util.List;


public class Library {

    public double clip(double val, double max, double min) {
        if (val > max) {
            val = max;
        } else if (val < min) {
            val = min;
        }
        return val;
    }

    public List<Double> calcCorrection(double offset, double angle, double yaw) {
        double correct = 0.0;
        double left;
        double right;
        double speed = 0.0;

        left = speed;
        right = speed;

        if ((Math.abs(offset) > 2.0) || (Math.abs(angle) > 2.0)) {
            correct = yaw + (angle / 2);
            correct = clip(correct, 45.0, -45.0);

            left = speed + ((correct / 90.0) * speed);
            right = speed - ((correct / 90.0) * speed);

            left = clip(left, 1.0, -1.0);
            right = clip(right, 1.0, -1.0);
        }

        List<Double> result = Arrays.asList(left,right);
        return result;
    }
}

