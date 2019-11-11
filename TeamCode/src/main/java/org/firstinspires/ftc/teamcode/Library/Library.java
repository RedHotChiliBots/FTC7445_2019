package org.firstinspires.ftc.teamcode.Library;

import java.util.Arrays;
import java.util.List;
import java.util.Collection;


public class Library {

    public double clip(double val, double max, double min) {
        if (val > max) {
            val = max;
        } else if (val < min) {
            val = min;
        }
        return val;
    }

    /**
     * calcCorrection takes relative position information from the Vuforia image
     * tracking and returns a left and right motor command to track the target.
     * The Vuforia responses are distances measured in inches and available via
     * these methods:
     *    getPosDist (x) - positive is cartesian distance from image
     *    getPosOffset (y) - negative is left, positive is right cartesian distance from image
     *    getPosHeight (z) - negative is down, positive is up cartesian distance from image
     *    getPosLOS (calc) - positive is Line Of Sight distance from image
     *    getPosAngle (calc) - negative is left, positive is right degrees from image center
     *    getRoll - n/a
     *    getPitch - n/a
     *    getYaw - negative is left, positive is right degrees of image in camera frame
     *
     * @param offset
     * @param angle
     * @param yaw
     * @return
     */
    public List<Double> calcCorrection(double offset, double angle, double yaw) {
        double correct = 0.0;
        double left;
        double right;
        double speed = 0.5;

        left = speed;
        right = speed;

//        if ((Math.abs(offset) > 2.0) || (Math.abs(angle) > 2.0)) {
            correct = yaw + (angle / 2);
            correct = clip(correct, 45.0, -45.0);

            left = speed + ((correct / 90.0) * speed);
            right = speed - ((correct / 90.0) * speed);

            left = clip(left, 1.0, -1.0);
            right = clip(right, 1.0, -1.0);
//        }

        return Arrays.asList(left,right);
    }

    /**
     * limits the value to be between a and b
     *
     * @param input the value to be limited
     * @param a     one end of the range
     * @param b     the other end of the range
     * @return the input limited to the range between a and b
     */
    public static double limit(double input, double a, double b) {
        if (a == b) return a; // if the ends of the range are equal

        // set min and max to a and b, making sure that min < max
        double min = a, max = b;
        if (a > b) {
            min = b;
            max = a;
        }

        // limit the input to be min < input < max
        if (input > max) return max;
        if (input < min) return min;
        return input;
    }

    /**
     * a limit function where min = -max
     *
     * @param input the value to be limited
     * @param max   the max absolute value
     * @return the input limited to be between -max to max
     */
    public static double mirrorLimit(double input, double max) {
        return limit(input, -max, max);
    }

    /**
     * Limiting function for motor power
     *
     * @param input the value to be limited
     * @return the input limited to the range from -1 to 1
     */
    public static double motorLimit(double input) {
        return mirrorLimit(input, 1);
    }

    /**
     * Limiting function for servo positions
     *
     * @param input the value to be limited
     * @return the input limited to the range from 0 to 1
     */
    public static double servoLimit(double input) {
        return limit(input, 0, 1);
    }

    /**
     * Join a list of objects with a separator
     *
     * @param list      the list of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static <T> String join(Collection<T> list, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (T item : list) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of booleans with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(boolean[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (boolean item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of bytes with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(byte[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (byte item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of chars with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(char[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (char item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of shorts with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(short[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (short item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of ints with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(int[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (int item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of longs with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(long[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (long item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of floats with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(float[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (float item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();
    }

    /**
     * Join an array of doubles with a separator
     *
     * @param array     the array of values
     * @param separator the String to put between the values
     * @return a String of the joined items
     */
    public static String join(double[] array, String separator) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (double item : array) {
            if (first) {
                first = false;
            } else {
                sb.append(separator);
            }
            sb.append(item);
        }
        return sb.toString();

    }
}
