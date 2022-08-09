package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase.COUNTS_PER_INCH;

public class Util {
    public boolean initialized;

    public Util() {
        initialized = true;
    }

    public double roundTwo(double n) {
        return (double) Math.round(n * 100) / 100;
    }
    public double getCountsAsInches (double odometryCounts) {
        return (double) roundTwo(odometryCounts / COUNTS_PER_INCH);
    }
    public double limit(double min, double max, double num) {
        if (num > max) {
            return max;
        }
        if (num < min) {
            return min;
        }
        return num;
    }
    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }

    public void stop() {
        if (initialized) {
            initialized = false;
        }
    }
    public void reset() {
    }
}