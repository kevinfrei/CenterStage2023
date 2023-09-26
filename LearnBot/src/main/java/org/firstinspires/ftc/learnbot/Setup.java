package org.firstinspires.ftc.learnbot;

import com.acmerobotics.dashboard.config.Config;

public class Setup {

    @Config
    public static class Connected {

        public static boolean DRIVEBASE = false;
        public static boolean TESTSUBSYSTEM = false;
        public static boolean PATHDRIVEBASE = true;

        // Stuff for testing
        public static boolean MOTOR = false;
        public static boolean SERVO = false;
        public static boolean DISTANCE_SENSOR = false;
        public static boolean COLOR_SENSOR = false;
    }

    @Config
    public static class HardwareNames {

        public static String FLMOTOR = "fl";
        public static String FRMOTOR = "fr";
        public static String RLMOTOR = "rl";
        public static String RRMOTOR = "rr";
        public static String IMU = "imu";
        public static String MOTOR = "m";
        public static String SERVO = "s";
        public static String DISTANCE = "d";
        public static String COLOR = "c";
    }

    @Config
    public static class OtherSettings {

        public static double STICK_DEAD_ZONE = 0.1;
        public static int AUTOTIME = 25;
    }

    @Config
    public static class HeadingSettings {

        // maximum robot rotation rate
        // Changing this will require re-PID tuning the HeadingDriveCommand's ROTATE_PID values
        public static double DEGREES_PER_SECOND = 120;
        // don't consider delays longer than this when calculating heading changes
        public static double LONGEST_DELAY = 0.1;
        // if the delay has been longer than LONGEST_DELAY, use this as the time change
        public static double FIRST_CHANGE = 0.05;
    }
}
