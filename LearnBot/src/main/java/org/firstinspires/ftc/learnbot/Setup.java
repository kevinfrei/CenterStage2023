package org.firstinspires.ftc.learnbot;

import com.acmerobotics.dashboard.config.Config;

public class Setup {

    @Config
    public static class Connected {

        public static boolean DRIVEBASE = false;
        public static boolean TESTSUBSYSTEM = false;
        public static boolean pathDriveBase = true;

        // Stuff for testing
        public static boolean motorTest = false;
        public static boolean servoTest = false;
        public static boolean distanceSensorTest = false;
        public static boolean colorSensorTest = false;
    }

    @Config
    public static class HardwareNames {

        public static String FLMOTOR = "flm";
        public static String FRMOTOR = "frm";
        public static String RLMOTOR = "rlm";
        public static String RRMOTOR = "rrm";
        public static String IMU = "imu";
        public static String TEST_MOTOR = "m";
        public static String TEST_SERVO = "s";
        public static String TEST_DISTANCE = "d";
        public static String TEST_COLOR = "c";
    }

    @Config
    public static class OtherSettings {

        public static double STICK_DEAD_ZONE = 0.1;
        public static int AUTOTIME = 25;
        public static double STICK_DEAD_ZONE = 0.08;
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
