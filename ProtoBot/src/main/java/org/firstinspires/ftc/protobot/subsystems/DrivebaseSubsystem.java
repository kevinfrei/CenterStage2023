package org.firstinspires.ftc.protobot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.technototes.library.command.Command;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.subsystem.drivebase.SimpleMecanumDrivebaseSubsystem;
import com.technototes.path.subsystem.MecanumConstants;
import com.technototes.path.subsystem.PathingMecanumDrivebaseSubsystem;
import java.util.function.Supplier;

public class DrivebaseSubsystem
    extends PathingMecanumDrivebaseSubsystem
    implements Supplier<Pose2d>, Loggable {

    @Override
    public Pose2d get() {
        return null;
    }

    @Config
    public abstract static class DriveConstants implements MecanumConstants {

        public static double SLOW_MOTOR_SPEED = 0.6;
        public static double FAST_MOTOR_SPEED = 1.0;
        public static double AUTO_MOTOR_SPEED = 0.9;
        public static double TRIGGER_THRESHOLD = 0.7;

        @TicksPerRev
        public static final double TICKS_PER_REV = 385.6; // previous: 537.6

        @MaxRPM
        public static final double MAX_RPM = 435; // 2021: 6000;

        public static double MAX_TICKS_PER_SEC = (TICKS_PER_REV * MAX_RPM) / 60.0;

        @UseDriveEncoder
        public static final boolean RUN_USING_ENCODER = true;

        @MotorVeloPID
        public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(
            20,
            0,
            3,
            MecanumConstants.getMotorVelocityF((MAX_RPM / 60) * TICKS_PER_REV)
        );

        @WheelRadius
        public static double WHEEL_RADIUS = 1.88976; // in

        @GearRatio
        public static double GEAR_RATIO = 0.6; // output (wheel) speed / input (motor) speed og: 1 / 19.2;

        //gear ration is actually .667 but i think it might mess up what we already have
        @TrackWidth
        public static double TRACK_WIDTH = 11.75; // 2021: 10; // in

        @WheelBase
        public static double WHEEL_BASE = 8.5; // in

        @KV
        public static double kV =
            1.0 / MecanumConstants.rpmToVelocity(MAX_RPM, WHEEL_RADIUS, GEAR_RATIO);

        @KA
        public static double kA = 0;

        @KStatic
        public static double kStatic = 0;

        // This was 60, which was too fast. Things slid around a lot.
        @MaxVelo
        public static double MAX_VEL = 50;

        // This was 35, which also felt a bit too fast. The bot controls more smoothly now
        @MaxAccel
        public static double MAX_ACCEL = 30; //30

        // This was 180 degrees
        @MaxAngleVelo
        public static double MAX_ANG_VEL = Math.toRadians(180);

        // This was 90 degrees
        @MaxAngleAccel
        public static double MAX_ANG_ACCEL = Math.toRadians(90);

        @TransPID
        public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);

        @HeadPID
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

        @LateralMult
        public static double LATERAL_MULTIPLIER = 1.0; // Lateral position is off by 14%

        @VXWeight
        public static double VX_WEIGHT = 1;

        @VYWeight
        public static double VY_WEIGHT = 1;

        @OmegaWeight
        public static double OMEGA_WEIGHT = 1;

        @PoseLimit
        public static int POSE_HISTORY_LIMIT = 100;

        // FL - 0.82
        // FR - 0.8
        // RL - 0.1
        // RR - 0.74
        public static double AFR_SCALE = 0.9;
        public static double AFL_SCALE = 0.9;
        public static double ARR_SCALE = 0.9;
        public static double ARL_SCALE = 0.9;
    }

    private static final boolean ENABLE_POSE_DIAGNOSTICS = true;

    @Log(name = "Pose2d: ")
    public String poseDisplay = ENABLE_POSE_DIAGNOSTICS ? "" : null;

    @Log.Number(name = "FL")
    public EncodedMotor<DcMotorEx> fl2;

    @Log.Number(name = "FR")
    public EncodedMotor<DcMotorEx> fr2;

    @Log.Number(name = "RL")
    public EncodedMotor<DcMotorEx> rl2;

    @Log.Number(name = "RR")
    public EncodedMotor<DcMotorEx> rr2;

    @Log(name = "Turbo")
    public boolean Turbo = false;

    @Log(name = "Snail")
    public boolean Snail = false;

    @Log
    public String locState = "none";

    @Log(name = "cur heading")
    double curHeading;

    public DrivebaseSubsystem(
        IMU imu,
        EncodedMotor<DcMotorEx> flMotor,
        EncodedMotor<DcMotorEx> frMotor,
        EncodedMotor<DcMotorEx> rlMotor,
        EncodedMotor<DcMotorEx> rrMotor
    ) {
        super(flMotor, frMotor, rlMotor, rrMotor, imu, () -> DriveConstants.class);
        curHeading = imu.gyroHeading();
    }

    @Override
    public void register() {
        super.register();
    }

    @Override
    public Subsystem setDefaultCommand(Command c) {
        return super.setDefaultCommand(c);
    }

    @Override
    public Command getDefaultCommand() {
        return super.getDefaultCommand();
    }

    @Override
    public void periodic() {
        curHeading = this.getExternalHeading();
    }
}
