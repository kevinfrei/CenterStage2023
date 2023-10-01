package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutoConstantsRed {
    /*
    public static class Wing {
        public static Pose2d START = new Pose2d(36, -66, toRadians(90));
        public static Pose2d STACK = new Pose2d(60, -12, toRadians(0));

        public static Pose2d PARK_LEFT = new Pose2d(12, -36, toRadians(90));
        public static Pose2d PARK_MIDDLE = new Pose2d(36, -36, toRadians(-90));
        public static Pose2d PARK_RIGHT = new Pose2d(60, -36, toRadians(180));

        public static Pose2d W_JUNCTION = new Pose2d(27, -5, toRadians(120));
        public static Pose2d S_JUNCTION = new Pose2d(4, -28, toRadians(135));

        //right facing between
        public static Pose2d W_JUNCTION_TO_BETWEEN = new Pose2d(34, -15, toRadians(0));
        //left facing between
        public static Pose2d STACK_TO_BETWEEN = new Pose2d(37, -12, toRadians(180));

        public static Pose2d LOW_TO_BETWEEN = new Pose2d(33,-60,toRadians(165));

        public static Pose2d LOW_JUNCTION_LEFT = new Pose2d(26,-52, toRadians(120));
        public static Pose2d LOW_JUNCTION_RIGHT = new Pose2d(48,-24, toRadians(60));


        // These are 'trajectory pieces' which should be named like this:
        // {STARTING_POSITION}_TO_{ENDING_POSITION}
        public static double MAX_VEL = 50;
        public static double MAX_ACCEL = 40;
        public static double MAX_ANG_VEL = Math.toRadians(180);
        public static double MAX_ANG_ACCEL = Math.toRadians(120);
        public static double TRACK_WIDTH = 9.5;

        public static MinVelocityConstraint MIN_VEL = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        public static ProfileAccelerationConstraint PROF_ACCEL = new ProfileAccelerationConstraint(MAX_ACCEL);
        public static Function<Pose2d, TrajectoryBuilder> function = pose -> new TrajectoryBuilder(pose, MIN_VEL, PROF_ACCEL);
        public static Supplier<Trajectory>

                START_TO_W_JUNCTION =
                () -> function.apply(START).splineTo(W_JUNCTION.vec(), W_JUNCTION.getHeading()).build(),
        //START_TO_SOUTH_JUNCTION =
        // ()-> function.apply(START).lineToLinearHeading(SOUTH_JUNCTION).build(),
        W_JUNCTION_TO_STACK =
                () -> function.apply(W_JUNCTION).lineToLinearHeading(STACK).build(),
        //SOUTH_JUNCTION_TO_STACK = b->b.apply(JUNCTION).lineToLinearHeading(STACK).build(),
        STACK_TO_W_JUNCTION =
                () -> function.apply(STACK).lineToLinearHeading(W_JUNCTION).build(),
        //STACK_TO_SOUTH_JUNCTION =
        // ()->function.apply(STACK).lineToLinearHeading(JUNCTION).build(),
        W_JUNCTION_TO_JUNCTION_TO_BETWEEN =
                () -> function.apply(W_JUNCTION).lineToLinearHeading(W_JUNCTION_TO_BETWEEN).build(),

        STACK_TO_STACK_TO_BETWEEN =
                () -> function.apply(STACK).lineToLinearHeading(STACK_TO_BETWEEN).build(),
                BETWEEN_TO_STACK =
                        () -> function.apply(W_JUNCTION_TO_BETWEEN).lineToLinearHeading(STACK).build(),
                BETWEEN_TO_W_JUNCTION =
                        () -> function.apply(STACK_TO_BETWEEN).lineToLinearHeading(W_JUNCTION).build(),

        W_JUNCTION_TO_PARK_LEFT =
                () -> function.apply(W_JUNCTION).lineToLinearHeading(PARK_LEFT).build(),
                W_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(W_JUNCTION).lineToLinearHeading(PARK_MIDDLE).build(),
                W_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(W_JUNCTION).lineToLinearHeading(PARK_RIGHT).build(),


        //SOUTH_JUNCTION_TO_PARK_LEFT =
        // ()->function.apply(JUNCTION).lineToLinearHeading(PARK_LEFT).build()
        //SOUTH_JUNCTION_TO_PARK_MIDDLE =
        // ()->function.apply(JUNCTION).lineToLinearHeading(PARK_MIDDLE).build()
        //SOUTH_JUNCTION_TO_PARK_RIGHT =
        // ()->function.apply(JUNCTION).lineToLinearHeading(PARK_RIGHT).build()


        //Left Low Junction
        START_TO_LEFT_LOW =
                () -> function.apply(START).lineToLinearHeading(LOW_JUNCTION_LEFT).build(),

        LEFT_LOW_TO_BETWEEN_LEFT =
                () -> function.apply(LOW_JUNCTION_LEFT).lineToLinearHeading(LOW_TO_BETWEEN).build(),
        BETWEEN_TO_PARK_LEFT =
                () -> function.apply(LOW_TO_BETWEEN).splineTo(PARK_LEFT.vec(),PARK_LEFT.getHeading()).build(),

        LEFT_LOW_TO_PARK_MIDDLE =
                () -> function.apply(LOW_JUNCTION_LEFT).lineToLinearHeading(PARK_MIDDLE).build(),

        LEFT_LOW_TO_BETWEEN_RIGHT =
                () -> function.apply(LOW_JUNCTION_LEFT).lineToLinearHeading(PARK_MIDDLE).build(),

        //Right Low Junction
        START_TO_RIGHT_LOW =
                () -> function.apply(START).splineTo(LOW_JUNCTION_RIGHT.vec(), LOW_JUNCTION_RIGHT.getHeading()).build();


    }
     */
    public static class Wing {
        public static Pose2d START = new Pose2d(36, 57, toRadians(-90));
        public static Pose2d BACK = new Pose2d(36, 57, toRadians(180));
        public static Pose2d PARK_CENTER = new Pose2d(-58, 10, toRadians(0));
        //public static Pose2d PARK_MIDDLE = new Pose2d(-36, 36, toRadians(-90));
        public static Pose2d PARK_CORNER = new Pose2d(-58, 57, toRadians(180));
        public static Pose2d E_JUNCTION = new Pose2d(-28, -4, toRadians(13));
        public static Pose2d S_JUNCTION = new Pose2d(-4, -28, toRadians(45));
        // between goes backward while rotating
        public static Pose2d BETWEEN = new Pose2d(-47, 12, toRadians(180));
        public static Pose2d RIGHT_SPIKE = new Pose2d(22, 36, toRadians(-90));
        public static Pose2d CENTER_SPIKE = new Pose2d(36, 22, toRadians(-90));
        public static Pose2d LEFT_SPIKE = new Pose2d(54, 36, toRadians(-90));

        // These are 'trajectory pieces' which should be named like this:
        // {STARTING_POSITION}_TO_{ENDING_POSITION}
        public static double MAX_VEL = 50;
        public static double MAX_ACCEL = 40;
        public static double MAX_ANG_VEL = Math.toRadians(180);
        public static double MAX_ANG_ACCEL = Math.toRadians(120);
        public static double TRACK_WIDTH = 9.5;

        public static MinVelocityConstraint MIN_VEL = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        public static ProfileAccelerationConstraint PROF_ACCEL = new ProfileAccelerationConstraint(MAX_ACCEL);
        public static Function<Pose2d, TrajectoryBuilder> function = pose -> new TrajectoryBuilder(pose, MIN_VEL, PROF_ACCEL);
        public static Supplier<Trajectory>


                START_TO_CENTER_SPIKE =
                () -> function.apply(START)
//                        .splineTo(W_JUNCTION.vec(), W_JUNCTION.getHeading())
                        .lineToLinearHeading(CENTER_SPIKE)
                        .build(),
                CENTER_SPIKE_TO_BACK =
                        () -> function.apply(CENTER_SPIKE)
                                //.lineToLinearHeading(STACK)
                                .lineToLinearHeading(BACK)
                                .build(),
        //START_TO_S_JUNCTION=
        //   () -> function.apply(START).lineToLinearHeading().build()
        BACK_TO_PARK_CORNER =
                () -> function.apply(BACK)
                        //.lineToLinearHeading(STACK)
                        .lineToLinearHeading(PARK_CORNER)
                        .build(),
                STACK_TO_W_JUNCTION =
                        () -> function.apply(BACK)
                                .lineToLinearHeading(LEFT_SPIKE)
                                .build(),
        //STACK_TO_S_JUNCTION=
        //() -> function.apply(STACK).lineToLinearHeading().build(),
        CENTER_SPIKE_TO_RIGHT_SPIKE =
                () -> function.apply(CENTER_SPIKE)
                        .lineToLinearHeading(RIGHT_SPIKE)
                        .build(),
                W_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                W_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(LEFT_SPIKE)
                                .splineTo(PARK_CENTER.vec(), Math.toRadians(-90))
                                .build(),
                S_JUNCTION_TO_PARK_LEFT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                S_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                S_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                E_JUNCTION_TO_BETWEEN =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(BETWEEN)
                                .build(),
                W_JUNCTION_TO_BETWEEN =
                        () -> function.apply(LEFT_SPIKE)
                                .lineToLinearHeading(BETWEEN)
                                .build(),
                BETWEEN_TO_STACK =
                        () -> function.apply(BETWEEN)
                                .lineToLinearHeading(BACK)
                                .build(),
                STACK_TO_BETWEEN =
                        () -> function.apply(BACK)
                                .lineToLinearHeading(CENTER_SPIKE)
                                .build(),
                BETWEEN_TO_W_JUNCTION =
                        () -> function.apply(CENTER_SPIKE)
                                .lineToLinearHeading(LEFT_SPIKE)
                                .build(),
                START_TO_BETWEEN3 =
                        () -> function.apply(START)
                                .lineToLinearHeading(CENTER_SPIKE)
                                .build(),
                BETWEEN3_TO_W_JUNCTION =
                        () -> function.apply(CENTER_SPIKE)
                                .lineToLinearHeading(LEFT_SPIKE)
                                .build();
    }

    public static class Stage {
        public static Pose2d START = new Pose2d(-13, 57, toRadians(-90));
        public static Pose2d STACK = new Pose2d(-62, 12, toRadians(180));
        public static Pose2d PARK_CENTER = new Pose2d(-58, 10, toRadians(0));
        //public static Pose2d PARK_MIDDLE = new Pose2d(-36, 36, toRadians(-90));
        public static Pose2d PARK_CORNER = new Pose2d(-60, -60, toRadians(-90));
        public static Pose2d E_JUNCTION = new Pose2d(-28, -4, toRadians(13));
        public static Pose2d S_JUNCTION = new Pose2d(-4, -28, toRadians(45));
        // between goes backward while rotating
        public static Pose2d BETWEEN = new Pose2d(-47, 12, toRadians(180));
        public static Pose2d RIGHT_SPIKE = new Pose2d(-20, 36, toRadians(-90));
        public static Pose2d CENTER_SPIKE = new Pose2d(-13, 22, toRadians(-90));
        public static Pose2d LEFT_SPIKE = new Pose2d(-1, 36, toRadians(-90));

        // These are 'trajectory pieces' which should be named like this:
        // {STARTING_POSITION}_TO_{ENDING_POSITION}
        public static double MAX_VEL = 50;
        public static double MAX_ACCEL = 40;
        public static double MAX_ANG_VEL = Math.toRadians(180);
        public static double MAX_ANG_ACCEL = Math.toRadians(120);
        public static double TRACK_WIDTH = 9.5;

        public static MinVelocityConstraint MIN_VEL = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        public static ProfileAccelerationConstraint PROF_ACCEL = new ProfileAccelerationConstraint(MAX_ACCEL);
        public static Function<Pose2d, TrajectoryBuilder> function = pose -> new TrajectoryBuilder(pose, MIN_VEL, PROF_ACCEL);
        public static Supplier<Trajectory>


                START_TO_CENTER_SPIKE =
                () -> function.apply(START)
//                        .splineTo(W_JUNCTION.vec(), W_JUNCTION.getHeading())
                        .lineToLinearHeading(CENTER_SPIKE)
                        .build(),
                CENTER_SPIKE_TO_START =
                        () -> function.apply(CENTER_SPIKE)
                                //.lineToLinearHeading(STACK)
                                .lineToLinearHeading(START)
                                .build(),
        //START_TO_S_JUNCTION=
        //   () -> function.apply(START).lineToLinearHeading().build()
        START_TO_PARK_CENTER =
                () -> function.apply(START)
                        //.lineToLinearHeading(STACK)
                        .lineToLinearHeading(PARK_CENTER)
                        .build(),
                STACK_TO_W_JUNCTION =
                        () -> function.apply(STACK)
                                .lineToLinearHeading(LEFT_SPIKE)
                                .build(),
        //STACK_TO_S_JUNCTION=
        //() -> function.apply(STACK).lineToLinearHeading().build(),
        CENTER_SPIKE_TO_RIGHT_SPIKE =
                () -> function.apply(CENTER_SPIKE)
                        .lineToLinearHeading(RIGHT_SPIKE)
                        .build(),
                W_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                W_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(LEFT_SPIKE)
                                .splineTo(PARK_CENTER.vec(), Math.toRadians(-90))
                                .build(),
                S_JUNCTION_TO_PARK_LEFT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                S_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                S_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_CENTER)
                                .build(),
                E_JUNCTION_TO_BETWEEN =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(BETWEEN)
                                .build(),
                W_JUNCTION_TO_BETWEEN =
                        () -> function.apply(LEFT_SPIKE)
                                .lineToLinearHeading(BETWEEN)
                                .build(),
                BETWEEN_TO_STACK =
                        () -> function.apply(BETWEEN)
                                .lineToLinearHeading(STACK)
                                .build(),
                STACK_TO_BETWEEN =
                        () -> function.apply(STACK)
                                .lineToLinearHeading(CENTER_SPIKE)
                                .build(),
                BETWEEN_TO_W_JUNCTION =
                        () -> function.apply(CENTER_SPIKE)
                                .lineToLinearHeading(LEFT_SPIKE)
                                .build(),
                START_TO_BETWEEN3 =
                        () -> function.apply(START)
                                .lineToLinearHeading(CENTER_SPIKE)
                                .build(),
                BETWEEN3_TO_W_JUNCTION =
                        () -> function.apply(CENTER_SPIKE)
                                .lineToLinearHeading(LEFT_SPIKE)
                                .build();
    }
}