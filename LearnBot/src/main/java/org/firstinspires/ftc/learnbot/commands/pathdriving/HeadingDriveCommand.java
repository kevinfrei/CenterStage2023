package org.firstinspires.ftc.learnbot.commands.pathdriving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.technototes.library.command.Command;
import com.technototes.library.control.Stick;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.util.MathUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.learnbot.Setup;
import org.firstinspires.ftc.learnbot.helpers.PIDFController;
import org.firstinspires.ftc.learnbot.subsystems.PathFollowingSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class HeadingDriveCommand implements Command, Loggable {

    public PathFollowingSubsystem subsystem;

    // These are the 3 axes we watch:
    public DoubleSupplier x, y, r;

    // This is the button pressed to snap to the closest right angle
    public BooleanSupplier straight;

    @Log(name = "targetHeading")
    public double targetHeadingDeg;

    @Log(name = "heading")
    public double curHeadingDeg;

    public ElapsedTime lastRead;
    // This should be true when we have been rotating due to user input
    // When it changes from true to false, we should set the targetHeading to curHeading,
    // but *only* when it changes from true to false!
    public boolean lastRotateChange;

    PIDFController rotationalPid;

    // These values are just flat out wrong. I pulled them out of thin air.
    public static PIDCoefficients ROTATE_PID = new PIDCoefficients(1e-2, 1e-3, 1e-4);

    public HeadingDriveCommand(
        PathFollowingSubsystem sub,
        Stick xyStick,
        Stick rotStick,
        BooleanSupplier straighten
    ) {
        addRequirements(sub);
        subsystem = sub;
        x = xyStick.getXSupplier();
        y = xyStick.getYSupplier();
        r = rotStick.getXSupplier();
        straight = straighten;
        targetHeadingDeg = Math.toDegrees(sub.getExternalHeading());
        curHeadingDeg = targetHeadingDeg;
        lastRead = new ElapsedTime();
        lastRotateChange = false;
        rotationalPid = new PIDFController(ROTATE_PID);
        rotationalPid.setOutputBounds(-1, 1);
    }

    // Use this constructor if you don't want auto-straightening
    public HeadingDriveCommand(PathFollowingSubsystem sub, Stick xyStick, Stick rotStick) {
        this(sub, xyStick, rotStick, () -> false);
    }

    // This will make the bot snap to an angle, if the 'straighten' button is pressed
    // Otherwise, it tries to track a target heading, or reads the user rotation
    // and adjusts the target heading accordingly.
    private double getRotation(double headingInRads) {
        // headingInRads should be [0-2pi]
        // headingDeg is +/-180, cuz that's just easier for my weak brain...
        double headingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(headingInRads));

        // Get the amount of time passed since we last read the rotation value from the user
        double lastReadDelta = getTimeSinceLastRead();

        // Check to see if we're trying to straighten the robot
        if (straight.getAsBoolean()) {
            // Snap to the closest 90 degree angle
            // There's some annoying wrap-around stuff to deal with here that I haven't yet:
            rotationalPid.targetPosition = MathUtils.closestTo(headingDeg, -180, -90, 0, 90, 180);
            // indicate that the user is doing something
            // (so that when the user *stops* doing something, we'll stop the bot from rotating)
            this.lastRotateChange = true;
        } else {
            double rotationInput = r.getAsDouble();
            boolean userInput = (Math.abs(rotationInput) > Setup.OtherSettings.STICK_DEAD_ZONE);
            if (!userInput) {
                rotationInput = 0;
                if (this.lastRotateChange) {
                    // If the user just moved to "zero",
                    // then set the target heading to the current heading.
                    // Basically, stop rotating.
                    this.targetHeadingDeg = headingDeg;
                }
            }
            // Calculate the angle change, based on the rotational speed settings over time
            double changeDegrees =
                rotationInput * lastReadDelta * Setup.HeadingSettings.DEGREES_PER_SECOND;
            if (!userInput && this.lastRotateChange) {
                this.targetHeadingDeg = headingDeg;
            }
            this.lastRotateChange = userInput;
            rotationalPid.targetPosition = this.targetHeadingDeg + changeDegrees;
        }
        return rotationalPid.update(this.curHeadingDeg);
    }

    // Return the fraction of a second since we last read user input
    private double getTimeSinceLastRead() {
        double sinceLastRead = this.lastRead.seconds();
        this.lastRead.reset(); // restart the timer for our "rotation per time" calculation
        // If it's been a long time since we last read input, and we get *new* input,
        // return a small value so the bot doesn't jerk in the direction indicated
        if (sinceLastRead > Setup.HeadingSettings.LONGEST_DELAY) {
            sinceLastRead = Setup.HeadingSettings.FIRST_CHANGE; // first time reading the stick since op start
        }
        return sinceLastRead;
    }

    @Override
    public void execute() {
        // If subsystem is busy it is running a trajectory.
        if (!subsystem.isBusy()) {
            double curHeading = subsystem.getExternalHeading();

            // The math & signs looks wonky, because this makes things field-relative
            // (Remember that "3 O'Clock" is zero degrees)
            Vector2d input = new Vector2d(
                -y.getAsDouble() * subsystem.speed,
                -x.getAsDouble() * subsystem.speed
            )
                .rotated(-curHeading);
            subsystem.setWeightedDrivePower(
                new Pose2d(input.getX(), input.getY(), getRotation(curHeading))
            );
        }
        subsystem.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean cancel) {
        if (cancel) subsystem.setDriveSignal(new DriveSignal());
    }
}
