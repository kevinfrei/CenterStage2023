package org.firstinspires.ftc.learnbot.controllers;

import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.CommandButton;
import com.technototes.library.control.CommandGamepad;
import com.technototes.library.control.Stick;
import com.technototes.library.logger.Loggable;
import org.firstinspires.ftc.learnbot.Robot;
import org.firstinspires.ftc.learnbot.Setup;
import org.firstinspires.ftc.learnbot.commands.pathdriving.HeadingDriveCommand;
import org.firstinspires.ftc.learnbot.commands.pathdriving.ResetGyroCommand;

public class PathController implements Loggable {

    public Robot robot;
    public CommandGamepad gamepad;

    public Stick driveLeftStick, driveRightStick;
    public CommandButton resetGyroButton;
    public CommandButton straighten;
    public Command joystickDriving;

    public PathController(CommandGamepad g, Robot r) {
        this.robot = r;
        gamepad = g;
        AssignNamedControllerButton();
        if (Setup.Connected.PATHDRIVEBASE) {
            bindDriveControls();
        }
    }

    private void AssignNamedControllerButton() {
        resetGyroButton = gamepad.ps_options;
        driveLeftStick = gamepad.leftStick;
        driveRightStick = gamepad.rightStick;
        straighten = gamepad.ps_share;
        joystickDriving =
            new HeadingDriveCommand(
                robot.pathingSubsystem,
                driveLeftStick,
                driveRightStick,
                () -> straighten.getAsBoolean()
            );
    }

    public void bindDriveControls() {
        CommandScheduler.getInstance().scheduleJoystick(joystickDriving);
        /*
            turboButton.whenPressed(new TurboCommand(robot.drivebaseSubsystem));
            turboButton.whenReleased(new SlowCommand(robot.drivebaseSubsystem));
            */
        resetGyroButton.whenPressed(new ResetGyroCommand(robot.pathingSubsystem));
    }
}
