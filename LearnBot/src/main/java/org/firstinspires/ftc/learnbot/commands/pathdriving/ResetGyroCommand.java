package org.firstinspires.ftc.learnbot.commands.pathdriving;

import com.technototes.library.command.Command;
import org.firstinspires.ftc.learnbot.subsystems.PathFollowingSubsystem;

public class ResetGyroCommand implements Command {

    public PathFollowingSubsystem subsystem;

    public ResetGyroCommand(PathFollowingSubsystem s) {
        subsystem = s;
    }

    @Override
    public void execute() {
        subsystem.setExternalHeading(0);
    }
}
