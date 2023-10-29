package org.firstinspires.ftc.twenty403.commands.hang;

import com.technototes.library.command.Command;
import org.firstinspires.ftc.twenty403.subsystems.HangSubsystem;

public class HangDown implements Command {

    private HangSubsystem hang;

    public HangDown(HangSubsystem h) {
        hang = h;
        addRequirements(h);
    }

    @Override
    public void execute() {
        hang.hingeDown();
    }
}
