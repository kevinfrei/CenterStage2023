package org.firstinspires.ftc.learnbot.commands;

import com.technototes.library.command.Command;
import com.technototes.library.command.SimpleRequiredCommand;
import org.firstinspires.ftc.learnbot.subsystems.PlacementSubsystem;

public class Lift {

    public static Command HighCommand(PlacementSubsystem ps) {
        return new SimpleRequiredCommand<>(ps, PlacementSubsystem::liftHeightHigh);
    }

    public static Command MediumCommand(PlacementSubsystem ps) {
        return new SimpleRequiredCommand<>(ps, PlacementSubsystem::liftHeightMedium);
    }

    public static Command LowCommand(PlacementSubsystem ps) {
        return new SimpleRequiredCommand<>(ps, PlacementSubsystem::liftHeightLow);
    }
}
