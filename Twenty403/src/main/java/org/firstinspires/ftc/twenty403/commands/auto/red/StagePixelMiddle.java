package org.firstinspires.ftc.twenty403.commands.auto.red;

import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.path.command.TrajectorySequenceCommand;
import org.firstinspires.ftc.twenty403.AutoConstants;
import org.firstinspires.ftc.twenty403.Robot;

public class StagePixelMiddle extends SequentialCommandGroup {

    public StagePixelMiddle(Robot r) {
        super(
        new TrajectorySequenceCommand(r.drivebaseSubsystem, AutoConstants.StageRed.START_TO_MIDDLE_SPIKE),
        new TrajectorySequenceCommand(r.drivebaseSubsystem, AutoConstants.StageRed.MIDDLE_SPIKE_TO_CLEAR),
        new TrajectorySequenceCommand(r.drivebaseSubsystem, AutoConstants.StageRed.CLEAR_TO_MID_PARK_CENTER),
        new TrajectorySequenceCommand(r.drivebaseSubsystem, AutoConstants.StageRed.MID_PARK_CENTER_TO_PARK_CENTER)
        );
    }
}
