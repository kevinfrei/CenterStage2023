package org.firstinspires.ftc.sixteen750.commands.auto.blue;

import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.path.command.TrajectorySequenceCommand;
import org.firstinspires.ftc.sixteen750.AutoConstants;
import org.firstinspires.ftc.sixteen750.Robot;

public class StagePixelRight extends SequentialCommandGroup {

    public StagePixelRight(Robot r) {
        super(
            new TrajectorySequenceCommand(r.drivebase, AutoConstants.StageBlue.START_TO_MID_CLEAR)
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        AutoConstants.StageBlue.MID_CLEAR_TO_RIGHT_SPIKE
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        AutoConstants.StageBlue.RIGHT_SPIKE_TO_MID_CLEAR
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        AutoConstants.StageBlue.MID_CLEAR_TO_CLEAR
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        AutoConstants.StageBlue.CLEAR_TO_LEFT_CLEAR
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        AutoConstants.StageBlue.LEFT_CLEAR_TO_MID_PARK_CENTER
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        AutoConstants.StageBlue.MID_PARK_CENTER_TO_PARK_CENTER
                    )
                )
        );
    }
}
