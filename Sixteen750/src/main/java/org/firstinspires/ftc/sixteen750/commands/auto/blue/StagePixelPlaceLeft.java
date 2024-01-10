package org.firstinspires.ftc.sixteen750.commands.auto.blue;

import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.path.command.TrajectorySequenceCommand;

import org.firstinspires.ftc.sixteen750.AutoConstants.StageBlue;
import org.firstinspires.ftc.sixteen750.Robot;

public class StagePixelPlaceLeft extends SequentialCommandGroup {

    public StagePixelPlaceLeft(Robot r) {
        super(
            new TrajectorySequenceCommand(r.drivebase, StageBlue.START_TO_LEFT_SPIKE)
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        StageBlue.LEFT_SPIKE_TO_CLEAR
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        StageBlue.CLEAR_TO_LEFT_CLEAR
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        StageBlue.LEFT_CLEAR_TO_PLACE_LEFT
                    )
                )
                .andThen(
                    new TrajectorySequenceCommand(
                        r.drivebase,
                        StageBlue.PLACE_LEFT_TO_MID_PARK_CENTER
                    )
                )
        );
    }
}
