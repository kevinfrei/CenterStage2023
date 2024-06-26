package org.firstinspires.ftc.sixteen750.commands.auto.blue;

import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import com.technototes.path.command.TrajectorySequenceCommand;
import org.firstinspires.ftc.sixteen750.AutoConstants;
import org.firstinspires.ftc.sixteen750.Robot;
import org.firstinspires.ftc.sixteen750.commands.auto.PixelScoring;

public class StagePixelPlaceMiddle extends SequentialCommandGroup {

    public StagePixelPlaceMiddle(Robot r) {
        super(
            new TrajectorySequenceCommand(
                r.drivebase,
                AutoConstants.StageBlue.START_TO_MIDDLE_SPIKE
            ),
            new TrajectorySequenceCommand(
                r.drivebase,
                AutoConstants.StageBlue.MIDDLE_SPIKE_TO_CLEAR
            ),
            new TrajectorySequenceCommand(r.drivebase, AutoConstants.StageBlue.ClEAR_TO_LEFT_CLEAR),
            new TrajectorySequenceCommand(
                r.drivebase,
                AutoConstants.StageBlue.LEFT_CLEAR_TO_PLACE_MIDDLE
            ),
            //place command
            new PixelScoring(r.placement),
            new TrajectorySequenceCommand(
                r.drivebase,
                AutoConstants.StageBlue.PLACE_MIDDLE_TO_MID_PARK_CENTER
            )
        );
    }
}
