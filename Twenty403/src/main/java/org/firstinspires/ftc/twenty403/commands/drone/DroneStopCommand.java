package org.firstinspires.ftc.twenty403.commands.drone;

import com.technototes.library.command.Command;
import org.firstinspires.ftc.twenty403.subsystems.DroneSubsystem;

public class DroneStopCommand implements Command {

    private DroneSubsystem subsystem;

    public DroneStopCommand(DroneSubsystem s) {
        this.subsystem = s;
        addRequirements(this.subsystem); // Keeps robot from breaking
    }

    @Override
    public void execute() {
        this.subsystem.unlaunch();
    }
}
