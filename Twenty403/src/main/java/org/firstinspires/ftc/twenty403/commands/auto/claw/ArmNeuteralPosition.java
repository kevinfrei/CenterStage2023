package org.firstinspires.ftc.twenty403.commands.auto.claw;


import com.technototes.library.command.Command;
import org.firstinspires.ftc.twenty403.subsystems.ClawSubsystem;
public class ArmNeuteralPosition implements Command {

    private ClawSubsystem subsystem;
    public ArmNeuteralPosition (ClawSubsystem n) {
        subsystem = n;
        addRequirements(n);

    }
    @Override
    public void execute()  {
        subsystem.neutralArmPosition();
    }
}
