package org.firstinspires.ftc.learnbot.commands;

import com.technototes.library.command.Command;
import com.technototes.library.command.SimpleCommand;
import org.firstinspires.ftc.learnbot.subsystems.MotorTestSubsystem;

public class AnalogMotor {

    public static Command IncrementCommand(MotorTestSubsystem motor) {
        return new SimpleCommand(motor::motorInc);
    }

    public static Command DecrementCommand(MotorTestSubsystem motor) {
        return new SimpleCommand(motor::motorDec);
    }

    public static Command ToggleControlCommand(MotorTestSubsystem motor) {
        return new SimpleCommand(motor::toggleMotorControlMode);
    }

    public static Command ToggleStopModeCommand(MotorTestSubsystem motor) {
        return new SimpleCommand(motor::toggleMotorStopMode);
    }
}
