package org.firstinspires.ftc.learnbot.commands;

import com.technototes.library.command.Command;
import com.technototes.library.command.SimpleRequiredCommand;
import org.firstinspires.ftc.learnbot.subsystems.TestSubsystem;

public class Test {

    public static Command ServoLeft(TestSubsystem ts) {
        return new SimpleRequiredCommand<>(ts, TestSubsystem::servoLeft);
    }

    public static Command ServoRight(TestSubsystem ts) {
        return new SimpleRequiredCommand<>(ts, TestSubsystem::servoRight);
    }

    public static Command MotorForward(TestSubsystem ts) {
        return new SimpleRequiredCommand<>(ts, TestSubsystem::forwardSpinning);
    }

    public static Command MotorBackward(TestSubsystem ts) {
        return new SimpleRequiredCommand<>(ts, TestSubsystem::backwardSpinning);
    }

    public static Command MotorStop(TestSubsystem ts) {
        return new SimpleRequiredCommand<>(ts, TestSubsystem::stopSpinning);
    }
}
