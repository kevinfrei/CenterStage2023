package org.firstinspires.ftc.learnbot.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.learnbot.Hardware;
import org.firstinspires.ftc.learnbot.Robot;
import org.firstinspires.ftc.learnbot.commands.pathdriving.ResetGyroCommand;
import org.firstinspires.ftc.learnbot.controllers.DriverController;
import org.firstinspires.ftc.learnbot.helpers.StartingPosition;

@TeleOp(name = "Just Driving")
@SuppressWarnings("unused")
public class JustDrivingTeleOp extends CommandOpMode {

    public Robot robot;
    public DriverController controlsDriver;
    // public ControlOperator controlsOperator;
    public Hardware hardware;

    @Override
    public void uponInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware = new Hardware(hardwareMap);
        robot = new Robot(hardware);
        controlsDriver = new DriverController(driverGamepad, robot);
        // controlsOperator = new ControlOperator(codriverGamepad, robot);
        robot.pathingSubsystem.setPoseEstimate(new Pose2d(0, 0, 0));

        // When we start an opmode, we reset the gyro
        CommandScheduler
            .getInstance()
            .scheduleForState(new ResetGyroCommand(robot.pathingSubsystem), OpModeState.INIT);
    }
}
