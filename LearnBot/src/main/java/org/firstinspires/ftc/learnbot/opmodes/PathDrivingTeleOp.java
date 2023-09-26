package org.firstinspires.ftc.learnbot.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.logger.Loggable;
import com.technototes.library.structure.CommandOpMode;
import org.firstinspires.ftc.learnbot.Hardware;
import org.firstinspires.ftc.learnbot.Robot;
import org.firstinspires.ftc.learnbot.commands.pathdriving.HeadingDriveCommand;
import org.firstinspires.ftc.learnbot.commands.pathdriving.ResetGyroCommand;
import org.firstinspires.ftc.learnbot.controllers.PathController;

@TeleOp(name = "Heading Driving")
@SuppressWarnings("unused")
public class PathDrivingTeleOp extends CommandOpMode implements Loggable {

    public Robot robot;
    public PathController controlsDriver;
    // public ControlOperator controlsOperator;
    public Hardware hardware;

    public HeadingDriveCommand drivingCommand;

    @Override
    public void uponInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware = new Hardware(hardwareMap);
        robot = new Robot(hardware);

        // The Controller wires up all the commands that respond to the controller
        controlsDriver = new PathController(driverGamepad, robot);

        robot.pathingSubsystem.setPoseEstimate(new Pose2d(0, 0, 0));

        // When we start the opmode, we reset the gyro
        CommandScheduler
            .getInstance()
            .scheduleForState(new ResetGyroCommand(robot.pathingSubsystem), OpModeState.INIT);
    }
}
