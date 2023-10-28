package org.firstinspires.ftc.twenty403;

import com.technototes.library.logger.Loggable;
import com.technototes.library.util.Alliance;
import java.util.Set;
import org.firstinspires.ftc.twenty403.helpers.StartingPosition;
import org.firstinspires.ftc.twenty403.subsystems.ClawSubsystem;
import org.firstinspires.ftc.twenty403.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.twenty403.subsystems.DroneSubsystem;
import org.firstinspires.ftc.twenty403.subsystems.VisionSubsystem;

public class Robot implements Loggable {

    public StartingPosition position;
    public Alliance alliance;
    public double initialVoltage;

    public DrivebaseSubsystem drivebaseSubsystem;
    public ClawSubsystem clawSubsystem;
    public VisionSubsystem vision;
    public DroneSubsystem droneSubsystem;

    public Robot(Hardware hw, Alliance team, StartingPosition pos) {
        this.position = pos;
        this.alliance = team;
        this.initialVoltage = hw.voltage();
        if (Setup.Connected.DRIVEBASE) {
            this.drivebaseSubsystem = new DrivebaseSubsystem(hw.fl, hw.fr, hw.rl, hw.rr, hw.imu);
        }

        if (Setup.Connected.WEBCAM) {
            this.vision = new VisionSubsystem(hw.camera, team, pos);
        } else {
            this.vision = new VisionSubsystem();
        }
        if (Setup.Connected.CLAWSUBSYSTEM) {
            this.clawSubsystem = new ClawSubsystem(hw.clawServo, hw.elbowServo, hw.swingMotor);
        } else {
            this.clawSubsystem = new ClawSubsystem();
        }
        if (Setup.Connected.DRONESUBSYSTEM) {
            this.droneSubsystem = new DroneSubsystem(hw.launchServo);
        } else {
            this.droneSubsystem = new DroneSubsystem();
        }
    }
}
