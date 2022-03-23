package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class CalibrationCommand extends CommandBase {
    Pose2d position;
    public CalibrationCommand(Pose2d pos) {
        position = pos;
        addRequirements(HoodSubsystem.getInstance(),DrivetrainSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        HoodSubsystem.getInstance().resetHoodPosition();
        DrivetrainSubsystem.getInstance().resetOdometryFromPosition(position);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
