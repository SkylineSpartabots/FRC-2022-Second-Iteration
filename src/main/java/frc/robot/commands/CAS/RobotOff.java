package frc.robot.commands.CAS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotOff extends CommandBase {
    public RobotOff() {
        addRequirements(
            HoodSubsystem.getInstance(), 
            ShooterSubsystem.getInstance(),
            IndexerSubsystem.getInstance()
            );
    }

    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShooterPercentPower(0);
        HoodSubsystem.getInstance().CASIsInactive();
        HoodSubsystem.getInstance().stopHood();
        IndexerSubsystem.getInstance().setIndexerPercentPower(0, false);
        IndexerSubsystem.getInstance().setIntakePercentPower(0, false);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
