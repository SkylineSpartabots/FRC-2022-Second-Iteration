package frc.robot.commands.CAS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotIdle extends CommandBase {


    public RobotIdle() {
      addRequirements(
          HoodSubsystem.getInstance(), 
          ShooterSubsystem.getInstance(),
          IndexerSubsystem.getInstance()
          );
    }

    @Override
    public void initialize() {        
        HoodSubsystem.getInstance().CASIsInactive();
        HoodSubsystem.getInstance().stopHood();
        ShooterSubsystem.getInstance().setShooterPercentPower(shooterIdle);
        IndexerSubsystem.getInstance().setIndexerPercentPower(indexerUp, true);
        IndexerSubsystem.getInstance().setIntakePercentPower(intakeOn, true);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}