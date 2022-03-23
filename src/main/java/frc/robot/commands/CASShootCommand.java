package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CASShootCommand extends CommandBase {
    private final HoodSubsystem m_hood;
    private final ShooterSubsystem m_shooter;
    private final DrivetrainSubsystem m_drivetrain;


    public CASShootCommand() {
        m_hood = HoodSubsystem.getInstance();
        m_shooter = ShooterSubsystem.getInstance();
        m_drivetrain = DrivetrainSubsystem.getInstance();
        addRequirements(m_shooter, m_hood);
    }

    @Override
    public void initialize() {
        m_hood.resetHoodPosition();
        m_hood.CASIsActive();
    }

    //hood values span from 0 (reset positon) to -17,000
    int maxHoodPosition = -17000;

    @Override
    public void execute(){
      double distanceFromHub = DrivetrainSubsystem.calculateDistance(
        m_drivetrain.getPose().getX(), m_drivetrain.getPose().getY(), Constants.targetHudPosition.getX(),Constants.targetHudPosition.getY());
      calculateShooterHood(distanceFromHub);
    }

    //hood position formula, y = hood position, x = distance away:
    //y = -6316 * x + 7317
    //limit: hood cannot be greater than 0 or less than -15,000

    //shooter velocity formula, y = shooter velocity, x = distance away:
    //y = 36.70 * (x ^ 3.105) + 9868
    //limit: hood velocity cannot be less than 10,000 or more than 14,500

    //CALIBRATE THESE VALUES
    //shooter limits: cannot shoot if greater than 5 meters away
    //if greater than 5 meters away, reset hood position and turn off shooter
    
    private void calculateShooterHood(double distance){
      double minHubDistance = 2;
      double maxHubDistance = 6.0;

      double hoodSlope = -10309.0;
      double hoodIntercept = 21041.0;
      double targetHoodPosition = hoodSlope * distance + hoodIntercept;

      double shooterSlope = 735.8;
      double shooterIntercept = 6157.0;
      double targetShooterVelocity = shooterSlope * distance + shooterIntercept;
      /*double shooterA = 36.70;
      double shooterExp = 3.105;
      double shooterIntercept = 9868.0;
      double targetShooterVelocity = shooterA * Math.pow(distance, shooterExp) + shooterIntercept;*/
      if(targetShooterVelocity > 11000) targetShooterVelocity = 11000;
      //else if(targetShooterVelocity > 14500) targetShooterVelocity = 14500;

      
      if(targetHoodPosition > 0) targetHoodPosition = 0;
      else if(targetHoodPosition < -38000) {
        targetHoodPosition = -38000;
        //CANNOT SHOOT IF THIS IS CALLED
      }

      boolean shootable = true;
      //if distance invalid, slow down
      if(distance > maxHubDistance || distance < minHubDistance) {
          shootable = false;
      }

      m_hood.moveHoodToPosition((int)targetHoodPosition);
      m_shooter.setShooterVelocity((int)targetShooterVelocity);

      SmartDashboard.putNumber("Hood Target", targetHoodPosition);
      SmartDashboard.putNumber("Shooter Target", targetShooterVelocity);
      SmartDashboard.putNumber("Distance Away", distance);
      SmartDashboard.putBoolean("Shootable?", shootable);

    }



    @Override
    public void end(boolean interruptable){
      m_hood.CASIsInactive();
    }

}
