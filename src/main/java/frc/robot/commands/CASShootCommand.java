package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
    }

    //hood values span from 0 (reset positon) to -17,000
    int maxHoodPosition = -17000;

    @Override
    public void execute(){
      double distanceFromHub = calculateDistance(
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
      double minHubDistance = 1.5;
      double maxHubDistance = 5.0;

      double hoodSlope = -6316.0;
      double hoodIntercept = 7317.0;
      double targetHoodPosition = hoodSlope * distance + hoodIntercept;

      double shooterA = 36.70;
      double shooterExp = 3.105;
      double shooterIntercept = 9868.0;
      double targetShooterVelocity = shooterA * Math.pow(distance, shooterExp) + shooterIntercept;
      if(targetShooterVelocity < 10000) targetShooterVelocity = 10000;
      else if(targetShooterVelocity > 14500) targetShooterVelocity = 14500;

      
      if(targetHoodPosition > 0) targetHoodPosition = 0;
      else if(targetHoodPosition < -15000) {
        targetHoodPosition = 0;
        targetShooterVelocity = 0;
      }
/*
      //if distance invalid, slow down
      if(distance > maxHubDistance || distance < minHubDistance) {
        targetHoodPosition = 0;
        targetShooterVelocity = 0;
      }*/

      m_hood.moveHoodToPosition((int)targetHoodPosition);
      m_shooter.setShooterVelocity((int)targetShooterVelocity);

      SmartDashboard.putNumber("Hood Target", targetHoodPosition);
      SmartDashboard.putNumber("Shooter Target", targetShooterVelocity);
      SmartDashboard.putNumber("Distance Away", distance);

    }



    private double calculateDistance(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2));
    }

}
