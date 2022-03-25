package frc.robot.commands.CAS;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeIndexerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootByLimelight extends CommandBase {
    private final HoodSubsystem m_hood;
    private final ShooterSubsystem m_shooter;
    private boolean moveIndexer;
    private boolean shotBall = false;;


    public ShootByLimelight(boolean moveIndexer) {
        m_hood = HoodSubsystem.getInstance();
        m_shooter = ShooterSubsystem.getInstance();
        addRequirements(m_shooter, m_hood);
        this.moveIndexer = moveIndexer;
    }

    @Override
    public void initialize() {
        m_hood.resetHoodPosition();
    }


    @Override
    public void execute(){
      LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
      int shooterSpeed = 0;
      if(m_limelightSubsystem.getXOffset() == 0.000000){
        shooterSpeed = 9000;
      }
      else{
        shooterSpeed = calculateShooterSpeed(LimelightSubsystem.getInstance().getDistance());
      }
      
      m_shooter.setShooterVelocity(shooterSpeed);
    }
    
    private int calculateShooterSpeed(double distance){

      double shooterSlope = 1099;
      double shooterIntercept = 7000.0;

      double minVelocity = 10000;
      double maxVelocity = 13000;

      int shooterThreshold = 300;
      
      double targetShooterVelocity = shooterSlope * distance + shooterIntercept;

      if(targetShooterVelocity > maxVelocity) {
        targetShooterVelocity = maxVelocity;
      }
      else if(targetShooterVelocity < minVelocity){
        targetShooterVelocity = minVelocity;
      }
      

      //CHECKS IF WE CAN SHOOT
      if(moveIndexer){//checks if we want to move indexer, then if we are shootable, then if we have not shot ball yet
        if(m_shooter.isShooterAtVelocity((int)targetShooterVelocity, shooterThreshold) && Math.abs(LimelightSubsystem.getInstance().getXOffset()) <5){               
              IndexerSubsystem.getInstance().setIndexerPercentPower(Constants.indexerUp, false);               
              IndexerSubsystem.getInstance().setIntakePercentPower(Constants.intakeOn, false);    
        }
      }


      SmartDashboard.putNumber("Shooter Target", targetShooterVelocity);
      SmartDashboard.putNumber("Distance Away", distance);

      return (int)targetShooterVelocity;
    }

    @Override
    public void end(boolean interruptable){
        new RobotIdle().schedule();
    }

    class DistanceShooter{
      public double distance;
      public double shooter;
      public DistanceShooter(double distance, double shooter){
          this.distance = distance;
          this.shooter = shooter;
      }
    }
}

