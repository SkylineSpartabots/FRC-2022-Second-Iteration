package frc.robot.commands.CAS;

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
        m_hood.CASIsActive();
    }


    @Override
    public void execute(){
      LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
      if(m_limelightSubsystem.getXOffset() == 0){
        calculateShooterHood(DrivetrainSubsystem.distanceFromHub());
      }
      else{
        calculateShooterHood(LimelightSubsystem.getInstance().getDistance());
      }
    }

    //hood position formula, y = hood position, x = distance away:
    //y = -6316 * x + 7317
    //limit: hood cannot be greater than 0 or less than -37,000

    //shooter velocity formula, y = shooter velocity, x = distance away:
    //y = 735.8 * x + 6157.0
    //limit: hood velocity cannot be less than 7,000 or more than 11,000

    //CALIBRATE THESE VALUES
    //shooter limits: cannot shoot if greater than 5 meters away
    //if greater than 5 meters away, reset hood position and turn off shooter
    
    private void calculateShooterHood(double distance){

      double hoodSlope = -10309.0;
      double hoodIntercept = 21041.0;

      double shooterSlope = 735.8;
      double shooterIntercept = 6157.0;

      double minVelocity = 7500;
      double maxVelocity = 10500;

      double minHood = 0;
      double maxHood = -32000;

      int shooterThreshold = 200;
      int hoodThreshold = 250;
      

      double targetHoodPosition = hoodSlope * distance + hoodIntercept;
      double targetShooterVelocity = shooterSlope * distance + shooterIntercept;
      boolean shootable = true;

      if(targetShooterVelocity > maxVelocity) {
        targetShooterVelocity = maxVelocity;
        shootable = false;
      }
      else if(targetShooterVelocity < minVelocity){
        targetShooterVelocity = minVelocity;
        shootable = false;
      }      
      if(targetHoodPosition > minHood) {
        targetHoodPosition = minHood;
        shootable = false;
      }
      else if(targetHoodPosition < maxHood) {
        targetHoodPosition = maxHood;
        shootable = false;
      }

      //SET HOOD TO 0 WHEN OUT OF RANGE????
      if(!shootable){
        targetHoodPosition = 0;
      }
      
      m_hood.moveHoodToPosition((int)targetHoodPosition);
      m_shooter.setShooterVelocity((int)targetShooterVelocity);

      //CHECKS IF WE CAN SHOOT
      if(moveIndexer && shootable && !shotBall){//checks if we want to move indexer, then if we are shootable, then if we have not shot ball yet
        if(m_shooter.isShooterAtVelocity((int)targetShooterVelocity, shooterThreshold)//is shooter at velocity?
            && m_hood.getIsHoodAtPosition((int)targetHoodPosition, hoodThreshold)){ //is hood at position?
              boolean withinRange = false;

              if(LimelightSubsystem.getInstance().getXOffset() == 0){//do we use limelight or odo angle to hub?
                if(Math.abs(Math.toRadians(
                  DrivetrainSubsystem.findAngle(DrivetrainSubsystem.getInstance().getPose(), Constants.targetHudPosition.getX(), Constants.targetHudPosition.getY(), 180))) 
                  < 3){
                    //checks if we are within angle of hub using odo
                    withinRange = true;
                }
              }
              else{
                if(Math.abs(LimelightSubsystem.getInstance().getXOffset()) <3){
                  //checks if we are within angle of hub using limelight
                  withinRange = true;
                }
              }
              if(withinRange){
                new SequentialCommandGroup(
                  new SetIndexerCommand(Constants.indexerUp, false), 
                  new WaitCommand(0.25),
                  new SetIndexerCommand(Constants.intakeOn, false)).schedule();                   
                shotBall = true;
              }
        }
      }


      SmartDashboard.putNumber("Hood Target", targetHoodPosition);
      SmartDashboard.putNumber("Shooter Target", targetShooterVelocity);
      SmartDashboard.putNumber("Distance Away", distance);
      SmartDashboard.putBoolean("IN RANGE?", shootable);

    }

    @Override
    public void end(boolean interruptable){
      m_hood.CASIsInactive();
      if(shotBall){
        //if shot, set intake and indexer to automatic
        new RobotIdle().schedule();
      }
    }

}
