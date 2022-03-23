// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.lib.util.Controller;
import frc.robot.commands.CASDriveCommand;
import frc.robot.commands.CASShootCommand;
import frc.robot.commands.SetSubsystemCommand.SetHoodCommand;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetShooterCommand;
import frc.robot.factories.AutonomousCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem m_drivetrainSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    LimelightSubsystem m_limelight = LimelightSubsystem.getInstance();
    // Set the scheduler to log Shuffleboard events for command initialize,
    // interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker(
        "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker(
        "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker(
        "Command finished", command.getName(), EventImportance.kNormal));

    // Configure the button bindings
    configureButtonBindings();

    // initialize Shuffleboard swapping of autonomous commands
    AutonomousCommandFactory.swapAutonomousCommands();
  }

  private static final Controller m_controller = new Controller(new XboxController(0));

  public static Controller getController(){
    return m_controller;
  }

  // configures button bindings to controller
  private void configureButtonBindings() {
    final double triggerDeadzone = 0.8;

    HoodSubsystem m_hoodSubsystem = HoodSubsystem.getInstance();
    IndexerSubsystem m_indexerSubsystem = IndexerSubsystem.getInstance();
    ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();

    //DPAD
    Trigger dpadUp = new Trigger(() -> {return m_controller.getDpadUp();});
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});    
    Trigger dpadLeft = new Trigger(() -> {return m_controller.getDpadLeft();});
    Trigger dpadRight = new Trigger(() -> {return m_controller.getDpadRight();});

    dpadUp.whenActive(m_hoodSubsystem::moveHoodUp).whenInactive(m_hoodSubsystem::stopHood);  //works  
    dpadDown.whenActive(m_hoodSubsystem::moveHoodDown).whenInactive(m_hoodSubsystem::stopHood);   //works
    dpadRight.whenActive(m_hoodSubsystem::resetHoodPosition);
    dpadLeft.whenActive(m_drivetrainSubsystem::resetFromStart);

    m_controller.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets to 0 -> for testing only
    m_controller.getRightBumper().whenActive(new SetIntakeIndexerCommand(0.8, 0.5));//right bumper hold
    m_controller.getRightBumper().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release
    m_controller.getLeftBumper().whenActive(new SetIntakeIndexerCommand(-0.5, -0.3));//right bumper hold
    m_controller.getLeftBumper().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release

    // back button
    /*
    m_controller.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets odometry and heading
    m_controller.getXButton().whenPressed(m_drivetrainSubsystem::resetFromStart);// resets odometry and heading
    //m_controller.getStartButton().whenPressed(m_drivetrainSubsystem::resetOdometryFromReference);//what does this do????
    m_controller.getStartButton().whenPressed(new InstantCommand(()-> IndexerSubsystem.getInstance().startAutoIntaking()));

     //left triggers and bumpers
     Trigger leftTriggerAxis = new Trigger(() -> { return m_controller.getLeftTriggerAxis() > triggerDeadzone;});//left trigger deadzone 0.8
     leftTriggerAxis.whenActive(new SetShooterCommand(shooterRamp));//on trigger hold
     leftTriggerAxis.whenInactive(new SetShooterCommand(shooterIdle));//on trigger release
     m_controller.getLeftBumper().whenPressed(new SetShooterCommand(shooterOff));//left bumper stops shooter
     
     //right triggers and bumpers
     Trigger rightTriggerAxis = new Trigger(() -> { return m_controller.getRightTriggerAxis() > triggerDeadzone;});//right trigger deadzone 0.8
     rightTriggerAxis.whenActive(//TO DO: FIGURE OUT CANCELLING COMMAND
        new SequentialCommandGroup( //on trigger hold, waits for
          new SetShooterCommand(shooterFire), //ramps up shooter to shooting speeds
          new WaitUntilCommand(() -> {return ShooterSubsystem.getInstance().shooterAtVelocityRPS(shootVelocityCondition, 8000);}), //waits for correct velocity
          new SetIndexerCommand(indexerFire, false), new SetIntakeCommand(intakeOn, false))); //fires indexer
     rightTriggerAxis.whenInactive(new ParallelCommandGroup(
        new SetShooterCommand(shooterIdle),
        new SetIndexerCommand(indexerOff, false),
        new SetIntakeCommand(intakeOff, false)));//on trigger release


m_controller.getRightBumper().whenActive(new SetIndexerCommand(indexerUp, false));//right bumper hold
m_controller.getRightBumper().whenInactive(new SetIndexerCommand(indexerOff, false));//right bumper release
     //joystick buttons     
    m_controller.getRightStickButton().whenHeld(new CASDriveCommand());
     
    // buttons
    
    m_controller.getYButton().whenPressed(new SetIntakeCommand(intakeReverse));
    m_controller.getYButton().whenReleased(new SetIntakeCommand(intakeOn));

    
    //m_controller.getAButton().whenPressed(new SetHoodCommand(0));
    //m_controller.getYButton().whenPressed(new SetHoodCommand(-10000));
    HoodSubsystem m_hoodSubsystem = HoodSubsystem.getInstance();
    m_controller.getYButton().whenPressed(new InstantCommand(() ->  m_hoodSubsystem.resetHoodPosition(), m_hoodSubsystem));
    //m_controller.getYButton().whenPressed(m_hoodSubsystem::resetHoodPosition);
    m_controller.getAButton().whenPressed(new SetIntakeCommand(intakeOn, false));
    m_controller.getBButton().whenPressed(new SetIntakeCommand(intakeOff, false));
    //DPAD
    
    //FOR TESTING PURPOSES
    ShooterSubsystem m_ShooterSubsystem = ShooterSubsystem.getInstance();//sets shooter manually
    Trigger dpadUp = new Trigger(() -> {return m_controller.getDpadUp();});
    dpadUp.whenActive(new CASShootCommand());
    //dpadUp.whenActive(m_ShooterSubsystem::increaseVelocity);
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});
    //dpadDown.whenActive(m_ShooterSubsystem::decreaseVelocity);
    
    Trigger dpadLeft = new Trigger(() -> {return m_controller.getDpadLeft();});//sets hood position manually
    dpadLeft.whenActive(m_hoodSubsystem::increaseTarget);
    Trigger dpadRight = new Trigger(() -> {return m_controller.getDpadRight();});
    dpadRight.whenActive(m_hoodSubsystem::decreaseTarget);
    
    
    Trigger dpadUp = new Trigger(() -> {return m_controller.getDpadUp();});//hold dpad up for indexer up
    dpadUp.whenActive(new SetIndexerCommand(indexerUp)).whenInactive(new SetIndexerCommand(indexerOff));
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});//hold dpad down for indexer down
    dpadDown.whenActive(new SetIndexerCommand(indexerDown)).whenInactive(new SetIndexerCommand(indexerOff));
    */
  }

  public void onRobotDisabled() {
    //called when robot is disabled. Set all subsytems to 0
    IndexerSubsystem.getInstance().setIntakePercentPower(0.0, false);
    IndexerSubsystem.getInstance().setIndexerPercentPower(0.0, false);
    ShooterSubsystem.getInstance().setShooterVelocity(0);
    HoodSubsystem.getInstance().moveHoodToPosition(0);
  }
}
