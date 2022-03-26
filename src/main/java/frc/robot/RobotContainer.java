// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.lib.util.Controller;
import frc.robot.commands.*;
import frc.robot.commands.CAS.AimByLimelight;
import frc.robot.commands.CAS.RobotIdle;
import frc.robot.commands.CAS.RobotOff;
import frc.robot.commands.CAS.ShootByLimelight;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.factories.AutonomousCommandFactory;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private static final Controller m_controller2 = new Controller(new XboxController(1));

  public static Controller getController(){
    return m_controller;
  }

  // configures button bindings to controller
  private void configureButtonBindings() {
    IndexerSubsystem m_indexerSubsystem = IndexerSubsystem.getInstance();
    ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    ShooterSubsystem m_climbSubsystem = ShooterSubsystem.getInstance();
    final double triggerDeadzone = 0.3;

    //FIRST CONTROLLER

    //DPAD
    Trigger dpadUp = new Trigger(() -> {return m_controller.getDpadUp();});
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});
    Trigger dpadLeft = new Trigger(() -> {return m_controller.getDpadLeft();});
    Trigger dpadRight = new Trigger(() -> {return m_controller.getDpadRight();});

    //SHOOTER ADJUSTMENT CONTROLS
    dpadUp.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(250)));  //works  
    dpadDown.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(-250)));   //works
    dpadRight.whenActive(m_shooterSubsystem::stopShooter);
    dpadLeft.whenActive(m_drivetrainSubsystem::resetFromStart);

    m_controller.getStartButton().whenPressed(m_drivetrainSubsystem::resetFromStart);// resets to 0 -> for testing only
    m_controller.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets to 0 -> for testing only
    m_controller.getRightBumper().whenActive(new SetIntakeIndexerCommand(intakeOn, indexerUp));//right bumper hold
    m_controller.getRightBumper().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release
    m_controller.getLeftBumper().whenActive(new SetIntakeIndexerCommand(intakeReverse, indexerDown));//right bumper hold
    m_controller.getLeftBumper().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release

    m_controller.getAButton().whenActive(new RobotIdle());
    m_controller.getYButton().whenActive(new RobotOff());

    m_controller.getXButton().whenHeld(new ShootByLimelight(false));
    m_controller.getXButton().whenHeld(new AimByLimelight("left"));
    m_controller.getBButton().whenHeld(new ShootByLimelight(false));
    m_controller.getBButton().whenHeld(new AimByLimelight("right"));
    
    m_controller.getRightStickButton().whenHeld(new ShootByLimelight(false));
    m_controller.getLeftStickButton().whenHeld(new AimByLimelight());
    
    Trigger leftTriggerAxis = new Trigger(() -> { return m_controller.getLeftTriggerAxis() > triggerDeadzone;});
    Trigger rightTriggerAxis = new Trigger(() -> { return m_controller.getRightTriggerAxis() > triggerDeadzone;});

    leftTriggerAxis.whileActiveOnce(new ShootByLimelight(true));
    leftTriggerAxis.whileActiveOnce(new AimByLimelight("left"));
    leftTriggerAxis.whenInactive(new SequentialCommandGroup(new WaitCommand(1), new RobotIdle()));
    rightTriggerAxis.whileActiveOnce(new ShootByLimelight(true));
    rightTriggerAxis.whileActiveOnce(new AimByLimelight("right"));
    rightTriggerAxis.whenInactive(new SequentialCommandGroup(new WaitCommand(1), new RobotIdle()));


    //SECOND CONTROLLER   : TODO ADD MANUAL OVERRIDE FOR INTAKING AND INDEXING INDIVIDUALLY

    //DPAD
    Trigger dpadUp2 = new Trigger(() -> {return m_controller2.getDpadUp();});
    Trigger dpadDown2 = new Trigger(() -> {return m_controller2.getDpadDown();});    
    Trigger dpadLeft2 = new Trigger(() -> {return m_controller2.getDpadLeft();});
    Trigger dpadRight2 = new Trigger(() -> {return m_controller2.getDpadRight();});

    dpadUp2.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(250)));  //works  
    dpadDown2.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(-250)));   //works
    dpadRight2.whenActive(m_shooterSubsystem::stopShooter);
    dpadLeft2.whenActive(m_drivetrainSubsystem::resetFromStart);
    
    //second controller controls intake only
    m_controller2.getAButton().whenActive(new SetIntakeCommand(intakeOn, false)).whenInactive(new SetIntakeCommand(0.0, false));
    m_controller2.getYButton().whenActive(new SetIntakeCommand(intakeReverse, false)).whenInactive(new SetIntakeCommand(0.0, false));    
    
    Trigger leftTriggerAxis2 = new Trigger(() -> { return m_controller2.getLeftTriggerAxis() > triggerDeadzone;});
    Trigger rightTriggerAxis2 = new Trigger(() -> { return m_controller2.getRightTriggerAxis() > triggerDeadzone;});

    leftTriggerAxis2.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().climbPower(climbDown)))
                   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().climbPower(0)));
    rightTriggerAxis2.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().climbPower(climbUp)))
                    .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().climbPower(0)));

    
    m_controller2.getLeftBumper().whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().getLeftPivot().set(ControlMode.PercentOutput, pivotDown)))
                              .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().getLeftPivot().set(ControlMode.PercentOutput, pivotUp)));
    m_controller2.getRightBumper().whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(pivotUp)))
                              .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(0))); 

    //have operator have fixed shooter controls
    
    m_controller2.getXButton().whenHeld(new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed)));
    m_controller2.getXButton().whenHeld(new AimByLimelight("left"));
    m_controller2.getXButton().whenReleased(new RobotIdle());
    
    m_controller2.getBButton().whenHeld(new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed)));
    m_controller2.getBButton().whenHeld(new AimByLimelight("right"));
    m_controller2.getBButton().whenReleased(new RobotIdle());
    
    m_controller2.getAButton().whenActive(new RobotIdle());
    m_controller2.getYButton().whenActive(new RobotOff());
  }

  public void onRobotDisabled() {
    //called when robot is disabled. Set all subsytems to 0
    IndexerSubsystem.getInstance().setIntakePercentPower(0.0, false);
    IndexerSubsystem.getInstance().setIndexerPercentPower(0.0, false);
    ShooterSubsystem.getInstance().setShooterVelocity(0);
    HoodSubsystem.getInstance().moveHoodToPosition(0);
  }
}
