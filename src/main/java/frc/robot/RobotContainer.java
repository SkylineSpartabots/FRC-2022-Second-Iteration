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
    Trigger dpadUpRight = new Trigger(() -> {return m_controller.getDpadUpRight();});
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});
    Trigger dpadLeft = new Trigger(() -> {return m_controller.getDpadLeft();});
    Trigger dpadRight = new Trigger(() -> {return m_controller.getDpadRight();});
  
    //dpad up and dpad right controls left and right climb. press both to move at the same time
    dpadUp.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(climbUp)))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)));
    dpadRight.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(climbUp)))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));
    dpadUpRight.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(climbUp)))
      .whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(climbUp)))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));   

    dpadDown.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(climbDown)))
      .whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(climbDown)))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));   
    m_controller.getYButton().whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(pivotDown)))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(0)));

    m_controller.getStartButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets to 0 -> for testing only
    m_controller.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets to 0 -> for testing only
    m_controller.getRightBumper().whenActive(new SetIntakeIndexerCommand(intakeOn, indexerUp));//right bumper hold
    m_controller.getRightBumper().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release
    m_controller.getLeftBumper().whenActive(new SetIntakeIndexerCommand(intakeReverse, indexerDown));//right bumper hold
    m_controller.getLeftBumper().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release

    m_controller.getAButton().whenActive(new RobotIdle());
    dpadLeft.whenActive(new RobotOff());

    m_controller.getXButton().whenHeld(new ShootByLimelight(false));
    m_controller.getXButton().whenHeld(new AimByLimelight());
    m_controller.getBButton().whenHeld(new ShootByLimelight(false));
    m_controller.getBButton().whenHeld(new AimByLimelight());
    
    m_controller.getRightStickButton().whenHeld(new ShootByLimelight(false));
    m_controller.getLeftStickButton().whenHeld(new AimByLimelight());
    
    Trigger leftTriggerAxis = new Trigger(() -> { return m_controller.getLeftTriggerAxis() > triggerDeadzone;});
    Trigger rightTriggerAxis = new Trigger(() -> { return m_controller.getRightTriggerAxis() > triggerDeadzone;});

    leftTriggerAxis.whileActiveOnce(new ShootByLimelight(true));
    leftTriggerAxis.whileActiveOnce(new AimByLimelight());
    leftTriggerAxis.whenInactive(new SequentialCommandGroup(new WaitCommand(1), new RobotIdle()));
    rightTriggerAxis.whileActiveOnce(new ShootByLimelight(true));
    rightTriggerAxis.whileActiveOnce(new AimByLimelight());
    rightTriggerAxis.whenInactive(new SequentialCommandGroup(new WaitCommand(1), new RobotIdle()));


    //SECOND CONTROLLER: TODO ADD MANUAL OVERRIDE FOR INTAKING AND INDEXING INDIVIDUALLY AND EJECTION

    //DPAD
    Trigger dpadUp2 = new Trigger(() -> {return m_controller2.getDpadUp();});
    Trigger dpadDown2 = new Trigger(() -> {return m_controller2.getDpadDown();});    
    Trigger dpadLeft2 = new Trigger(() -> {return m_controller2.getDpadLeft();});
    Trigger dpadRight2 = new Trigger(() -> {return m_controller2.getDpadRight();});

    dpadUp2.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(250)));  //works  
    dpadDown2.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(-250)));   //works
    dpadRight2.whenActive(new InstantCommand(() -> m_shooterSubsystem.setShooterVelocity(shooterEjection)));
    dpadLeft2.whenActive(new InstantCommand(() -> m_shooterSubsystem.setShooterVelocity(shooterFixed)));
    
    Trigger leftTriggerAxis2 = new Trigger(() -> { return m_controller2.getLeftTriggerAxis() > triggerDeadzone;});
    Trigger rightTriggerAxis2 = new Trigger(() -> { return m_controller2.getRightTriggerAxis() > triggerDeadzone;});

    leftTriggerAxis2.whenActive(new SetIndexerCommand(indexerDown,false))
                    .whenInactive(new SetIndexerCommand(0.0, false));
    rightTriggerAxis2.whenActive(new SetIndexerCommand(indexerUp,false))
                     .whenInactive(new SetIndexerCommand(0.0, false));
    m_controller2.getLeftBumper().whenPressed(new SetIntakeCommand(intakeReverse,false))
                    .whenReleased(new SetIndexerCommand(0.0, false));
    m_controller2.getRightBumper().whenPressed(new SetIntakeCommand(intakeOn,false))
                     .whenReleased(new SetIntakeCommand(0.0, false));
                    
                    
    Trigger leftJoyStick = new Trigger(() -> { return Math.abs(m_controller2.getLeftY()) > triggerDeadzone;});
    Trigger rightJoystick = new Trigger(() -> { return Math.abs(m_controller2.getRightY()) > triggerDeadzone;});
    leftJoyStick.whileActiveContinuous(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(-climbUp*m_controller2.getLeftY())))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)));
    rightJoystick.whileActiveContinuous(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(-climbUp*m_controller2.getRightY())))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));

    m_controller2.getLeftStickButton().whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(pivotDown)))
                              .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(0)));
    m_controller2.getRightStickButton().whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(pivotDown)))
                              .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(0)));

    
    m_controller2.getXButton().whenHeld(new SetIntakeCommand(intakeOn,true));
    m_controller2.getBButton().whenHeld(new SetIndexerCommand(indexerUp,true));
    m_controller2.getAButton().whenActive(new RobotIdle());
    m_controller2.getYButton().whenActive(new RobotOff());    
    m_controller2.getStartButton().whenPressed(m_drivetrainSubsystem::resetOdometry);
    m_controller2.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);
  }

  public void onRobotDisabled() {
    //called when robot is disabled. Set all subsytems to 0
    IndexerSubsystem.getInstance().setIntakePercentPower(0.0, false);
    IndexerSubsystem.getInstance().setIndexerPercentPower(0.0, false);
    ShooterSubsystem.getInstance().setShooterVelocity(0);
  }
}
