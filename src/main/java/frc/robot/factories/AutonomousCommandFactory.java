package frc.robot.factories;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.commands.WaitUntilCommand.WaitUntilIndexerCommand;
import frc.robot.commands.WaitUntilCommand.WaitUntilIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

import java.util.List;

public class AutonomousCommandFactory {

    public static SendableChooser<Command> m_chooser = new SendableChooser<>();

    public static void swapAutonomousCommands() {
        m_chooser.setDefaultOption("Blue Four Ball Auto Bottom Left", blueFourBallAuto());

        SmartDashboard.putData(m_chooser);
    }

    public static Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public static Pose2d getPose(double x, double y, double rot){
       return new Pose2d(x, y, new Rotation2d(Math.toRadians(rot)));
    }
    public static Command blueFourBallAuto(){
        DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

        Pose2d startPose = getPose(7.77, 2.94, -111);
        Pose2d shootPose = getPose(7.67, 2.59, -111);
        Pose2d ball1 = getPose(5.5, 2.07, -160);
        Pose2d ball2 = getPose(1.58, 1.48, -137);
        Pose2d ball3 = getPose(7.63, 0.65, -90);

        //SET STARTING POSITION
        Command resetOdo = new InstantCommand(() ->  m_drivetrainSubsystem.resetOdometryFromPosition(startPose), m_drivetrainSubsystem);

        Command turnOnIntake = new WaitUntilIntakeCommand();
        Command rampUpShooter = new SetShooterCommand(shooterRamp);
        Command driveToFirstBall = new TrajectoryDriveCommand(ball1, List.of(), false);
        Command driveBackToShoot = new TrajectoryDriveCommand(shootPose,List.of(), true);
        Command fireIndexer = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish = new WaitCommand(1);
        Command turnOnIntake2 = new WaitUntilIndexerCommand();
        Command driveToSecondBall = new TrajectoryDriveCommand(ball2, List.of(), false);
        Command driveBackToShootSecondTime = new TrajectoryDriveCommand(shootPose,List.of(),true);
        Command fireIndexer2 = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish2 = new WaitCommand(1);
        Command turnOffIndexer2 = new SetIndexerCommand(indexerOff);
        Command turnOffShooter2 = new SetShooterCommand(shooterOff);
        //Command driveToThirdBall = new TrajectoryDriveCommand(ball3, List.of(), false);
        //Command turnIntakeOff = new SetIntakeCommand(intakeOff);

        return new SequentialCommandGroup(        
            resetOdo,
            new ParallelDeadlineGroup(//ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    rampUpShooter,
                    driveToFirstBall,
                    driveBackToShoot),
                turnOnIntake),
            fireIndexer, new SetIntakeCommand(intakeOn),
            waitForShooterToFinish,
            new ParallelDeadlineGroup(//WHY DOES THIS NOT WORK WHEN NO BALLS INTAKED IN???????????????
                new SequentialCommandGroup(
                    driveToSecondBall,
                    driveBackToShootSecondTime),
                turnOnIntake2),
            fireIndexer2, new SetIntakeCommand(intakeOn),
            waitForShooterToFinish2,
            turnOffIndexer2,
            turnOffShooter2
            );
    }
}