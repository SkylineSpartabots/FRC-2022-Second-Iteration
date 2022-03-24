package frc.robot.factories;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.CAS.RobotOff;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;
import java.util.List;

public class AutonomousCommandFactory {

    public static SendableChooser<Command> m_chooser = new SendableChooser<>();
    public static void swapAutonomousCommands() {
        m_chooser.setDefaultOption("fiveBallAuto", fiveBallAuto());
        m_chooser.addOption("PIDTest", PIDTest());
        SmartDashboard.putData(m_chooser);
    }

    public static Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public static Pose2d getPose(double x, double y, double rot){
            return new Pose2d(x, y, new Rotation2d(Math.toRadians(rot)));
        }

    public static Command fiveBallAuto(){
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(7.59, 1.79, -88.42)),            
            new SetIntakeCommand(intakeOn,true),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed)),
            new SetHoodCommand((int)hoodFixed),
            new TrajectoryDriveCommand(getPose(7.58, 0.68, -90.19), List.of(), false,0.5, 2 ,1),
            new TrajectoryDriveCommand(getPose(5.59, 2.27, -142.05), List.of(new Translation2d(6.03, 1.98)), true, 0.6, 3,1),
            new SetIntakeCommand(intakeOn,false),
            new SetIndexerCommand(indexerUp,false),
            new TrajectoryDriveCommand(getPose(5.48, 2.18, -141.57), List.of(), false, 0.5, 0.3, 0.3),
            new WaitCommand(1),
            new SetIntakeCommand(intakeOn,true),
            new SetIndexerCommand(indexerUp,true),
            new TrajectoryDriveCommand(getPose(1.38, 1.41, -137.29), List.of(), false, 0.2, 3, 1.5),
            new TrajectoryDriveCommand(getPose(5.48, 2.18, -141.57), List.of(), true, 0.4,3,1.5),
            new SetIntakeCommand(intakeOn,false),
            new SetIndexerCommand(indexerUp,false),
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command PIDTest(){
        DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

        Pose2d position1 = getPose(7.781, 2.937, -110.1519);
        Pose2d position2 = getPose(7.475, 1.756, -87.28149);

        Command resetOdo = new InstantCommand(()->m_drivetrainSubsystem.resetOdometryFromPosition(position1), m_drivetrainSubsystem);

        Command driveToPosition2 = new TrajectoryDriveCommand(position2, List.of(), false);


        return new SequentialCommandGroup(
            resetOdo,
            driveToPosition2
            );
    }

}