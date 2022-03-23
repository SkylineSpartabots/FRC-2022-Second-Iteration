package frc.robot.factories;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;

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
        Pose2d position1 = getPose(7.494, 1.776, -87.28149);
        Pose2d position2 = getPose(7.545, 0.715, -89.37506);
        Pose2d position3 = getPose(5.809, 2.507, -143.5408);
        Pose2d position4 = getPose(5.321, 2.117, -140.974);
        Pose2d position5 = getPose(1.367, 1.423, -136.7892);
        Pose2d position6 = getPose(5.321, 2.117, -140.974);

        Command calibration = new CalibrationCommand(position1);
        Command command0 = new SetShooterCommand(0.5);
        Command command1 = new SetHoodCommand(-14000);
        Command command2 = new SetIntakeCommand(intakeOn,true);
        Command command3 = new SetIndexerCommand(indexerUp,true);
        Command driveToPosition2 = new TrajectoryDriveCommand(position2, List.of(), false);
        Command driveToPosition3 = new TrajectoryDriveCommand(position3, List.of(), true);
        Command command4 = new SetIntakeCommand(intakeOn,false);
        Command command5 = new SetIndexerCommand(indexerUp,false);
        Command driveToPosition4 = new TrajectoryDriveCommand(position4, List.of(), true);
        Command command6 = new SetIntakeCommand(intakeOn,true);
        Command command7 = new SetIndexerCommand(indexerUp,true);
        Command driveToPosition5 = new TrajectoryDriveCommand(position5, List.of(), false);
        Command driveToPosition6 = new TrajectoryDriveCommand(position6, List.of(), true);
        Command command8 = new SetIntakeCommand(intakeOn,false);
        Command command9 = new SetIndexerCommand(indexerUp,false);


        return new SequentialCommandGroup(
            calibration,
            command0,
            command1,
            command2,
            command3,
            driveToPosition2, 
            driveToPosition3,  
            command4,
            command5,
            driveToPosition4,
            command6,
            command7,
            driveToPosition5,
            driveToPosition6,
            command8,
            command9
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