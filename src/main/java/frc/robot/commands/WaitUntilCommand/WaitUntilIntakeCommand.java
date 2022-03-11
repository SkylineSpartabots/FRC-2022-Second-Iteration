package frc.robot.commands.WaitUntilCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitUntilIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_subsystem;
    private final ColorSensorSubsystem m_color;

    public WaitUntilIntakeCommand() {
        m_subsystem = IntakeSubsystem.getInstance();
        m_color = ColorSensorSubsystem.getInstance();
        addRequirements(m_subsystem, m_color);
    }

    @Override
    public void initialize() {
        m_subsystem.setIntakePercentPower(Constants.intakeOn);
    }

    @Override
    public boolean isFinished() {
        return m_color.isIntakeBallLoaded();
    }
    @Override
    public void end(boolean interrupted){
        m_subsystem.setIntakePercentPower(Constants.intakeOff);
        if(!m_color.isAllianceBallIndexer()){
            new SequentialCommandGroup(
                new SetIndexerCommand(Constants.intakeReverse), 
                new WaitCommand(1),
                new SetIndexerCommand(Constants.indexerOff)).schedule();
        }
    }
}
