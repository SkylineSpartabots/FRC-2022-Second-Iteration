package frc.robot.commands.WaitUntilCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetShooterCommand;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitUntilIndexerCommand extends CommandBase {
    private final IndexerSubsystem m_subsystem;
    private final ColorSensorSubsystem m_color;

    public WaitUntilIndexerCommand() {
        m_subsystem = IndexerSubsystem.getInstance();
        m_color = ColorSensorSubsystem.getInstance();
        addRequirements(m_subsystem, m_color);
    }

    @Override
    public void initialize() {
        m_subsystem.setIndexerPercentPower(Constants.indexerUp);
        IntakeSubsystem.getInstance().setIntakePercentPower(Constants.intakeOn);
    }

    @Override
    public boolean isFinished() {
        return m_color.isIndexerBallLoaded();
    }
    @Override
    public void end(boolean interrupted){
        m_subsystem.setIndexerPercentPower(Constants.indexerOff);
        if(!m_color.isAllianceBallIndexer()){
            new SequentialCommandGroup(
                new ParallelCommandGroup(new WaitUntilHoodAtPosition(0), new WaitUntilShooterAtVelocity(5000)),
                new SetIndexerCommand(Constants.indexerUp), 
                new WaitUntilCommand(() -> !m_color.isIndexerBallLoaded()),
                new SetShooterCommand(0), 
                new WaitUntilIndexerCommand());
        }
        else{
            new WaitUntilIntakeCommand().schedule();
        }
    }
}
