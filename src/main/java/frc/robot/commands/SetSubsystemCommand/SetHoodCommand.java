package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class SetHoodCommand extends CommandBase {
    private final HoodSubsystem m_subsystem;
    private int position;

    public SetHoodCommand(int hoodfixed) {
        m_subsystem = HoodSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.position = hoodfixed;
    }

    @Override
    public void initialize() {
        m_subsystem.moveHoodToPosition(position);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
