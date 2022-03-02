package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class SetHoodCommand extends CommandBase {
    private final HoodSubsystem m_subsystem;
    private int position;

    public SetHoodCommand(int position) {
        m_subsystem = HoodSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.position = position;
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
