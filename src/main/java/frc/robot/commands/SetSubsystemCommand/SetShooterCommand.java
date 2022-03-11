package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    private double velocity;

    public SetShooterCommand(double velocity) {
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.velocity = velocity;
    }

    @Override
    public void initialize() {
       m_subsystem.setShooterVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
