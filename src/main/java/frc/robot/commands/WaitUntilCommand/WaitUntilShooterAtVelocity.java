package frc.robot.commands.WaitUntilCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitUntilShooterAtVelocity extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    private double velocity;

    public WaitUntilShooterAtVelocity(double percentPower) {
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.velocity = percentPower;
    }

    @Override
    public void initialize() {
       m_subsystem.setShooterVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
      return m_subsystem.isShooterAtVelocity((int)velocity, 1000);
    }
}
