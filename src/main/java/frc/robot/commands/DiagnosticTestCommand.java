package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DiagnosticTestCommand extends CommandBase{
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private IndexerSubsystem m_indexerSubsystem;
    private ShooterSubsystem m_shooterSubsystem;

    private final double expectedPeakDriveVelocity = 0;
    private final double expectedPeakIndexerVelocity = 0;
    private final double expectedPeakIntakeVelocity = 0;
    private final double expectedPeakShooterVelocity = 0;

    private double actualPeakDriveVelocity = 0;
    private double actualPeakIndexerVelocity = 0;
    private double actualPeakIntakeVelocity = 0;
    private double actualPeakShooterVelocity = 0;

    private final Timer m_timer = new Timer();
    private final double testingTime = 3;
    
    public DiagnosticTestCommand(){
        m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
        m_indexerSubsystem = IndexerSubsystem.getInstance();
        m_shooterSubsystem = ShooterSubsystem.getInstance();
        addRequirements(m_drivetrainSubsystem, m_indexerSubsystem, m_shooterSubsystem);
    }
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }
    @Override
    public void execute() {
        actualPeakDriveVelocity = Math.max(actualPeakDriveVelocity, m_drivetrainSubsystem.getVelocity());
        actualPeakIndexerVelocity = Math.max(actualPeakIndexerVelocity, m_indexerSubsystem.getIndexerVelocity());
        actualPeakIntakeVelocity = Math.max(actualPeakIntakeVelocity, m_indexerSubsystem.getIntakeVelocity());
        actualPeakShooterVelocity = Math.max(actualPeakShooterVelocity, m_shooterSubsystem.getVelocity());
    }
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        SmartDashboard.putBoolean("Driving?", actualPeakDriveVelocity >= expectedPeakDriveVelocity);
        SmartDashboard.putBoolean("Indexing?", actualPeakIndexerVelocity >= expectedPeakIndexerVelocity);
        SmartDashboard.putBoolean("Intaking?", actualPeakIntakeVelocity >= expectedPeakIntakeVelocity);
        SmartDashboard.putBoolean("Shooting?", actualPeakShooterVelocity >= expectedPeakShooterVelocity);
    }
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(testingTime);
    }
}