package frc.robot.commands.CAS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.lib.util.Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AimByOdo extends TeleopDriveCommand{ //REPLACABLE BY AIM SEQUENCE
    private Translation2d m_targetPosition;
    private PIDController m_thetaController;

    public AimByOdo() {
        super(DrivetrainSubsystem.getInstance());
       
        m_targetPosition = Constants.targetHudPosition;
        m_thetaController = new PIDController(3.0,0,0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void driveWithJoystick() {
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        
        double targetAngle = Math.toRadians(findAngle(m_drivetrainSubsystem.getPose(), m_targetPosition.getX(), m_targetPosition.getY(), 180));

        var rot = m_thetaController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians(),targetAngle);

         if(Math.abs(targetAngle) < 3.0){
            rot = 0;
        }

        SmartDashboard.putNumber("targetAngle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot);
    
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));

        LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
        if(Math.abs(m_limelightSubsystem.getXOffset()) < 3.0 && m_limelightSubsystem.getXOffset() != 0.0){      
            double x = 8.23 - (m_limelightSubsystem.getDistance() * Math.cos(Math.toRadians(DrivetrainSubsystem.getInstance().getGyroscopeRotation().getDegrees() + 180 + m_limelightSubsystem.getXOffset())));
            double y = 4.165 - (m_limelightSubsystem.getDistance() * Math.sin(Math.toRadians(DrivetrainSubsystem.getInstance().getGyroscopeRotation().getDegrees() + 180+ m_limelightSubsystem.getXOffset())));
            DrivetrainSubsystem.getInstance().resetOdometryFromPosition(
                x,y, DrivetrainSubsystem.getInstance().getGyroscopeRotation().getDegrees());
        }

    }

    public double findAngle(Pose2d currentPose, double toX, double toY, double offsetDeg){
        double deltaY = (toY - currentPose.getY());
        double deltaX = (toX - currentPose.getX());

        double absolute = Math.toDegrees(Math.atan2(deltaY, deltaX));
        return m_drivetrainSubsystem.normalize(absolute + offsetDeg);
      }

    public double normalize(double deg){
        double result = deg;
        if(Math.abs(result)>180){
            result = -Math.copySign(360-Math.abs(result), result);
        }
        return result;
    }
}
