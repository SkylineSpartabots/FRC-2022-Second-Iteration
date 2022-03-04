package frc.robot.commands;

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
import frc.robot.subsystems.DrivetrainSubsystem;

public class CASDriveCommand extends TeleopDriveCommand{
    private Translation2d m_targetPosition;
    private ProfiledPIDController m_thetaController;

    public CASDriveCommand() {
        super(DrivetrainSubsystem.getInstance());
       
        m_targetPosition = Constants.targetHudPosition;
        // m_thetaController = new ProfiledPIDController(DriveConstants.kpRotation, DriveConstants.kiRotation, 
        //     DriveConstants.kdRotation, DriveConstants.kRotationConstraints);
        //m_thetaController = new PIDController(DriveConstants.kpRotation, DrivetrainConstants.kiRotation, kd)
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void driveWithJoystick() {
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        
        double tx = findAngle(m_drivetrainSubsystem.getPose(), m_targetPosition.getX(), m_targetPosition.getY());
        SmartDashboard.putNumber("tx", tx);

        //double rot = m_thetaController.calculate(0, tx);

         double rot = 0.1 * tx;
         rot = Math.copySign(Math.pow(Math.abs(rot), 0.5),rot);//multiplies it by the root of the heading error, keeping sign

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot);
    
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    public double findAngle(Pose2d currentPose, double toX, double toY){
        double deltaY = (toY - currentPose.getY());
        double deltaX = (toX - currentPose.getX());

        double absolute = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double angle = normalize(currentPose.getRotation().getDegrees() - 180);
        double result = normalize(absolute - angle);

        return result;
      }

    public double normalize(double deg){
        double result = deg;
        if(Math.abs(result)>180){
            result = -Math.copySign(360-Math.abs(result), result);
        }
        return result;
    }
}
