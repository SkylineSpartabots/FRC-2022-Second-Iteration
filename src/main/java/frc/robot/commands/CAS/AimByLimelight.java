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

public class AimByOdometry extends TeleopDriveCommand{
    private PIDController m_thetaController;

    public AimByOdometry() {
        super(DrivetrainSubsystem.getInstance());
       
        m_thetaController = new PIDController(8.0,0,0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void driveWithJoystick() {
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var rot = m_thetaController.calculate(Math.toRadians(LimelightSubsystem.getInstance().getXOffset()),0.0);
        
        if(Math.abs(LimelightSubsystem.getInstance().getXOffset()) < 3.0){
            rot = 0;
        }

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot);
    
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
    }
}