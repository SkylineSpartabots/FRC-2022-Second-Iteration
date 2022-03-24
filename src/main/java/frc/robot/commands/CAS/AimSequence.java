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

public class AimSequence extends TeleopDriveCommand{
    private Translation2d m_targetPosition;
    private PIDController odoController;
    private PIDController limelightController;

    public AimSequence() {
        super(DrivetrainSubsystem.getInstance());
       
        m_targetPosition = Constants.targetHudPosition;
        odoController = new PIDController(2.0,0,0);
        odoController.enableContinuousInput(-Math.PI, Math.PI);
        limelightController = new PIDController(2.0,0,0);
        limelightController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void driveWithJoystick() {
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        double rot;

        LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
        if(Math.abs(m_limelightSubsystem.getXOffset()) != 0.0){
            
            rot = limelightController.calculate(Math.toRadians(m_limelightSubsystem.getXOffset()),0.0);
            if(Math.abs(m_limelightSubsystem.getXOffset()) < 3.0){
                rot = 0;
            }            

            if(Math.abs(m_limelightSubsystem.getXOffset()) < 20.0 && m_limelightSubsystem.getXOffset() != 0.0){      
                double x = 8.23 - (m_limelightSubsystem.getDistance() * 
                    Math.cos(Math.toRadians(DrivetrainSubsystem.getInstance().getGyroscopeRotation().getDegrees() + 180 
                    - m_limelightSubsystem.getXOffset())));
                double y = 4.165 - (m_limelightSubsystem.getDistance() * 
                    Math.sin(Math.toRadians(DrivetrainSubsystem.getInstance().getGyroscopeRotation().getDegrees() + 180
                    - m_limelightSubsystem.getXOffset())));//plus or minus xoffset???
                
                DrivetrainSubsystem.getInstance().resetOdometryFromPosition(x,y);
            }
        }
        else{
            double targetAngle = Math.toRadians(DrivetrainSubsystem.findAngle(m_drivetrainSubsystem.getPose(), m_targetPosition.getX(), m_targetPosition.getY(), 180));
            rot = odoController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians(),targetAngle);
            
            if(Math.abs(Math.toDegrees(m_drivetrainSubsystem.getGyroscopeRotation().getRadians() - targetAngle)) < 3.0){
                rot = 0;
            }
        }

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot);
    
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
        //corrects odometry using limelight

    }
}
