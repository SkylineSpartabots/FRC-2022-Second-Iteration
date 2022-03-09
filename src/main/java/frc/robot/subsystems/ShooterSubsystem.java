package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.commands.CASShootCommand;

public class ShooterSubsystem extends SubsystemBase {
    //get instance
    private static ShooterSubsystem instance = null;          
    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private final LazyTalonFX mMasterShooter, mSlaveShooter;  

    PIDController shooterController;
    private ShooterSubsystem() {
        mMasterShooter = TalonFXFactory.createDefaultFalcon("Master Shooter Motor", Ports.MASTER_SHOOTER_MOTOR);
        configureMotor(mMasterShooter, true);
        mSlaveShooter = TalonFXFactory.createSlaveFalcon("Follower Shooter Motor", Ports.FOLLOW_SHOOTER_MOTOR, Ports.MASTER_SHOOTER_MOTOR);
        mSlaveShooter.setMaster(mMasterShooter);
        configureMotor(mSlaveShooter, false);

    }

    private void configureMotor(LazyTalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.config_kF(0, 0.047, Constants.kTimeOutMs);
        talon.config_kP(0, 0.02, Constants.kTimeOutMs);
        talon.config_kI(0, 0, Constants.kTimeOutMs);
        talon.config_kD(0, 0, Constants.kTimeOutMs);
    }

    //percent power: -1 through 1. Voltage compensated
    public void setShooterPercentPower(double power) {
        mMasterShooter.set(ControlMode.PercentOutput, power);
    }

    //unit: rotations per 100 ms. Shooter velocity for against the hub: 10,000 rp100ms
    public void setShooterVelocity(double velocity){        
        //uses Feed Foward and PID to set to velocity. configured in configureMotor()
        mMasterShooter.set(ControlMode.Velocity, velocity);
    }

    //detects if shooter is at a RPS. Ex: shooterAtVelocityRPS(10000) [for against the hub]
    public boolean shooterAtVelocityRPS(double velocity){
        if(mMasterShooter.getSelectedSensorVelocity()> velocity){
            return true;
        }
        else{
            return false;
        }
    }

    //manual set velocity FOR TESTING PURPOSES
    private double velocity = 0;
    public void increaseVelocity(){velocity += 500;setShooterVelocity(velocity);}
    public void decreaseVelocity(){velocity -= 500;setShooterVelocity(velocity);}


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Percent", mMasterShooter.getMotorOutputPercent());
        //SmartDashboard.putNumber("Shooter Target", velocity);
        SmartDashboard.putNumber("Shooter Vel", mMasterShooter.getSelectedSensorVelocity());
    }
}