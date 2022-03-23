package frc.robot.subsystems;


import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class HoodSubsystem extends SubsystemBase {
    //get instance
    private static HoodSubsystem instance = null;          
    public static HoodSubsystem getInstance() {
        if (instance == null) {
            instance = new HoodSubsystem();
        }
        return instance;
    }

    private final LazyTalonFX m_hoodMotor; 
    private int target = 0; //TARGET IS FOR OUTPUT ONLY

    private HoodSubsystem() {
        m_hoodMotor = TalonFXFactory.createDefaultFalcon("Hood Motor", Ports.HOOD_MOTOR);//creates motor
        m_hoodMotor.setInverted(false);
        m_hoodMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_hoodMotor.enableVoltageCompensation(true);
        m_hoodMotor.setNeutralMode(NeutralMode.Brake);
        m_hoodMotor.config_kF(0, 0.0, Constants.kTimeOutMs);
        m_hoodMotor.config_kP(0, 0.3, Constants.kTimeOutMs);
        m_hoodMotor.config_kI(0, 0.0, Constants.kTimeOutMs);
        m_hoodMotor.config_kD(0, 0.0, Constants.kTimeOutMs);
        resetHoodPosition();
    }
    public void moveHoodToPosition(int targetPosition){
        m_hoodMotor.set(ControlMode.Position, targetPosition);
        target = targetPosition;
    }

    public void resetHoodPosition() {
        m_hoodMotor.setSelectedSensorPosition(0);
        target = 0;
        moveHoodToPosition(target);
    }

    private boolean isCASActive = false;
    public void CASIsActive(){
        isCASActive = true;
    }
    public void CASIsInactive(){
        isCASActive = false;
    }

    //move hood using controls for reset
    public void moveHoodUp(){
        if(!isCASActive)
        m_hoodMotor.set(ControlMode.PercentOutput, -0.1);
    }
    public void moveHoodDown(){
        if(!isCASActive)
        m_hoodMotor.set(ControlMode.PercentOutput, 0.1);
    }
    public void stopHood(){
        if(!isCASActive)
        m_hoodMotor.set(ControlMode.PercentOutput, 0);
    }


    public double getHoodPosition(){        
        return m_hoodMotor.getSelectedSensorPosition();
    }

    //Min Position: 0
    //Max Position: -17,000
    public boolean getIsHoodAtPosition(double position, double threshold){
        if(Math.abs(getHoodPosition() - position) < threshold){
            return true;
        }
        else{
            return false;
        }
    }
    

    @Override
    public void periodic(){  
        SmartDashboard.putNumber("Hood Position", getHoodPosition());
        SmartDashboard.putNumber("Hood Target", target);
        SmartDashboard.putNumber("Hood Power", m_hoodMotor.getMotorOutputPercent());
    }
}