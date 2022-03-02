package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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

    private HoodSubsystem() {
        m_hoodMotor = TalonFXFactory.createDefaultFalcon("Intake Motor", Ports.HOOD_MOTOR);//creates motor
        m_hoodMotor.setInverted(false);
        m_hoodMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_hoodMotor.enableVoltageCompensation(true);
        m_hoodMotor.setNeutralMode(NeutralMode.Brake);

        //TODO: CONFIGURE PID CONTROLLER, ASK FRC DISCORD
    }

    public void resetHoodPosition() {
        m_hoodMotor.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs);
    }

    public void resetHoodPosition(int pos) {
        m_hoodMotor.setSelectedSensorPosition(pos, 0, Constants.kTimeOutMs);
    }

    public void moveHoodToPosition(int targetPos){//moves hood to position   
        m_hoodMotor.set(ControlMode.MotionMagic, targetPos);
    }

    public double getHoodPosition(){        
        return m_hoodMotor.getSelectedSensorPosition();
    }

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
    }
}