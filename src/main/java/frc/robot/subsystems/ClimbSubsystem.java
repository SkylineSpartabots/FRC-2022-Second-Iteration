package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance = null;

    public static ClimbSubsystem getInstance(){
        if(instance == null){
            instance = new ClimbSubsystem();
        }
        return instance;
    }

    //left motor handles the side closer to the shooter, right handles the side closer to intake
    private final LazyTalonFX mLeftClimb, mRightClimb;

    private ClimbSubsystem(){

        //motors for high climb??
        mLeftClimb = TalonFXFactory.createDefaultFalcon("Left Climb Motor", Ports.LEFT_CLIMB);
        configureMotor(mLeftClimb, false);
        mRightClimb = TalonFXFactory.createDefaultFalcon("Right Climb Motor", Ports.RIGHT_CLIMB);
        configureMotor(mRightClimb, false);
    }

    private void configureMotor(LazyTalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
        /*
        talon.config_kF(0, 0.047, Constants.kTimeOutMs);
        talon.config_kP(0, 0.02, Constants.kTimeOutMs);
        talon.config_kI(0, 0, Constants.kTimeOutMs);
        talon.config_kD(0, 0, Constants.kTimeOutMs);
        */
    }

    //up is true, down is false
    public void setLeftPercentPower(double power){
        mLeftClimb.set(ControlMode.PercentOutput, power);
    }

    public void setRightPercentPower(double power){
        mRightClimb.set(ControlMode.PercentOutput, power);        
    }
    
    @Override
    public void periodic(){  
        SmartDashboard.putNumber("Climb Right Speed", mRightClimb.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Climb Left Speed", mLeftClimb.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Climb Right Position", mRightClimb.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb Left Position", mLeftClimb.getSelectedSensorPosition());
    }

}

//left is 42, right 41