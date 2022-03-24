package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonSRXFactory;
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
    private final LazyTalonSRX mLeftPivot, mRightPivot;

    private ClimbSubsystem(){
        mLeftClimb = TalonFXFactory.createDefaultFalcon("Left Climb Motor", Ports.LEFT_CLIMB);
        configureMotor(mLeftClimb, false);
        mRightClimb = TalonFXFactory.createDefaultFalcon("Right Climb Motor", Ports.RIGHT_CLIMB);
        configureMotor(mRightClimb, false);
        mLeftPivot = TalonSRXFactory.createDefaultTalon("Left Pivot Motor", Ports.LEFT_PIVOT);
        configureMotor(mLeftPivot, false);
        mRightPivot = TalonSRXFactory.createDefaultTalon("Left Pivot Motor", Ports.RIGHT_PIVOT);
        configureMotor(mRightPivot, false);
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

    private void configureMotor(LazyTalonSRX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
    }

    public LazyTalonFX getLeftClimb(){return mLeftClimb;}
    public LazyTalonFX getRightClimb(){return mRightClimb;}
    public LazyTalonSRX getLeftPivot(){return mLeftPivot;}
    public LazyTalonSRX getRightPivot(){return mRightPivot;}


    public void setPercentPower(LazyTalonFX talon, double power){
        talon.set(ControlMode.PercentOutput, 0.5);
    } 

    public void setPercentPower(LazyTalonSRX talon, double power){
        talon.set(ControlMode.PercentOutput, 0.5);
    }    

    public void moveToPosition(LazyTalonFX talon, double position){
        talon.set(ControlMode.PercentOutput, 0.5);
    } 

    public void moveToPosition(LazyTalonSRX talon, double position){
        talon.set(ControlMode.PercentOutput, 0.5);
    }


    public void climbPower(double power){
        setPercentPower(mLeftClimb, power);
        setPercentPower(mRightClimb, power);
    }

    public void pivotPower(double power){
        setPercentPower(mLeftPivot, power);
        setPercentPower(mRightPivot, power);
    }
    
    public void climbToPosition(int position){
        moveToPosition(mLeftClimb, position);
        moveToPosition(mRightClimb, position);
    }

    public void pivotToPosition(int position){
        moveToPosition(mLeftPivot, position);
        moveToPosition(mRightPivot, position);
    }

    public void resetAllPositions(){
        mLeftClimb.setSelectedSensorPosition(0);
        mRightClimb.setSelectedSensorPosition(0);
        mLeftPivot.setSelectedSensorPosition(0);
        mRightPivot.setSelectedSensorPosition(0);
    }

    
    @Override
    public void periodic(){  
        SmartDashboard.putNumber("Pivot Right Position", mRightPivot.getSelectedSensorPosition());
        SmartDashboard.putNumber("Pivot Left Position", mLeftPivot.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb Right Position", mRightClimb.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb Left Position", mLeftClimb.getSelectedSensorPosition());
    }

}

//left is 42, right 41