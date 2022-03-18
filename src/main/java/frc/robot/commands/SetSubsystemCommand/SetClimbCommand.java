package frc.robot.commands.SetSubsystemCommand;

import java.util.Set;

import javax.naming.InitialContext;

import org.opencv.features2d.MSER;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetClimbCommand extends CommandBase{
    //direction: true for climb up, false for down, side: true for left, false for right.
    private boolean isUp;
    private boolean side;

    private final ClimbSubsystem m_subsystem;

    public SetClimbCommand(boolean isUp, boolean side){
        this.isUp = isUp;
        this.side = side;
        this.m_subsystem = ClimbSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        if(side){
            m_subsystem.setLeftPercentPower(isUp);
        } else {
            m_subsystem.setRightPercentPower(isUp);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
