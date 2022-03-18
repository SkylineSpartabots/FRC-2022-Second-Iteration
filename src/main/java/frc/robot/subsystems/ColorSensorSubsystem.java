package frc.robot.subsystems;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Objects;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

public class ColorSensorSubsystem extends SubsystemBase {
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;
    private static final I2C.Port kMxpI2C = I2C.Port.kMXP;
    private static final ColorMatch m_colorMatcherIntake = new ColorMatch();
    private static final ColorMatch m_colorMatcherIndexer = new ColorMatch();
    private static ColorSensorSubsystem mInstance;
    //Initialize other sensors with this method
    private static TunedColorSensor m_intakeSensor;
    private static TunedColorSensor m_indexerSensor;
    private ColorSensorSubsystem() {
        m_colorMatcherIntake.addColorMatch(Constants.kColorSensorBlueIntake);
        m_colorMatcherIntake.addColorMatch(Constants.kColorSensorRedIntake);
        m_colorMatcherIndexer.addColorMatch(Constants.kColorSensorBlueIndexer);
        m_colorMatcherIndexer.addColorMatch(Constants.kColorSensorRedIndexer);
        m_intakeSensor = new TunedColorSensor(Constants.kColorSensorLoadingDistance, onboardI2C);
        m_indexerSensor = new TunedColorSensor(Constants.kColorSensorIndexerDistance, kMxpI2C);
    }

    private class TunedColorSensor{
        protected ColorSensorV3 colorSensor;
        protected double distanceThreshold;
        public TunedColorSensor(double threshold, I2C.Port port){
            colorSensor = new ColorSensorV3(port);
            distanceThreshold = threshold;
        }
    }
    public static ColorSensorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ColorSensorSubsystem();
        }
        return mInstance;
    }

    private Color getMatchedColorIntake(Color detected) {
        return m_colorMatcherIntake.matchClosestColor(detected).color;
    }
    private Color getMatchedColorIndexer(Color detected) {
        return m_colorMatcherIndexer.matchClosestColor(detected).color;
    }

    private boolean isAllianceBallIntake(Color detected, Color allianceColor) {
        final Color match = getMatchedColorIntake(detected);
        return match.equals(allianceColor);
    }
    private boolean isAllianceBallIndexer(Color detected, Color allianceColor) {
        final Color match = getMatchedColorIndexer(detected);
        return match.equals(allianceColor);
    }

    public int getProximity(TunedColorSensor tunedSensor) {
        return tunedSensor.colorSensor.getProximity();
    }

    public boolean isBallLoaded(TunedColorSensor tunedSensor) {
        return getProximity(tunedSensor) >= tunedSensor.distanceThreshold;
    }
    public boolean isIntakeBallLoaded(){
        return isBallLoaded(m_intakeSensor);
    }
    public boolean isAllianceBallIntake(){
        return isAllianceBallIntake(m_intakeSensor.colorSensor.getColor(), Constants.allianceColorIntake);
    }
    public boolean isIndexerBallLoaded(){
        return isBallLoaded(m_indexerSensor);
    }
    public boolean isAllianceBallIndexer(){
        return isAllianceBallIndexer(m_indexerSensor.colorSensor.getColor(), Constants.allianceColorIndexer);
    }

    @Override
    public void periodic() {
         Color detectedColor = m_indexerSensor.colorSensor.getColor();
        
         SmartDashboard.putNumber("indexer proximity", getProximity(m_indexerSensor));
         SmartDashboard.putNumber("indexer red", detectedColor.red);
         SmartDashboard.putNumber("indexer blue", detectedColor.blue);
         SmartDashboard.putNumber("indexer green", detectedColor.green);
         SmartDashboard.putBoolean("indexer alliance", isAllianceBallIndexer(detectedColor, Constants.kColorSensorBlueIndexer));
         SmartDashboard.putBoolean("indexer loaded?", isBallLoaded(m_indexerSensor));
         
         Color color2 = m_intakeSensor.colorSensor.getColor();
        
         SmartDashboard.putNumber("intake proximity", getProximity(m_intakeSensor));
         SmartDashboard.putNumber("intake red", color2.red);
         SmartDashboard.putNumber("intake blue", color2.blue);
         SmartDashboard.putNumber("intake green", color2.green);
         SmartDashboard.putBoolean("intake alliance", isAllianceBallIntake(color2, Constants.kColorSensorBlueIntake));
         SmartDashboard.putBoolean("intake loaded?", isBallLoaded(m_intakeSensor));
    }
    /* TODO (in general)
     *  implement a way to check if a ball is loaded - done, see isBallLoaded
     *  create 3 color sensors rather than the 1 in code
     *  periodic
     *  tune distToBallLoaded var and the Color constants
     *  ensure the i2cport var works
     */
}
