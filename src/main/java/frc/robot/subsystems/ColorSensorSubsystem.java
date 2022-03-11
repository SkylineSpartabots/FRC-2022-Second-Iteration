package frc.robot.subsystems;


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Objects;

public class ColorSensorSubsystem extends SubsystemBase {
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;
    private static final I2C.Port kMxpI2C = I2C.Port.kMXP;
    private static final ColorMatch m_colorMatcher = new ColorMatch();
    private static ColorSensorSubsystem mInstance;
    //Initialize other sensors with this method
    private static TunedColorSensor m_intakeSensor;
    private static TunedColorSensor m_indexerSensor;
    private ColorSensorSubsystem() {
        m_colorMatcher.addColorMatch(Constants.kColorSenorBlue);
        m_colorMatcher.addColorMatch(Constants.kColorSenorRed);
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

    private static Color getMatchedColor(Color detected) {
        return m_colorMatcher.matchClosestColor(detected).color;
    }

    private static boolean isAllianceBall(Color detected, Color allianceColor) {
        final Color match = getMatchedColor(detected);
        return match.equals(allianceColor);
    }

    public static int getProximity(TunedColorSensor tunedSensor) {
        return tunedSensor.colorSensor.getProximity();
    }

    public static boolean isBallLoaded(TunedColorSensor tunedSensor) {
        return getProximity(tunedSensor) >= tunedSensor.distanceThreshold;
    }
    @Override
    public void periodic() {
        Color detectedColor = m_indexerSensor.colorSensor.getColor();

        SmartDashboard.putNumber("proximity", getProximity(m_indexerSensor));
        SmartDashboard.putNumber("red", detectedColor.red);
        SmartDashboard.putNumber("blue", detectedColor.blue);
        SmartDashboard.putNumber("green", detectedColor.green);
        SmartDashboard.putBoolean("alliance", isAllianceBall(detectedColor, Constants.kColorSenorBlue));
        SmartDashboard.putBoolean("loaded?", isBallLoaded(m_indexerSensor));
    }
    /* TODO (in general)
     *  implement a way to check if a ball is loaded - done, see isBallLoaded
     *  create 3 color sensors rather than the 1 in code
     *  periodic
     *  tune distToBallLoaded var and the Color constants
     *  ensure the i2cport var works
     */
}
