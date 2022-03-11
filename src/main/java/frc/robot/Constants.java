// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;

//holds robot-wide constants
public final class Constants {
    public static final Color kColorSenorBlue = new Color(0.153076171875,0.39892578125, 0.448486328125);
    public static final Color kColorSenorRed = new Color(0.532470703125, 0.340576171875, 0.12744140625);
    public static final double kColorSensorLoadingDistance = 200;
}
