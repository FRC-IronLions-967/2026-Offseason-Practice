// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 3.87;
  public static final double trackWidth = Units.inchesToMeters(21.75);

  // Device CAN IDs
  public static final int pigeonCanId = 9;
  public static final int leftLeaderCanId = 1;
  public static final int leftFollowerCanId = 2;
  public static final int rightLeaderCanId = 3;
  public static final int rightFollowerCanId = 4;

  // Motor configuration
  public static final int currentLimit = 60;
  // Measured Radii Left: 3.08 in, Right: 3.10 in. Tim Scholl, Syl Islam, Levi Richards. 10/17/26
  public static final double wheelRadiusMeters = Units.inchesToMeters(3.1);
  public static final double motorReduction = 8.45;
  public static final boolean leftInverted = false;
  public static final boolean rightInverted = true;
  public static final DCMotor gearbox = DCMotor.getCIM(2);

  // Velocity PID configuration

  // Finalized Kp and Kd. 10/24/26, Tim Scholl, Levi Richards, Syl Islam
  public static final double realKp = 0.155;
  public static final double realKd = 0.0;
  public static final double realKs = 0.12882;
  public static final double realKv = 2.27217;

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;

  // PathPlanner configuration
  public static final double robotMassKg = 20.35;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 0.975;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              gearbox.withReduction(motorReduction),
              currentLimit,
              2),
          trackWidth);
}
