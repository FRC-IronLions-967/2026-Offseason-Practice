// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AprilTagIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean isConnected = false;
    public boolean hasTarget = false;
    public TargetInfo[] targetInfo;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public PoseTypes poseType;
    public boolean isTrying = false;
  }

  public enum PoseTypes {SINGLE, MULTI};

  public static record TargetInfo(int tagID, double targetYaw, double targetPitch, double distanceToTarget) {}

  public static record PoseObservation(
      double ambiguity, Pose3d pose, double timestamp, double avgTagDistance, int tagCount) {}

  public static record VisionPoseObs(Pose2d poseObs, boolean poseObsGood, double timestamp) {}

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
