// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// todo: this entire script

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;

  private final VisionConsumer consumer;
  private final Supplier<ChassisSpeeds> speedsSupplier;

  private boolean rejectPose;

  private ChassisSpeeds speeds;

  /** Creates a new AprilTagVision. */
  public AprilTagVision(
      VisionConsumer consumer, Supplier<ChassisSpeeds> speedsSupplier, AprilTagIO... io) {
    this.io = io;
    this.consumer = consumer;
    this.speedsSupplier = speedsSupplier;
    this.inputs = new AprilTagIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/AprilTag/Camera" + Integer.toString(i), inputs[i]);
    }

    // update speed supplier here for better runtime
    speeds = speedsSupplier.get();
    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {

      for (var obs : inputs[cameraIndex].poseObservations) {

        Logger.recordOutput(
            "Vision/AprilTag/Camera" + Integer.toString(cameraIndex) + "/distance",
            obs.tagDistance());

        // filtering
        rejectPose =
            obs.ambiguity() > VisionConstants.maxAmbiguity.get()
                || obs.pose().getZ() > VisionConstants.maxZError.get()
                || obs.pose().getX() < 0
                || obs.pose().getY() < 0
                || obs.pose().getX() > VisionConstants.kTagLayout.getFieldLength()
                || obs.pose().getY() > VisionConstants.kTagLayout.getFieldWidth()
                || Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                    > VisionConstants.maxSpeed.get()
                || Math.abs(speeds.omegaRadiansPerSecond) > VisionConstants.maxAngularSpeed.get();

        if (rejectPose) continue;

        double stdDev = VisionConstants.measuredVisionErrorFactor / Math.pow(obs.tagDistance(), 2);

        Pose2d combinedPose =
            new Pose2d(
                obs.pose().getX(),
                obs.pose().getY(),
                new Rotation2d(obs.pose().getRotation().getZ()));

        consumer.accept(
            combinedPose,
            obs.timestamp(),
            VecBuilder.fill(stdDev, stdDev, VisionConstants.estimatedAngleStdDev));
      }
    }
  }

  // Feeds into the drive poseEstimator
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
