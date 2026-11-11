// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// todo: this entire script

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;

  private final VisionConsumer consumer;
  private final Supplier<ChassisSpeeds> speedsSupplier;

  private List<Pose3d> acceptedPoses;
  private List<Double> acceptedTimestamps;
  private List<Double> acceptedLinearW;
  private List<Double> acceptedAngularW;

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

    acceptedPoses = new ArrayList<>();
    acceptedTimestamps = new ArrayList<>();
    acceptedLinearW = new ArrayList<>();
    acceptedAngularW = new ArrayList<>();
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
    // Update Pose
    if (acceptedPoses.size() > 0) {
      acceptedAngularW.clear();
      acceptedLinearW.clear();
      acceptedPoses.clear();
      acceptedTimestamps.clear();
    }
    acceptedPoses.clear();
    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
      for (var obs : inputs[cameraIndex].poseObservations) {

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

        acceptedPoses.add(
            new Pose3d(
                obs.pose().getX() + VisionConstants.camera0OffsetX,
                obs.pose().getY() + VisionConstants.camera0OffsetY,
                obs.pose().getZ(),
                obs.pose().getRotation()));
        acceptedTimestamps.add(obs.timestamp());
        
      }
    }
    double sumX = 0;
    double sumY = 0;
    /*double sumZ = 0;*/

    double sumYaw = 0;

    double avgTimestamp = 0;

    // double sumCosYaw = 0;
    // double sumSinYaw = 0;
    /*
    double sumCosPitch = 0;
    double sumSinPitch = 0;

    double sumCosRoll = 0;
    double sumSinRoll = 0;
    
    double weightedTimestampSum = 0;
    double totalLinearWeight = 0;
    double totalAngularWeight = 0;
    */

    for (int i = 0; i < acceptedPoses.size(); i++) {
      Pose3d pose = acceptedPoses.get(i);

      sumX += pose.getX();
      sumY += pose.getY();

      sumYaw += pose.getRotation().getZ();
    }

    double avgX = sumX / acceptedPoses.size();
    double avgY = sumY / acceptedPoses.size();

    double avgYaw = sumYaw / acceptedPoses.size();

    //Calculate Standard Deviation
    sumX = 0;
    sumY = 0;

    sumYaw = 0;

    for (int i = 0; i < acceptedPoses.size(); i++) {
      Pose3d pose = acceptedPoses.get(i);
      sumX = pose.getX();
      sumY = pose.getY();

      sumYaw = pose.getRotation().getZ();

      sumX += Math.pow(pose.getX() - avgX, 2);
      sumY += Math.pow(pose.getY() - avgY, 2);

      sumYaw += Math.pow(pose.getRotation().getZ() - avgYaw, 2);
    }

    // Dividing by n
    sumX /= acceptedPoses.size();
    sumY /= acceptedPoses.size();

    sumYaw /= acceptedPoses.size();

    // Standard deviation variables
    double standardDevX = Math.sqrt(sumX);
    double standardDevY = Math.sqrt(sumY);

    double standardDevYaw = Math.sqrt(sumYaw);
    
    Pose2d combinedPose = new Pose2d(avgX, avgY, new Rotation2d(avgYaw)); // Check if in degrees or rad

    // Timestamp average calculation
    for (int i = 0; i < acceptedTimestamps.size(); i++){
      avgTimestamp += acceptedTimestamps.get(i);
    }

    avgTimestamp /= acceptedTimestamps.size();

    consumer.accept(
          combinedPose,
          avgTimestamp,
          VecBuilder.fill(standardDevX, standardDevY, standardDevYaw));
    
    /* 
    if (totalLinearWeight > 0 && totalAngularWeight > 0) {

      double fusedX = sumX / totalLinearWeight;
      double fusedY = sumY / totalLinearWeight;
      double fusedZ = sumZ / totalLinearWeight;

      double fusedYaw = Math.atan2(sumSinYaw, sumCosYaw);
      double fusedPitch = Math.atan2(sumSinPitch, sumCosPitch);
      double fusedRoll = Math.atan2(sumSinRoll, sumCosRoll);

      double fusedTimestamp = weightedTimestampSum / totalLinearWeight;

      Pose3d combinedPose =
          new Pose3d(fusedX, fusedY, fusedZ, new Rotation3d(fusedRoll, fusedPitch, fusedYaw));

      // Proper fused variance:
      double fusedLinearVar = 1.0 / Math.pow(totalLinearWeight, 2);
      double fusedAngularVar = 1.0 / Math.pow(totalAngularWeight, 2);

      double fusedLinearStd = Math.sqrt(fusedLinearVar);
      double fusedAngularStd = Math.sqrt(fusedAngularVar);

      
    }*/
  }

  // Feeds into the drive poseEstimator
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
