// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.vision.AprilTagIO.TargetInfo;

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
  public AprilTagVision(VisionConsumer consumer, Supplier<ChassisSpeeds> speedsSupplier, AprilTagIO... io) {
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

  /**
   * @param cameraIndex what camera to look
   * @param aprilTagID what april tag to look at
   * @return the rot and norm of the specified april tag
   */
  public TargetInfo getTargetInfo(int cameraIndex, int aprilTagID) {
    for (int i = 0; i < inputs[cameraIndex].targetInfo.length; i++) {
      if (inputs[cameraIndex].targetInfo[i].tagID() == aprilTagID) {
        return inputs[cameraIndex].targetInfo[i];
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/AprilTag/Camera" + Integer.toString(i), inputs[i]);
    }

    //update speed supplier here for better runtime
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
            || Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > VisionConstants.maxSpeed.get()
            || Math.abs(speeds.omegaRadiansPerSecond) > VisionConstants.maxAngularSpeed.get();

        if (rejectPose) continue;

        acceptedPoses.add(new Pose3d(
          obs.pose().getX() + VisionConstants.camera0OffsetX,
          obs.pose().getY() + VisionConstants.camera0OffsetY,
          obs.pose().getZ(),
          obs.pose().getRotation()
        ));
        acceptedTimestamps.add(obs.timestamp());
        /*
         * If you have/are taken AP Stat this can make sense. 
         * The base line is in radians/meters and is how far of you think the pose is 68% of the time.
         * Then we can modify it based on how far and how many tags we have. 
         * EG. We have a std dev baseline of 0.1m. Our x component of the standard deviation is 5.0. We have 2 tags(HUB) and they are 3m away. So then the standard deviation would be 0.15m 
         * So there would be a 68% chance that the actual pose would be between 4.85 and 5.15 and a 95% chance that the actual pose would be between 4.7 and 5.3
         */
        double stdFactor = obs.avgTagDistance() / obs.tagCount();
        double linearStdDev = VisionConstants.camera1linearStdDevBaseline * stdFactor;
        double angularStdDev = VisionConstants.camera1angularStdDevBaseline * stdFactor;

        if (obs.tagCount() == 1) {
          linearStdDev *= 2.0;
          angularStdDev = 99999;
        }

        acceptedLinearW.add(1/linearStdDev);
        acceptedAngularW.add(1/angularStdDev);

        // consumer.accept(obs.pose().toPose2d(), obs.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
    }
    double sumX = 0;
    double sumY = 0;
    double sumZ = 0;

    double sumCosYaw = 0;
    double sumSinYaw = 0;

    double sumCosPitch = 0;
    double sumSinPitch = 0;

    double sumCosRoll = 0;
    double sumSinRoll = 0;

    double weightedTimestampSum = 0;
    double totalLinearWeight = 0;
    double totalAngularWeight = 0;

    for (int i = 0; i < acceptedPoses.size(); i++) {

      Pose3d pose = acceptedPoses.get(i);

      double linearW = acceptedLinearW.get(i);
      double angularW = acceptedAngularW.get(i);

      // Weighted position
      sumX += pose.getX() * linearW;
      sumY += pose.getY() * linearW;
      sumZ += pose.getZ() * linearW;

      // Weighted timestamp
      weightedTimestampSum += acceptedTimestamps.get(i) * linearW;

      totalLinearWeight += linearW;

      // Rotation — use sin/cos averaging to avoid wrap issues
      double yaw = pose.getRotation().getZ();
      double pitch = pose.getRotation().getY();
      double roll = pose.getRotation().getX();

      sumCosYaw += Math.cos(yaw) * angularW;
      sumSinYaw += Math.sin(yaw) * angularW;

      sumCosPitch += Math.cos(pitch) * angularW;
      sumSinPitch += Math.sin(pitch) * angularW;

      sumCosRoll += Math.cos(roll) * angularW;
      sumSinRoll += Math.sin(roll) * angularW;

      totalAngularWeight += angularW;
    }

    if (totalLinearWeight > 0 && totalAngularWeight > 0) {

      double fusedX = sumX / totalLinearWeight;
      double fusedY = sumY / totalLinearWeight;
      double fusedZ = sumZ / totalLinearWeight;

      double fusedYaw = Math.atan2(sumSinYaw, sumCosYaw);
      double fusedPitch = Math.atan2(sumSinPitch, sumCosPitch);
      double fusedRoll = Math.atan2(sumSinRoll, sumCosRoll);

      double fusedTimestamp = weightedTimestampSum / totalLinearWeight;

      Pose3d combinedPose =
          new Pose3d(
              fusedX,
              fusedY,
              fusedZ,
              new Rotation3d(fusedRoll, fusedPitch, fusedYaw));

      // Proper fused variance:
      double fusedLinearVar = 1.0 / Math.pow(totalLinearWeight, 2);
      double fusedAngularVar = 1.0 / Math.pow(totalAngularWeight, 2);

      double fusedLinearStd = Math.sqrt(fusedLinearVar);
      double fusedAngularStd = Math.sqrt(fusedAngularVar);

      consumer.accept(
          combinedPose.toPose2d(),
          fusedTimestamp,
          VecBuilder.fill(fusedLinearStd, fusedLinearStd, fusedAngularStd));
    }
  }


  //Feeds into the drive poseEstimator
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
      Pose2d pose,
      double timestamp,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
