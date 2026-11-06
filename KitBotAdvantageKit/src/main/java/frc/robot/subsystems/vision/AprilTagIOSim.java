// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class AprilTagIOSim extends AprilTagIOPhotonVision {
  private static VisionSystemSim visionSim;

  private Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public AprilTagIOSim(String cameraName, Transform3d robotToCamera, Supplier<Pose2d> robotPose) {
    super(cameraName, robotToCamera);

    this.poseSupplier = robotPose;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.kTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, VisionConstants.kTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera != null ? robotToCamera : new Transform3d());
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
