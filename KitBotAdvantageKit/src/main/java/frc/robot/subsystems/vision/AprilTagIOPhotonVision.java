// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.security.Timestamp;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class AprilTagIOPhotonVision implements AprilTagIO {

  protected PhotonCamera camera;
  protected Transform3d robotToCamera = null;

  //variables for periodic
  protected List<PoseObservation> poseObservations = new LinkedList<>();
  protected Transform3d fieldToCamera;
  protected Transform3d fieldToRobot;
  protected Pose3d robotPose;
  protected Transform3d fieldToTarget;
  protected Transform3d cameraToTarget;

  public AprilTagIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
  } 

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.isTrying = true;
    inputs.isConnected = camera.isConnected();
    if (inputs.isConnected) {
      //Read unread results
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();

      poseObservations.clear();

      for (PhotonPipelineResult result : results) {
        if (result.multitagResult.isPresent()) {
          var multitagResult = result.getMultiTagResult().get();
          fieldToCamera = multitagResult.estimatedPose.best;
          fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
          poseObservations.add(new PoseObservation(multitagResult.estimatedPose.ambiguity, robotPose, result.getTimestampSeconds()));
        } else if (result.hasTargets()) {
          //Single Target
          var target = result.getBestTarget();
          var tagPose = VisionConstants.kTagLayout.getTagPose(target.fiducialId);
          if (tagPose.isPresent()) {
            fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
            cameraToTarget = target.bestCameraToTarget;
            fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
            poseObservations.add(new PoseObservation(target.poseAmbiguity, robotPose, result.getTimestampSeconds()));
          }
        }
      }

      inputs.hasTarget = !poseObservations.isEmpty();

    } else {
      inputs.hasTarget = false;
    }
  }
}
