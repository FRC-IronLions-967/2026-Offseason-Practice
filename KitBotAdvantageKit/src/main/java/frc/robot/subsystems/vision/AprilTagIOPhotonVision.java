// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class AprilTagIOPhotonVision implements AprilTagIO {

  protected PhotonCamera camera;
  protected Transform3d robotToCamera = null;

  //variables for periodic
  protected List<PoseObservation> poseObservations = new LinkedList<>();
  protected List<TargetInfo> targetInfos = new LinkedList<>();
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
      poseObservations.clear();
      targetInfos.clear();
      inputs.poseObservations = new PoseObservation[0];
      inputs.targetInfo = new TargetInfo[0];
      var results = camera.getAllUnreadResults();
      if (!results.isEmpty()) {
        var result = results.get(results.size() - 1);
        inputs.hasTarget = result.hasTargets();
        // update inputs
        if (inputs.hasTarget) {
          targetInfos.clear();
          for (var target : result.targets) {
            targetInfos.add(
                new TargetInfo(target.getFiducialId(), target.getYaw(), target.getPitch(), target.getBestCameraToTarget().getTranslation().getNorm()));
          }
          // update pose
          if (result.multitagResult.isPresent()) {
            inputs.poseType = PoseTypes.MULTI;

            var multitagResult = result.getMultiTagResult().get();

            fieldToCamera = multitagResult.estimatedPose.best;
            Logger.recordOutput("fieldToCamera", fieldToCamera);
            fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            double totalTagDistance = 0.0;
            for (var target : result.targets) {
              totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            }

            poseObservations.add(new PoseObservation(
              multitagResult.estimatedPose.ambiguity, 
              robotPose, 
              result.getTimestampSeconds(), 
              totalTagDistance/result.targets.size(), 
              multitagResult.fiducialIDsUsed.size()
            ));

          } else if (!result.targets.isEmpty()) {
            inputs.poseType = PoseTypes.SINGLE;
            var target = result.getTargets().get(0);

            var tagPose = VisionConstants.kTagLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
              fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
              cameraToTarget = target.bestCameraToTarget;
              fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
              fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
              robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

              poseObservations.add(new PoseObservation(
                target.poseAmbiguity, 
                robotPose, 
                result.getTimestampSeconds(), 
                cameraToTarget.getTranslation().getNorm(), 
                1
              ));

            }
          }
          inputs.targetInfo = new TargetInfo[targetInfos.size()];
          for (int i = 0; i < targetInfos.size(); i++) {
            inputs.targetInfo[i] = targetInfos.get(i);
          }
          inputs.poseObservations = new PoseObservation[poseObservations.size()];
          for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
          }
        }
      } else {
        inputs.hasTarget = false;
      } 
    }
  }
}
