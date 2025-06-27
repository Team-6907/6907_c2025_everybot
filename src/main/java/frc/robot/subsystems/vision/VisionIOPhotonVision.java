// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;



/**
 * A VisionIO implementation that reads from a PhotonCamera. It creates one or more PoseObservations
 * per cycle (multi-tag or single-tag) and one or more gamePieceObservations, plus a simple best-target yaw/pitch if desired.
 */
public class VisionIOPhotonVision implements VisionIO {
  // dont give too many observations: logs boom!
  public static final int MAX_OBSERVATION_PER_INPUT = 5;

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PriorityQueue<PoseObservation> poseObservations;
  private final PriorityQueue<GamePieceObservation> gamePieceObservations;
  private PhotonTrackedTarget kLastValidTarget = null;
 

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    // smaller timestamps first
    this.poseObservations =
        new PriorityQueue<>(
            MAX_OBSERVATION_PER_INPUT, Comparator.comparingDouble(PoseObservation::timestamp));
    this.gamePieceObservations =
        new PriorityQueue<>(
            MAX_OBSERVATION_PER_INPUT, Comparator.comparingDouble(GamePieceObservation::timestamp));
  }

  private void addObs(PoseObservation obs) {
    if (poseObservations.size() < MAX_OBSERVATION_PER_INPUT) {
      poseObservations.add(obs);
    } else if (poseObservations.peek().timestamp() < obs.timestamp()) {
      poseObservations.poll();
      poseObservations.add(obs);
    }
  }

  private void addObs(GamePieceObservation obs) {
    if (poseObservations.size() < MAX_OBSERVATION_PER_INPUT) {
      gamePieceObservations.add(obs);
    } else if (poseObservations.peek().timestamp() < obs.timestamp()) {
      poseObservations.poll();
      gamePieceObservations.add(obs);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    poseObservations.clear();
    gamePieceObservations.clear();
    inputs.corners.clear(); 

    for (var result : camera.getAllUnreadResults()) {
      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        addObs(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                0.0,
                0.0, // to let it pass the yaw pitch filter
                1.0, // to let it pass the area filter
                ObservationType.PHOTONVISION)); // Observation type
      }
      if (!result.targets.isEmpty()) { // Single tag result
        // you can actually do for all result.targets, why get(0)
        // var target = result.targets.get(0);
        for (var target : result.targets) {
          // Calculate robot pose
          var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
          // be careful if bestCameraToTarget==null!
          if (tagPose.isPresent()) {
            Transform3d fieldToTarget =
                new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
            Transform3d cameraToTarget = target.bestCameraToTarget;
            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            // Add tag ID
            tagIds.add((short) target.fiducialId);

            // Add observation
            addObs(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose, // 3D pose estimate
                    target.poseAmbiguity, // Ambiguity
                    1, // Tag count
                    cameraToTarget.getTranslation().getNorm(), // Average tag distance
                    target.yaw,
                    target.pitch,
                    target.area,
                    ObservationType.PHOTONVISION));
          }
        }
      }
      
      if (!result.targets.isEmpty()) {
        for (var target : result.targets) {
          if (target.getDetectedObjectClassID() >= 0) {  
             //not sure how to discriminate object from tag
             float confidence = target.getDetectedObjectConfidence();
            
             if (confidence < 0.5) continue; 
             List<TargetCorner> corners = target.getMinAreaRectCorners();
             
             if (corners != null && !corners.isEmpty()) {
               inputs.corners.addAll(corners);
             }

             kLastValidTarget = result.getBestTarget();

             addObs(
                new GamePieceObservation(
                    result.getTimestampSeconds(),
                    target.getYaw(),
                    target.getPitch(),
                    target.getArea(),
                    target.getMinAreaRectCorners(),
                    ObservationType.PHOTONVISION));
          }
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations =
        new PoseObservation[Math.min(MAX_OBSERVATION_PER_INPUT, poseObservations.size())];
    for (int i = 0; i < inputs.poseObservations.length; i++) {
      inputs.poseObservations[i] = poseObservations.poll();
    }
    

    inputs.gamePieceObservations =
         new GamePieceObservation[Math.min(MAX_OBSERVATION_PER_INPUT, gamePieceObservations.size())];
    for (int i = 0; i < inputs.gamePieceObservations.length; i++) {
      inputs.gamePieceObservations[i] = gamePieceObservations.poll();
    }



    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    //write corners
    
  }
}
