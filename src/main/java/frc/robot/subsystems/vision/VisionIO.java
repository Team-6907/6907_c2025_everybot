package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionIO.ObservationType;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.TargetCorner;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public GamePieceObservation[] gamePieceObservations = new GamePieceObservation[0];
    List<TargetCorner> corners = new ArrayList<>();
  }

  /**
   * Represents a robot pose sample used for pose estimation.
   *
   * <p>The main vision subsystem can then decide to accept or reject it
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double yaw,
      double pitch,
      double area,
      ObservationType type) {}

  public static record GamePieceObservation(
      double timestamp,
      double yaw,
      double pitch,
      double area,
      List<TargetCorner> corners,  
      ObservationType type) {}

  public static enum ObservationType {
    PHOTONVISION
  }
  


  public default void updateInputs(VisionIOInputs inputs) {}
}
