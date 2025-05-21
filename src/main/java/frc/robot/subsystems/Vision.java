// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.Radian;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DualEdgeDelayedBoolean;
import frc.robot.util.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import frc.robot.subsystems.vision.VisionIO.ObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.TargetCorner;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final Supplier<Translation2d> velocitySupplier;

  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private Optional<Rotation2d> leftAlgaeObservation = Optional.empty();
  private Optional<Rotation2d> rightAlgaeObservation = Optional.empty();
  private Pose3d rawPose3d;
  private final Drive drive; 
  private double leftAlgaeObservationTimestamp = 0.0;
  private double rightAlgaeObservationTimestamp = 0.0;
  private static final LoggedTunableNumber algaePersistanceTime =
  new LoggedTunableNumber("RobotState/AlgaePersistanceTime", 0.1);


  // keep track of multi-tag usage with a delayed boolean for each camera
  private final DualEdgeDelayedBoolean[] multiTagDelayedBooleans;
  private final Translation2d[] lastAcceptedPose2d; // For velocity gating
  private final double[] lastAcceptedTimestamp;

  public Vision(VisionConsumer consumer, Supplier<Translation2d> velocitySupplier, Drive drive, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.drive = drive;
    this.velocitySupplier = velocitySupplier;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // delayed boolean and last accepted pose for each camera
    multiTagDelayedBooleans = new DualEdgeDelayedBoolean[io.length];
    lastAcceptedPose2d = new Translation2d[io.length];
    lastAcceptedTimestamp = new double[io.length];
    for (int i = 0; i < io.length; i++) {
      multiTagDelayedBooleans[i] = new DualEdgeDelayedBoolean(MULTI_TAG_DELAY, EdgeType.RISING);
      lastAcceptedPose2d[i] = null;
      lastAcceptedTimestamp[i] = 0.0;
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      if (!inputs[cameraIndex].connected) {
        continue;
      }

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // multi tag usage check
      boolean sawMultiTag = false;
      for (PoseObservation obs : inputs[cameraIndex].poseObservations) {
        if (obs.tagCount() > 1) {
          sawMultiTag = true;
          break;
        }
      }
      boolean multiTagActive = multiTagDelayedBooleans[cameraIndex].update(sawMultiTag);

      // Loop over pose observations
      for (PoseObservation obs : inputs[cameraIndex].poseObservations) {
        double timestamp = obs.timestamp();
        Pose3d rawPose = obs.pose();
        robotPoses.add(rawPose);

        // -------- 1) Basic checks like tagCount > 0, ambiguity, Z margin ------
        // we dont trust multitag this year
        if (obs.tagCount() != 1) {
          robotPosesRejected.add(rawPose);
          continue;
        }
        if (obs.ambiguity() > ACCEPTABLE_AMBIGUITY_THRESHOLD) {
          robotPosesRejected.add(rawPose);
          continue;
        }
        if (obs.ambiguity() == 0) {
          robotPosesRejected.add(rawPose);
          continue;
        }

        if (Math.abs(rawPose.getZ()) > Z_MARGIN) {
          robotPosesRejected.add(rawPose);
          continue;
        }
        if (obs.area() < MIN_AREA[cameraIndex]) {
          robotPosesRejected.add(rawPose);
          continue;
        }

        // yaw pitch filter
        boolean yawPitchReject =
            (Math.abs(obs.yaw()) > ACCEPTABLE_YAW_THRESHOLD)
                || (Math.abs(obs.pitch()) > ACCEPTABLE_PITCH_THRESHOLD);
        if (yawPitchReject) {
          robotPosesRejected.add(rawPose);
          continue;
        }

        // -------- 2) Velocity gating -----------
        // Translation2d chassisVelocity = velocitySupplier.get();

        // if (lastAcceptedPose2d[cameraIndex] != null
        // && Math.abs(timestamp - lastAcceptedTimestamp[cameraIndex]) > 0.0001) {
        // double dt = timestamp - lastAcceptedTimestamp[cameraIndex];
        // Translation2d newT2d = rawPose.getTranslation().toTranslation2d();
        // Translation2d oldT2d = lastAcceptedPose2d[cameraIndex];
        // Translation2d visionVel = newT2d.minus(oldT2d).div(dt);

        // double deltav = visionVel.minus(chassisVelocity).getNorm();
        // if ((chassisVelocity.getNorm() < 0.5 && deltav > STATIC_DELTAV_BOUND)
        // || (chassisVelocity.getNorm() >= 0.5 && deltav > MOVING_DELTAV_BOUND)) {
        // // Reject
        // robotPosesRejected.add(rawPose);
        // continue;
        // }
        // }
        // Passed all filters => ACCEPT
        robotPosesAccepted.add(rawPose);
        lastAcceptedPose2d[cameraIndex] = rawPose.getTranslation().toTranslation2d();
        lastAcceptedTimestamp[cameraIndex] = timestamp;

        // ==== 3) Compute standard deviations ====
        double distFactor = Math.pow(obs.averageTagDistance(), 2.0) / Math.max(obs.tagCount(), 1);
        double linearStdDev = linearStdDevBaseline * distFactor;
        double angularStdDev = angularStdDevBaseline * distFactor;
        boolean updateHeading = false;

        // If multi-tag is active and we actually used multiple tags => better angle
        if (multiTagActive
            && obs.tagCount() > 1
            && obs.type() == ObservationType.PHOTONVISION) {
          // good angle
          updateHeading = true;
        } else {
          angularStdDev *= 2.0;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // ======= 4) Send the final observation to your pose consumer =======
        Matrix<N3, N1> measurementStdDevs =
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
        consumer.accept(rawPose.toPose2d(), timestamp, measurementStdDevs, updateHeading);
        rawPose3d = rawPose;
      }


      // =========== Log results for this camera ===========
      recordLogs(cameraIndex, tagPoses, robotPoses, robotPosesAccepted, robotPosesRejected);
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
      



      for (GamePieceObservation obs : inputs[cameraIndex].gamePieceObservations) {
        double timestamp = obs.timestamp();
        double yaw = obs.yaw();
        double pitch = obs.pitch();
        double area = obs.area();
        List<TargetCorner> corners = obs.corners();


        if (corners != null && !corners.isEmpty()) {
        
          double maxTy = Math.max(corners.get(2).y, corners.get(3).y);
          if (maxTy < 0.4) {
            continue;
          }

          double size = Math.abs(corners.get(0).x - corners.get(1).x);
          double angle = -(corners.get(0).x + corners.get(1).x) / 2.0;
          
        if (cameraIndex == 0) {
          
          leftAlgaeObservation = Optional.ofNullable(new Rotation2d(angle));
          leftAlgaeObservationTimestamp = Timer.getTimestamp();

          if (Timer.getTimestamp() - leftAlgaeObservationTimestamp > algaePersistanceTime.get()) {
            leftAlgaeObservation = Optional.empty();

        }
        } else if (cameraIndex == 1) {

          if (Timer.getTimestamp() - rightAlgaeObservationTimestamp > algaePersistanceTime.get()) {
            rightAlgaeObservation = Optional.empty();
          }
          rightAlgaeObservation = Optional.ofNullable(new Rotation2d(angle));
          rightAlgaeObservationTimestamp = Timer.getTimestamp();
        
        }
        
        if (leftAlgaeObservation.isPresent() && rightAlgaeObservation.isPresent()) {
          var leftVector = new Pose3d()
            .plus(VisionConstants.robotToCamera0)  
            .toPose2d() 
            .transformBy(new Transform2d(0.0, 0.0, leftAlgaeObservation.orElse(new Rotation2d()))); 
          var rightVector = new Pose3d()
            .plus(VisionConstants.robotToCamera1)  
            .toPose2d()
            .transformBy(new Transform2d(0.0, 0.0, leftAlgaeObservation.orElse(new Rotation2d())));

          double mLeft = leftVector.getRotation().getTan();
          double mRight = rightVector.getRotation().getTan();
          
          /*y = mLeft * x + b1  
            y = mRight * x + b2
            b1 = leftVector.getY() - mLeft * leftVector.getX()
            b2 = rightVector.getY() - mRight * rightVector.getX()
            point equation:
              mLeft * x + leftVector.getY() - mLeft * leftVector.getX() 
              = mRight * x + rightVector.getY() - mRight * rightVector.getX()
            result：
              x = (rightVector.getY() - leftVector.getY() + 
              mRight * rightVector.getX() - mLeft * leftVector.getX()) / (mLeft - mRight)
              double algaeY = mLeft * (algaeX - leftVector.getX()) + leftVector.getY();
            */
          double algaeX = (rightVector.getY() - leftVector.getY()) / (mLeft - mRight);
          double algaeY = mLeft * algaeX + leftVector.getY();

          Translation2d algaeTranslation = drive.getPose()
              .transformBy(GeomUtil.toTransform2d(algaeX, algaeY))
              .getTranslation();
          Translation3d algaeTranslations = new Translation3d(
              algaeTranslation.getX(),
              algaeTranslation.getY(),
              FieldConstants.algaeDiameter / 2.0
          );


      }
    




    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
    }
  }


  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs,
        boolean updateHeading);
  }

  private void recordLogs(
      int cameraIndex,
      List<Pose3d> tagPoses,
      List<Pose3d> cameraPoses,
      List<Pose3d> cameraPosesAccepted,
      List<Pose3d> cameraPosesRejected) {

    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/RobotPoses", cameraPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
        cameraPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
        cameraPosesRejected.toArray(new Pose3d[0]));
  }

  public static void main(String[] args) {
    Pose3d pose =
        new Pose3d(
            10.42,
            4.44,
            0.0,
            new Rotation3d(Degree.of(-0.03), Degree.of(-0.39), Degree.of(-10.73)));
    Transform3d tras =
        new Transform3d(
            0.32, -0.29, 0.25, new Rotation3d(0.0, -5 * (Math.PI / 180), Degree.of(15).in(Radian)));
    pose = pose.plus(robotToCamera1).plus(tras.inverse());
    System.out.println(
        String.format(
            "%.3f  %.3f  %.3f  %.3f  %.3f  %.3f",
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getMeasureX().in(Degree),
            pose.getRotation().getMeasureY().in(Degree),
            pose.getRotation().getMeasureZ().in(Degree)));
  }
}
