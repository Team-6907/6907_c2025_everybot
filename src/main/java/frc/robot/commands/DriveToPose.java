// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.commands;

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

public class DriveToPose extends Command {
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius");

  static {
    drivekP.initDefault(4.0); // 4.0
    drivekD.initDefault(0.0);
    thetakP.initDefault(6.5); // 4.5
    thetakD.initDefault(0.0);
    driveMaxVelocity.initDefault(3.0); // 1.2 2.0
    driveMaxAcceleration.initDefault(2.5); // 2.0
    thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
    thetaMaxAcceleration.initDefault(8.0);
    driveTolerance.initDefault(0.01);
    thetaTolerance.initDefault(Units.degreesToRadians(1.0));
    ffMinRadius.initDefault(0.1);
    ffMaxRadius.initDefault(0.15);
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  @AutoLogOutput(key = "DriveToPose/AtGoal")
  private boolean atGoalFlag = false;

  private static BooleanSupplier atGoalSupplier = () -> false;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  @Getter private boolean running = false;

  private static Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private static DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public static void setFeedforward(
      Supplier<Translation2d> linear, DoubleSupplier omega, BooleanSupplier goalSupplier) {
    linearFF = linear;
    omegaFF = omega;
    atGoalSupplier = goalSupplier;
  }

  /*private enum DriveState {
    ROTATE_TO_TARGET,
    DRIVE_TO_TARGET,
    ROTATE_TO_FINAL
  }

  private DriveState currentState = DriveState.ROTATE_TO_TARGET;*/

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds fieldVelocity = drive.getDriveSpeedFieldRelative();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();

    atGoalFlag = false;
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // use a small feedforward scaler if distance to target is small （0.10-0.15m？)
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;

    // reset integral term which we don't need, the setpoint should not change because we pass in
    // the same value
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);

    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(
                driveErrorAbs,
                0.0); // this updates the current setpoint = the current distance to target

    // don't move if at goal
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose
                    .getTranslation()
                    .minus(targetPose.getTranslation())
                    .getAngle()) // angle from target to current pose
            .transformBy(Util.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(Util.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    atGoalFlag = atGoalSupplier.getAsBoolean();


    if (atGoalFlag) {
      Translation2d manualDriveVelocity = linearFF.get();
      double manualThetaVelocity = thetaVelocity;
      drive.runClosedLoop(
          new ChassisSpeeds(
              manualDriveVelocity.getX(), manualDriveVelocity.getY(), manualThetaVelocity));
    } else {

      Rotation2d headingToTarget =
          currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

      thetaVelocity =
              thetaController.calculate(
                  currentPose.getRotation().getRadians(), headingToTarget.getRadians());

      Translation2d robotRelativeVelocity =
              driveVelocity.rotateBy(headingToTarget.unaryMinus());
      
      drive.runClosedLoop(new ChassisSpeeds(
                robotRelativeVelocity.getX(),  
                0.0,                           
                thetaVelocity));
        

      if (currentDistance < driveController.getPositionTolerance()) {
            thetaVelocity =
                thetaController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
  
            drive.runClosedLoop(new ChassisSpeeds(
                                0.0, 
                                0.0, 
                                thetaVelocity));
          }

      /*Rotation2d headingToTarget =
          currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

      switch (currentState) {
        case ROTATE_TO_TARGET:
          // rotate to goal direction
          thetaVelocity =
              thetaController.calculate(
                  currentPose.getRotation().getRadians(), headingToTarget.getRadians());

          if (Math.abs(currentPose.getRotation().minus(headingToTarget).getRadians())
              < thetaController.getPositionTolerance()) {
            currentState = DriveState.DRIVE_TO_TARGET;
          }

          drive.runClosedLoop(new ChassisSpeeds(0.0, 0.0, thetaVelocity));
          break;

        case DRIVE_TO_TARGET:
          // keep direction
          Translation2d robotRelativeVelocity =
              driveVelocity.rotateBy(headingToTarget.unaryMinus());

          if (currentDistance < driveController.getPositionTolerance()) {
            currentState = DriveState.ROTATE_TO_FINAL;
          }

          drive.runClosedLoop(new ChassisSpeeds(robotRelativeVelocity.getX(), 0.0, 0));
          break;

        case ROTATE_TO_FINAL:
          // rotate to goal rotation
          thetaVelocity =
              thetaController.calculate(
                  currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

          drive.runClosedLoop(new ChassisSpeeds(0.0, 0.0, thetaVelocity));
          break;*/
      }

      // Log data
      Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
      Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
      Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
      Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
      Logger.recordOutput(
          "DriveToPose/Setpoint",
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position)));
      Logger.recordOutput("DriveToPose/Goal", targetPose);
    }
  

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
