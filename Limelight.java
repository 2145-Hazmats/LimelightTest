// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class Limelight extends SubsystemBase {
  private final DriveTrain m_swerve;

  private double targetDistance;
  private double targetLateral;
  private double targetRotation;

  private PathPlannerPath path;


  /** Creates a new Limelight. */
  public Limelight(DriveTrain m_subsystem) {
    m_swerve = m_subsystem;
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("get TV", LimelightHelpers.getTV(""));
    SmartDashboard.putNumber("getFiducialID", LimelightHelpers.getFiducialID(""));
    SmartDashboard.putNumber("get Target Area", LimelightHelpers.getTA(""));
    SmartDashboard.putNumber("getBotPose2d X", LimelightHelpers.getBotPose2d("").getX());
    SmartDashboard.putNumber("getBotPose2d Y", LimelightHelpers.getBotPose2d("").getY());
    SmartDashboard.putNumber("getLatency_Pipeline", LimelightHelpers.getLatency_Pipeline(""));
  }


  public void generateLimelightPath() {
    targetDistance = -(getCameraTransform(2));
    targetLateral =  getCameraTransform(0);
    targetRotation = 0.0; // TODO: SET THIS

    SmartDashboard.putNumber("Target Distance", targetDistance);
    SmartDashboard.putNumber("Target Lateral", targetLateral);
    SmartDashboard.putNumber("Target Height", getCameraTransform(1));
    SmartDashboard.putNumber("Target Rotation", targetRotation);

    final List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(targetLateral, targetDistance, Rotation2d.fromDegrees(targetRotation)) //targetLateral and targetDistance might need to be swapped
    );
    path = new PathPlannerPath(
      bezierPoints,
      new PathConstraints(1, 1, 1, 1), //TODO: Set values
      new GoalEndState(0, Rotation2d.fromDegrees(targetRotation)));
    path.preventFlipping = true;
  }


  public Command followPathCommand() {
    return new FollowPathHolonomic(
      path,
      m_swerve::getPose,                // Need some custom methods in the swerve drivetrain
      m_swerve::getRobotRelativeSpeeds,
      m_swerve::driveRobotRelative,
      new HolonomicPathFollowerConfig(             // TODO: Set values
        new PIDConstants(1.0, 0.0, 0.0),  // Translation PID
        new PIDConstants(1.0, 0.0, 0.0),  // Rotation PID
        2.0,                        // Max module speed in m/s
        0.5,                       // drive base radius in meters
        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      m_swerve // subsytem requirements of the command
    );
  }


  // Custom method to get camtran from network table
  public double getCameraTransform(int index) {
    double[] camtrans = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
    return camtrans[index];
  }
}
