// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.FieldMap;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command 
{
  Drivetrain drivetrain;
  Localizer localizer;
  FieldMap fieldMap;
  OI oi;
  int aprilTagID;
  Pose2d targetPose;
  PIDController xController;
  PIDController yController;
  PIDController thetaController;
  double xVelocity;
  double yVelocity;
  double wVelocity;
  int slot;

  // private final static double maximumLinearVelocity = 3.5;   // Meters/second
  // private final static double maximumRotationVelocity = 4.0; // Radians/second
  private final static double maximumLinearVelocity = 1.5;   // Meters/second
  private final static double maximumRotationVelocity = 2.0; // Radians/second

  /** Creates a new alignToTag. */
  public AlignToTag(Drivetrain drivetrain, Localizer localizer, FieldMap fieldMap, OI oi) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.fieldMap = fieldMap;
    this.oi = oi;
    xVelocity = 0;
    yVelocity = 0;
    wVelocity = 0;
    slot = -1;

    xController = new PIDController(
      1.1, 
      0.0, 
      0.01
    );

    yController = new PIDController(
      1.1, 
      0.0, 
      0.01
    );

    thetaController = new PIDController(
      1.2, 
      0.0,
      0.01
    );

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    xController.reset();
    yController.reset();
    thetaController.reset();
    aprilTagID = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Pose2d currentPose = localizer.getPose();

    if (oi.getDriverDPadLeft())
    {
      slot = 0;
    }
    else if (oi.getDriverDPadUp())
    {
      slot = 1;
    }
    else if (oi.getDriverDPadRight())
    {
      slot = 2;
    }
    else
    {
      slot = -1;
    }

    if (aprilTagID == -1)
    {
      aprilTagID = fieldMap.getBestAprilTagID(currentPose);
      targetPose = fieldMap.getBestTagPose(aprilTagID, slot, 0.25);
    }

    if (targetPose == null)
    {
      return;
    }

    xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    wVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    xVelocity = MathUtil.clamp(xVelocity, -maximumLinearVelocity, maximumLinearVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maximumLinearVelocity, maximumLinearVelocity);
    wVelocity = MathUtil.clamp(wVelocity, -maximumRotationVelocity, maximumRotationVelocity);

    drivetrain.setTargetChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, wVelocity, localizer.getPose().getRotation())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    aprilTagID = -1;
    targetPose = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
