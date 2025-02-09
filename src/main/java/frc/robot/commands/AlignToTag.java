// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

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
import frc.robot.subsystems.FieldMap;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command 
{
  Drivetrain drivetrain;
  Localizer localizer;
  FieldMap fieldMap;
  int aprilTagID;
  Pose2d targetPose;
  PIDController xController;
  PIDController yController;
  PIDController thetaController;
  double xVelocity;
  double yVelocity;
  double wVelocity;

  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second

  Map<Integer, Double> tagThetas = new HashMap<>();


  /** Creates a new alignToTag. */
  public AlignToTag(Drivetrain drivetrain, Localizer localizer, FieldMap fieldMap) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.fieldMap = fieldMap;
    xVelocity = 0;
    yVelocity = 0;
    wVelocity = 0;
    tagThetas.put(11, Math.PI / 3);
    tagThetas.put(10, 0.0);
    tagThetas.put(9, 5 * Math.PI / 3);
    tagThetas.put(8, 4 * Math.PI / 3);
    tagThetas.put(7, Math.PI);
    tagThetas.put(6, 2 * Math.PI / 3);

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
    // targetPose = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  public Command alignToTag(int slot) {
    Pose2d currentPose = localizer.getPose();

    if (aprilTagID == -1)
    {
      aprilTagID = fieldMap.getBestAprilTagID(currentPose);
      targetPose = fieldMap.getBestTagPose(aprilTagID, slot, 0.5);
    }

    if (targetPose == null)
    {
      return null;
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
    return null; //TODO: re-write (this is definatly not the right way to do it)
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
