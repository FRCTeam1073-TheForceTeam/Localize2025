// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LidarAlign extends Command {
  /** Creates a new LidarAlign. */
  Lidar lidar;
  boolean hasLine = true;
  Drivetrain drivetrain;
  double lidarSlope;
  double angleToRotate;
  double thetaVelocity;
  PIDController thetaController;
  
  public LidarAlign(Lidar lidar, Drivetrain drivetrain) {
    this.lidar = lidar;
    this.drivetrain = drivetrain;
    thetaController = new PIDController(0.8, 0, 0.01);
    thetaController.enableContinuousInput(-Math.PI/2, Math.PI/2);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //hasLine = (lidar.getLine() != null);
    //if(hasLine && (lidar.findLineSegment(lidar.getLidarArray()) != null)){
      /*1. calculate slope of line detected by lidar */
      /* 2. Find arctan of the difference between their slopes - angle the robot needs to move */
      //angleToRotate = lidar.getAngleToRotate();
      /* 3. rotate the robot that to that set angle*/
      thetaVelocity = thetaController.calculate(drivetrain.getWrappedHeadingRadians(), drivetrain.getWrappedHeadingRadians() + angleToRotate);
      thetaVelocity = MathUtil.clamp(thetaVelocity, -2, 2);
      SmartDashboard.putNumber("LidarAlign theta velocity", thetaVelocity);
      drivetrain.setTargetChassisSpeeds(

        ChassisSpeeds.fromFieldRelativeSpeeds(
          0, 
          0, 
          thetaVelocity, 
          Rotation2d.fromDegrees(drivetrain.getHeadingDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setTargetChassisSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(lidar.getAngleToRotate()) < 0.05 || Math.abs(lidar.getAngleToRotate()) > 1.7){
    //   return true;
    // }
    // if(Math.abs(lidar.getAngleToRotate()) < 0.02){
    //   return true;
    // }
    // else if(lidar.getLidarArrayTimestamp() - lidar.getFilteredAngleTimestamp() > 1.0){
    //   return true;
    // }
      return false;
  }
}