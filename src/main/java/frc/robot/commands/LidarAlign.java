// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lidar;

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
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasLine = (lidar.getLine() != null);
    if(hasLine){
      /*1. calculate slope of line detected by lidar */
      double[] line = lidar.getLine().clone();
      lidarSlope = -line[0]/line[1];        
      /* 2. Find arctan of the difference between their slopes - angle the robot needs to move */
      angleToRotate = Math.atan(lidarSlope);
      /* 3. rotate the robot that to that set angle*/
      // TODO: fix thetaController/check if it's null??
      thetaVelocity = MathUtil.clamp(thetaController.calculate(drivetrain.getWrappedHeadingRadians(), drivetrain.getWrappedHeadingRadians() + angleToRotate), -2.0, 2.0);
      drivetrain.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, thetaVelocity, new Rotation2d(drivetrain.getWrappedHeadingRadians())));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!hasLine){
      return true;
    } 
    if(Math.abs(lidarSlope) < 0.07){
      return true;
    }
    else{
      return false;
    }
  }
}