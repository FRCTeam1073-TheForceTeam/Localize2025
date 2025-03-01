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
  double xToDrive;
  double thetaVelocity;
  double vx;
  int sign = 0;
  PIDController thetaController;
  PIDController vxController;
  
  public LidarAlign(Lidar lidar, Drivetrain drivetrain) {
    this.lidar = lidar;
    this.drivetrain = drivetrain;
    thetaController = new PIDController(0.8, 0, 0.01);
    vxController = new PIDController(0.8, 0, 0.01);
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
      if(lidar.getCovxy() < 0 && !lidar.getCovxyAtZero() && !lidar.getCovxyIsBad()) sign = 1;
      if(lidar.getCovxy() > 0 && !lidar.getCovxyAtZero() && !lidar.getCovxyIsBad()) sign = -1;
      if(lidar.getCovxy() == 0 && !lidar.getCovxyAtZero() && !lidar.getCovxyIsBad()) sign = 0;

      thetaVelocity = sign * 0.2;
      // thetaVelocity = thetaController.calculate(drivetrain.getWrappedHeadingRadians(), drivetrain.getWrappedHeadingRadians() + angleToRotate);
      // thetaVelocity = MathUtil.clamp(thetaVelocity, -2, 2);
      // SmartDashboard.putNumber("LidarAlign theta velocity", thetaVelocity);
      // drivetrain.setTargetChassisSpeeds(

      //   ChassisSpeeds.fromFieldRelativeSpeeds(
      //     0, 
      //     0, 
      //     thetaVelocity, 
      //     Rotation2d.fromDegrees(drivetrain.getHeadingDegrees())));
      xToDrive = lidar.getMeanX() - 0.4;
      if(xToDrive > 0){
        vx = vxController.calculate(drivetrain.getOdometryX(), drivetrain.getOdometryX() + xToDrive);
        if(vx < 0){
          vx = MathUtil.clamp(vx, -2, -0.2);
        }
        if(vx > 0){
          vx = MathUtil.clamp(vx, 0.2, 2);
        }
        if(lidar.getCovxyAtZero()){
          thetaVelocity = 0;
        }
        drivetrain.setTargetChassisSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(
           vx, 
           0, 
           thetaVelocity, 
           Rotation2d.fromDegrees(drivetrain.getHeadingDegrees())));
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setTargetChassisSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(lidar.getMeanX() <= 0.42 && lidar.getCovxyAtZero()){
      return true;
    }

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