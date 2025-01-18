// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class AutoCenterStart 
{
   public static Command create(int level, boolean isRed)
   {
    //TODO: fill out this auto por favor
        return new WaitCommand(5);
   }
}