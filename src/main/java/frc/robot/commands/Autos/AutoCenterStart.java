// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class AutoCenterStart 
{
   public static Command create(int level, boolean isRed, Drivetrain drivetrain)
    {
        switch (level)
        {
            case 1: 
               return CenterL1.create(isRed, drivetrain);
            case 2:
               return CenterL2.create(isRed, drivetrain);
            case 3:
               return CenterL3.create(isRed, drivetrain);
            case 4:
               return CenterL4.create(isRed, drivetrain);
            default:
               return new WaitCommand(0);
        }
    }
}