// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

/** Add your docs here. */
public class AutoLeftStart 
{
    public static Command create(int level, boolean isRed, Drivetrain drivetrain, Localizer localizer)
    {
        switch (level)
        {
            case 0:
               return GenericL0.create(isRed, drivetrain, localizer);
            case 1: 
               return LeftL1.create(isRed, drivetrain);
            case 2:
               return LeftL2.create(isRed, drivetrain);
            case 3:
               return LeftL3.create(isRed, drivetrain);
            case 4:
               return LeftL4.create(isRed, drivetrain);
            default:
            return new WaitCommand(0);
        }
    }
}
