package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class RightL2
{
    public static Command create(boolean isRed, Drivetrain drivetrain)  
    {
        return new WaitCommand(0);
    }  
}
