package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopRotateSchema extends MotionSchema 
{
    OI oi;
    double maximumRotationVelocity;
    boolean active;

    public TeleopRotateSchema(OI oi, double maximumRotationVelocity)
    {
        this.oi = oi;
        this.maximumRotationVelocity = maximumRotationVelocity;
        active = true;
    }

    public void setActive(boolean active)
    {
        this.active = active;
    }

    public boolean getActive()
    {
        return active;
    }

    @Override
    public void update(Drivetrain drivetrain)
    {
        //multiples the angle by a number from 1 to the square root of 30:
        double mult1 = 1.0 + (oi.getDriverLeftTrigger() * ((Math.sqrt(25)) - 1));
        double mult2 = 1.0 + (oi.getDriverRightTrigger() * ((Math.sqrt(25)) - 1));

        double rightX = oi.getDriverRotate();

        //sets deadzones on the controller to extend to .05:
        if(Math.abs(rightX) < .15) {rightX = 0;}

        double w = MathUtil.clamp(-(rightX * maximumRotationVelocity / 25) * mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);
        if (active)
        {
            setRotate(w, 1);
        }
        else
        {
            setRotate(0, 0);
        }
    }
}
