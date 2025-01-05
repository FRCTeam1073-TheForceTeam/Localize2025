package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopTranslateSchema extends MotionSchema 
{
    OI oi;
    double maximumLinearVelocity;

    public TeleopTranslateSchema(OI oi, double maximumLinearVelocity)
    {
        this.oi = oi;
        this.maximumLinearVelocity = maximumLinearVelocity;
    }

    @Override
    public void update(Drivetrain drivetrain) 
    {
        //multiples the angle by a number from 1 to the square root of 30:
        double mult1 = 1.0 + (oi.getDriverLeftTrigger() * ((Math.sqrt(25)) - 1));
        double mult2 = 1.0 + (oi.getDriverRightTrigger() * ((Math.sqrt(25)) - 1));

        double leftY = oi.getDriverTranslateY();
        double leftX = oi.getDriverTranslateX();

        //sets deadzones on the controller to extend to .05:
        if(Math.abs(leftY) < .15) {leftY = 0;}
        if(Math.abs(leftX) < .15) {leftX = 0;}

        double vx = MathUtil.clamp(-(leftY * maximumLinearVelocity / 25 )* mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        double vy = MathUtil.clamp(-(leftX * maximumLinearVelocity / 25 ) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);

        setTranslate(vx, vy, 1);
    }
}
