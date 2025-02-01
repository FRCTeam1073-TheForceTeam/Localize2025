// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Point extends SubsystemBase {
  double x;
  double y;
  /** Creates a new Point. */
  public Point(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public Point(){
    x = 0;
    y = 0;
  }

  public double getX(){
    return x;
  }

  public double getY(){
    return y;
  }

  public void setX(double xVal){
    x = xVal;
  }

  public void setY(double yVal){
    y = yVal;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
