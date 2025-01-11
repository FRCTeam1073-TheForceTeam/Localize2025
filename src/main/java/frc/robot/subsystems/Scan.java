// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class LidarData {
    //sampling mode once receives express scan request - get_lidar_conf (command to get typical scan mode)
    //quality, angle, angle, distance, distance
    //actual heading = angle_q6/64.0 degree
    //actual distance = distance_q2/4.0 mm
    private float quality;
    private float angle;
    private float distance;

}
