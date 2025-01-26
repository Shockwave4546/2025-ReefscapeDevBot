package org.dovershockwave.subsystems.swerve.lidar;

import au.grapplerobotics.LaserCan;

public class LidarIOLaserCan implements LidarIO {
  private final LaserCan laserCan = new LaserCan(41);
}
