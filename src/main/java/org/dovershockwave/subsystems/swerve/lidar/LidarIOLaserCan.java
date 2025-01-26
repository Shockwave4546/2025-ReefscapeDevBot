package org.dovershockwave.subsystems.swerve.lidar;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class LidarIOLaserCan implements LidarIO {
  private final LaserCan laserCan = new LaserCan(41);

  public LidarIOLaserCan() {
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override public void updateInputs(LidarIOInputs inputs) {
    final var measurement = laserCan.getMeasurement();
    final var hasValidMeasurement = measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    if (hasValidMeasurement) {
      inputs.distanceMeters = measurement.distance_mm / 1000.0;
    } else {
      inputs.distanceMeters = Double.MIN_VALUE;
    }
  }
}