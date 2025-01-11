package org.dovershockwave.subsystems.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveSubsystem {
  public static final Lock ODOMETRY_LOCK = new ReentrantLock();
}
