package org.dovershockwave.subsystems.swerve;

import edu.wpi.first.math.util.Units;
import org.dovershockwave.utils.PIDFGains;

public class SwerveConstants {
  public static final double ODOMETRY_FREQUENCY = 100.0; // Hz

  // Drive Motor Can ID Format = 1x
  public static final int FRONT_LEFT_DRIVE_ID = 10;
  public static final int FRONT_RIGHT_DRIVE_ID = 11;
  public static final int BACK_LEFT_DRIVE_ID = 13;
  public static final int BACK_RIGHT_DRIVE_ID = 14;

  // Turn Motor Can ID Format = 2x
  public static final int FRONT_LEFT_TURN_ID = 20;
  public static final int FRONT_RIGHT_TURN_ID = 21;
  public static final int BACK_LEFT_TURN_ID = 23;
  public static final int BACK_RIGHT_TURN_ID = 24;

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

  // MAXSwerve with 13 pinion teeth and 22 spur teeth
  public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22.0) / (13.0 * 15.0);
  public static final double DRIVE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / DRIVE_MOTOR_REDUCTION;
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((2 * Math.PI) / 60.0) / DRIVE_MOTOR_REDUCTION;
  public static final PIDFGains DRIVE_PIDF = new PIDFGains(0.0, 0.0, 0.0, 0.0);

  public static final boolean TURN_ENCODER_INVERTED = true;
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI;
  public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;
  public static final PIDFGains TURN_PIDF = new PIDFGains(0.0, 0.0, 0.0, 0.0);
}
