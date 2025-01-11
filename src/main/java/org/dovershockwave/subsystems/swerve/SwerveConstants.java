package org.dovershockwave.subsystems.swerve;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.MotorConstants;
import org.dovershockwave.utils.PIDFGains;

public class SwerveConstants {
  public static final double ODOMETRY_FREQUENCY = 100.0; // Hz

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
  // Distance between centers of left and right wheels on robot
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24.0);
  // Distance between front and back wheels on robot
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.0);
  public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(WHEEL_BASE_METERS, 2) + Math.pow(TRACK_WIDTH_METERS, 2));
  public static final double MAX_SPEED_METERS_PER_SECOND = 4.65;
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_SPEED_METERS_PER_SECOND / DRIVE_BASE_RADIUS;
  public static final Translation2d[] MODULE_TRANSLATIONS = {
          new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)
  };

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

  // MAXSwerve with 13 pinion teeth and 22 spur teeth
  public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22.0) / (13.0 * 15.0);
  public static final double DRIVE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / DRIVE_MOTOR_REDUCTION;
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((2 * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0;
  public static final PIDFGains DRIVE_PIDF = new PIDFGains(0.0187, 0.0, 0.246, 0.0084);
  public static final double DRIVE_KS = 0.0;
  public static final double DRIVE_KV = 0.0;

  public static final boolean TURN_ENCODER_INVERTED = true;
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI;
  public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;
  public static final PIDFGains TURN_PIDF = new PIDFGains(0.5, 0.0, 0.0, 0.0);

  public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.0, 0.0, 0.0);
  public static final PIDConstants ROTATION_PID = new PIDConstants(0.0, 0.0, 0.0);
  public static final RobotConfig PATH_PLANNER_ROBOT_CONFIG = new RobotConfig(
          45.3592,  // Robot mass (kg)
          6.883,             // Robot MOI (kg m^2)
          new ModuleConfig(
                  Units.inchesToMeters(1.5),   // Wheel radius (m)
                  4.65,                        // Max speed (m/s)
                  1.43,                        // Wheel COF (unitless)
                  DCMotor.getNEO(1).withReduction(DRIVE_MOTOR_REDUCTION),
                  MotorConstants.NEO_CURRENT_LIMIT,
                  1
          ),
          MODULE_TRANSLATIONS
  );
}
