package org.dovershockwave.subsystems.swerve;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.dovershockwave.MotorConstants;

import static org.dovershockwave.subsystems.swerve.SwerveConstants.*;

public class SwerveConfigs {
  public static final SparkBaseConfig DRIVE_CONFIG = new SparkMaxConfig();
  public static final SparkBaseConfig TURN_CONFIG = new SparkMaxConfig();

  static {
    DRIVE_CONFIG
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    DRIVE_CONFIG.encoder
            .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
    DRIVE_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(DRIVE_PIDF.p(), DRIVE_PIDF.i(), DRIVE_PIDF.d(), DRIVE_PIDF.ff());
    DRIVE_CONFIG.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

    TURN_CONFIG
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.NEO_550_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    TURN_CONFIG.absoluteEncoder
            .inverted(TURN_ENCODER_INVERTED)
            .positionConversionFactor(TURN_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(TURN_ENCODER_VELOCITY_FACTOR)
            .averageDepth(2);
    TURN_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .pidf(TURN_PIDF.p(), TURN_PIDF.i(), TURN_PIDF.d(), TURN_PIDF.ff());
    TURN_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
