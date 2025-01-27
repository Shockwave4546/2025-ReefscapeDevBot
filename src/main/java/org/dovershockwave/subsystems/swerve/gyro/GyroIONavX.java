package org.dovershockwave.subsystems.swerve.gyro;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.dovershockwave.subsystems.swerve.SparkOdometryThread;
import org.dovershockwave.subsystems.swerve.SwerveConstants;

import java.util.Queue;

public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(AHRS.NavXComType.kUSB1, (byte) SwerveConstants.ODOMETRY_FREQUENCY);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
  }

  @Override public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
    inputs.yawAccelerationRadPerSecSquared = Units.degreesToRadians(-navX.getRawAccelZ());
    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream().map((Double value) -> Rotation2d.fromDegrees(-value)).toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
