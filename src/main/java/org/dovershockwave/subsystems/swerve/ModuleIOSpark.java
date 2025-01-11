package org.dovershockwave.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;

import java.util.Queue;

import static org.dovershockwave.utils.SparkUtils.tryUntilOk;
import static org.dovershockwave.utils.SparkUtils.useValueIfOk;

public class ModuleIOSpark implements ModuleIO {
  private final SparkBase driveSpark;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController drivePID;

  private final SparkBase turnSpark;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController turnPID;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer turnConnectedDebouncer = new Debouncer(0.5);

  public ModuleIOSpark(ModuleType type) {
    driveSpark = new SparkMax(type.driveID, SparkLowLevel.MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    drivePID = driveSpark.getClosedLoopController();

    turnSpark = new SparkMax(type.turnID, SparkLowLevel.MotorType.kBrushless);
    turnEncoder = turnSpark.getAbsoluteEncoder();
    turnPID = turnSpark.getClosedLoopController();

    tryUntilOk(driveSpark, 5, spark -> {
      spark.configure(SwerveConfigs.DRIVE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    tryUntilOk(driveSpark, 5, spark -> {
      spark.getEncoder().setPosition(0.0);
    });

    tryUntilOk(turnSpark, 5, spark -> {
      spark.configure(SwerveConfigs.TURN_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override public void updateInputs(ModuleIOInputs inputs) {
    useValueIfOk(driveSpark, driveSpark.getEncoder()::getPosition, (value) -> {
      inputs.drivePositionRad = value;
    }, () -> {
      inputs.driveConnected = false;
    });
  }
}
