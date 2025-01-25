package org.dovershockwave.subsystems.swerve;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {
  public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static SparkOdometryThread instance = null;
  private final Notifier notifier = new Notifier(this::run);

  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (!timestampQueues.isEmpty()) {
      notifier.startPeriodic(1.0 / SwerveConstants.ODOMETRY_FREQUENCY);
    }
  }

  /**
   * Registers a Spark signal to be read from the thread.
   */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    ODOMETRY_LOCK.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /**
   * Registers a generic signal to be read from the thread.
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    ODOMETRY_LOCK.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /**
   * Returns a new queue that returns timestamp values for each sample.
   */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    ODOMETRY_LOCK.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  private void run() {
    // Save new data to queues
    ODOMETRY_LOCK.lock();
    try {
      // Get sample timestamp
      double timestamp = RobotController.getFPGATime() / 1e6;

      // Read Spark values, mark invalid in case of error
      double[] sparkValues = new double[sparkSignals.size()];
      boolean isValid = true;
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkValues[i] = sparkSignals.get(i).getAsDouble();
        if (sparks.get(i).getLastError() != REVLibError.kOk) {
          isValid = false;
        }
      }

      // If valid, add values to queues
      if (isValid) {
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkQueues.get(i).offer(sparkValues[i]);
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (Queue<Double> timestampQueue : timestampQueues) {
          timestampQueue.offer(timestamp);
        }
      }
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }
}
