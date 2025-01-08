package frc.robot.utils;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class SparkUtils {
  public static void useValueIfOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer, Runnable doIfNotOk) {
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(supplier.getAsDouble());
    } else {
      doIfNotOk.run();
    }
  }

  public static void useValuesIfOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer, Runnable doIfNotOk) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        doIfNotOk.run();
        return;
      }
    }
    consumer.accept(values);
  }

  public static void tryUntilOk(SparkBase spark, int maxAttempts, Consumer<SparkBase> command, Runnable doIfNotOk) {
    for (int i = 0; i < maxAttempts; i++) {
      command.accept(spark);
      if (spark.getLastError() == REVLibError.kOk) {
        return;
      }
    }
    doIfNotOk.run();
  }
}