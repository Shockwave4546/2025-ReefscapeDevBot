package org.dovershockwave.utils;

import org.dovershockwave.Constants;
import org.dovershockwave.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class TunableNumber {
  private final double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public TunableNumber(String key, double defaultValue) {
    this.defaultValue = defaultValue;
    if (Constants.TUNING_MODE && !RobotContainer.isCompetitionMatch()) {
      this.dashboardNumber = new LoggedNetworkNumber(Constants.TUNING_TABLE_NAME + "/" + key, defaultValue);
    }
  }

  public double get() {
    return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
  }

  public void set(double value) {
    dashboardNumber.set(value);
  }

  public boolean hasChanged(int id) {
    final double currentValue = get();
    final double lastValue = lastHasChangedValues.get(id);
    if (currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  public static void ifChanged(int id, Consumer<double[]> action, TunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableNumber::get).toArray());
    }
  }

  public static void ifChanged(int id, Runnable action, TunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }
}