package org.dovershockwave.utils;

import org.dovershockwave.Constants;
import org.dovershockwave.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class TunableBoolean {
  private final boolean defaultValue;
  private LoggedNetworkBoolean dashboardBoolean;
  private final Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  public TunableBoolean(String key, boolean defaultValue) {
    this.defaultValue = defaultValue;
    if (Constants.TUNING_MODE && !RobotContainer.isCompetitionMatch()) {
      this.dashboardBoolean = new LoggedNetworkBoolean(Constants.TUNING_TABLE_NAME + "/" + key, defaultValue);
    }
  }

  public boolean get() {
    return Constants.TUNING_MODE ? dashboardBoolean.get() : defaultValue;
  }

  public void set(boolean value) {
    dashboardBoolean.set(value);
  }

  public boolean hasChanged(int id) {
    final boolean currentValue = get();
    final Boolean lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  public static void ifChanged(int id, Consumer<Boolean[]> action, TunableBoolean... tunableBooleans) {
    if (Arrays.stream(tunableBooleans).anyMatch(tunableBoolean -> tunableBoolean.hasChanged(id))) {
      action.accept(Arrays.stream(tunableBooleans).filter(tunableBoolean -> tunableBoolean.hasChanged(id)).map(TunableBoolean::get).toArray(Boolean[]::new));
    }
  }

  public static void ifChanged(int id, Runnable action, TunableBoolean... tunableBooleans) {
    ifChanged(id, values -> action.run(), tunableBooleans);
  }
}