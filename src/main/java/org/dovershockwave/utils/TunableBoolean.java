package org.dovershockwave.utils;

import org.dovershockwave.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableBoolean {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private boolean defaultValue;
  private LoggedNetworkBoolean dashboardNumber;
  private final Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableBoolean(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableBoolean(String dashboardKey, boolean defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(boolean defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.TUNING_MODE) {
        dashboardNumber = new LoggedNetworkBoolean(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public boolean get() {
    if (!hasDefault) {
      return false;
    } else {
      return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    boolean currentValue = get();
    boolean lastValue = lastHasChangedValues.get(id);
    if (currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */

  public static void ifChanged(int id, Consumer<Boolean[]> action, TunableBoolean... tunableBooleans) {
    if (Arrays.stream(tunableBooleans).anyMatch(tunableBoolean -> tunableBoolean.hasChanged(id))) {
      Boolean[] results = Arrays.stream(tunableBooleans).filter(tunableBoolean -> tunableBoolean.hasChanged(id)).map(TunableBoolean::get).toArray(Boolean[]::new);
      action.accept(results);
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, TunableBoolean... tunableBooleans) {
    ifChanged(id, values -> action.run(), tunableBooleans);
  }
}