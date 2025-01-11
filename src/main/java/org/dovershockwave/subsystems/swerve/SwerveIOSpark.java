package org.dovershockwave.subsystems.swerve;

import org.dovershockwave.subsystems.swerve.module.Module;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSpark;
import org.dovershockwave.subsystems.swerve.module.ModuleType;

public class SwerveIOSpark implements SwerveIO {
  @Override public Module[] getModules() {
    return new Module[] {
      new Module(new ModuleIOSpark(ModuleType.FRONT_LEFT), ModuleType.FRONT_LEFT),
      new Module(new ModuleIOSpark(ModuleType.FRONT_RIGHT), ModuleType.FRONT_RIGHT),
      new Module(new ModuleIOSpark(ModuleType.BACK_LEFT), ModuleType.BACK_LEFT),
      new Module(new ModuleIOSpark(ModuleType.BACK_RIGHT), ModuleType.BACK_RIGHT)
    };
  }
}
