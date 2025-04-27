// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOkV5(int maxAttempts, Supplier<ErrorCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.equals(ErrorCode.OK)) break;
    }
  }
}
