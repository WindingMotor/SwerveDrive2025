// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

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
}
