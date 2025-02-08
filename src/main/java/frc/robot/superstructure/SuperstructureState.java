// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.superstructure;

import java.util.HashMap;
import java.util.Map;

public class SuperstructureState {
	private static final Map<String, State> dynamicStates = new HashMap<>();

	public static class State {
		private final String name;
		private final double heightM;
		private final int deg;
		private final double speed;

		private State(String name, double heightM, int deg, double speed) {
			this.name = name;
			this.heightM = heightM;
			this.deg = deg;
			this.speed = speed;
		}

		public double getHeightM() {
			return heightM;
		}

		public int getDeg() {
			return deg;
		}

		public double getSpeed() {
			return speed;
		}

		public String getName() {
			return name;
		}

		@Override
		public String toString() {
			return name;
		}
	}

	// Pre-defined states
	// Height (M), Angle (Deg), Speed (RPM)
	public static final State IDLE = createState("IDLE", 0.05, 18, 0.0);
	public static final State CLIMB = createState("CLIMB", 0.8, 0, 0.0);

	// Coral states
	public static final State CORAL_STATION = createState("CORAL_STATION", 0.38, 32, 0.3);
	public static final State L1_SCORING = createState("L1_SCORING", 0.5, 100, 0.0);
	public static final State L2_SCORING = createState("L2_SCORING", 0.8, 120, 0.0);
	public static final State L3_SCORING = createState("L3_SCORING", 1.4, 120, 0.0);
	public static final State L4_SCORING = createState("L4_SCORING", 2.415, 128, 0.0);
	// public static final State L4_SCORING = createState("L4_SCORING", 2.415, 121, 0.0);

	// Algae states
	public static final State ALGAE_GROUND = createState("ALGAE_GROUND", 0.1, 135, 0.95);
	public static final State ALGAE_PROCESSOR = createState("ALGAE_PROCESSOR", 0.1, 90, 0.1);
	public static final State ALGAE_BARGE = createState("ALGAE_BARGE", 0.1, 45, 0.1);
	public static final State ALGAE_L2 = createState("ALGAE_L2", 0.7, 90, 0.95);
	public static final State ALGAE_L3 = createState("ALGAE_L3", 1.85, 135, 0.95);

	public static State createState(String name, double heightM, int deg, double speed) {
		State newState = new State(name, heightM, deg, speed);
		dynamicStates.put(name, newState);
		return newState;
	}

	public static State getState(String name) {
		return dynamicStates.get(name);
	}

	public static Map<String, State> getAllStates() {
		return new HashMap<>(dynamicStates);
	}
}
