// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class SUB_Led extends SubsystemBase {
	private final AddressableLED ledStrip;
	private final AddressableLEDBuffer ledBuffer;
	private SuperstructureState.State localState;
	private Pair<Boolean, LEDPattern> climbMode;

	// Define LED patterns
	private final LEDPattern rainbowPattern;
	private final LEDPattern tealFlamePattern;
	private final LEDPattern intakePattern;
	private final LEDPattern defaultPattern;
	private final LEDPattern strobePattern;

	public final LEDPattern PUB_climbWaiting;
	public final LEDPattern PUB_climbReady;
	public final LEDPattern PUB_climbGo;

	public SUB_Led(int port, int length) {
		this.localState = SuperstructureState.IDLE;

		ledStrip = new AddressableLED(port);
		ledBuffer = new AddressableLEDBuffer(length);
		ledStrip.setLength(ledBuffer.getLength());

		// Initialize patterns
		rainbowPattern = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(25));

		// Teal flame effect
		tealFlamePattern =
				LEDPattern.gradient(
								LEDPattern.GradientType.kContinuous,
								new Color(0, 255, 255), // Bright teal
								new Color(0, 128, 128) // Darker teal
								)
						.breathe(Seconds.of(0.5));

		// Intake pattern effect
		intakePattern =
				LEDPattern.gradient(
								LEDPattern.GradientType.kContinuous,
								new Color(150, 255, 255),
								new Color(150, 128, 128))
						.breathe(Seconds.of(0.5));

		// Strobe effect for CORAL_STATION
		strobePattern = LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.1));

		PUB_climbWaiting = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.05));
		PUB_climbReady = LEDPattern.solid(Color.kGreen);
		PUB_climbGo = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.1));

		defaultPattern = rainbowPattern;
		climbMode = Pair.of(false, defaultPattern);

		setDefaultCommand(runPattern(defaultPattern).withName("Default"));
		ledStrip.start();
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) {
			rainbowPattern.applyTo(ledBuffer);
		} else {
			if (climbMode.getFirst()) {
				LEDPattern pattern = climbMode.getSecond();
				pattern.applyTo(ledBuffer);
			} else {
				updateBasedOnState();
			}
		}
		ledStrip.setData(ledBuffer);
	}

	private void updateBasedOnState() {
		String stateName = localState.getName();
		LEDPattern pattern = defaultPattern;

		// Climbing states - Green pulse indicating upward movement
		if (stateName.startsWith("CLIMB")) {
			pattern = LEDPattern.solid(Color.kGreen).breathe(Seconds.of(1.5));
		}

		// CORAL station - Pattern to indicate ready for intake
		if (stateName.equals("CORAL_STATION")) {
			pattern = intakePattern;
		}

		// Different patterns for each scoring height
		if (stateName.equals("L1_SCORING")) {
			pattern =
					LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlueViolet, Color.kBlue)
							.breathe(Seconds.of(0.75));
		}

		if (stateName.equals("L2_SCORING")) {
			pattern =
					LEDPattern.gradient(
									LEDPattern.GradientType.kContinuous, Color.kYellowGreen, Color.kYellow)
							.breathe(Seconds.of(0.75));
		}

		if (stateName.equals("L3_SCORING")) {
			pattern =
					LEDPattern.gradient(
									LEDPattern.GradientType.kContinuous, Color.kLawnGreen, Color.kGreenYellow)
							.breathe(Seconds.of(0.75));
		}

		if (stateName.equals("L4_SCORING")) {
			pattern =
					LEDPattern.gradient(
									LEDPattern.GradientType.kContinuous, Color.kOrangeRed, Color.kDarkOrange)
							.breathe(Seconds.of(0.75));
		}

		// Algae states - Teal flame effect
		if (stateName.startsWith("ALGAE")) {
			pattern = tealFlamePattern;
		}

		// IDLE - Rainbow pattern
		if (stateName.equals("IDLE")) {
			pattern = rainbowPattern;
		}

		pattern.applyTo(ledBuffer);
	}

	private Color getAllianceColor() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent()
				? (alliance.get() == Alliance.Red ? Color.kRed : Color.kBlue)
				: Color.kWhite;
	}

	public void updateLocalState(SuperstructureState.State newState) {
		localState = newState;
	}

	public Command runPattern(LEDPattern pattern) {
		return run(() -> pattern.applyTo(ledBuffer));
	}

	public Command setClimbState(Pair<Boolean, LEDPattern> climbMode) {
		return run(() -> this.climbMode = climbMode);
	}
}
