// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.math;

/**
 *
 *
 * <h2>Exponential Decay Feedforward Rotation Controller</h2>
 *
 * <h3>Mathematical Model:</h3>
 *
 * <pre>
 * P = P_max * (1 - (1 - |error|/90)^exponent)
 * </pre>
 *
 * <h3>Variables:</h3>
 *
 * <ul>
 *   <li><b>P</b> - output power [-1.0 to 1.0]
 *   <li><b>P_max</b> - maximum static feedforward value
 *   <li><b>error</b> - angular error in degrees [-180° to 180°]
 *   <li><b>exponent</b> - controls decay rate (higher = more aggressive deceleration)
 * </ul>
 *
 * <h3>Key Features:</h3>
 *
 * <ul>
 *   <li>Provides maximum power when far from target
 *   <li>Exponentially decreases power as error approaches zero
 *   <li>Automatically handles angle wrapping and shortest path
 *   <li>Zero output within specified tolerance zone
 *   <li>No integral windup issues common in PID
 *   <li>Works well for rotation targets where bouncing between target value is common
 * </ul>
 *
 * <h3>Error Normalization:</h3>
 *
 * <ul>
 *   <li>Maintains error between -180° and 180°
 *   <li>Uses shortest path calculation: error = (target - current) % 360
 *   <li>Power direction determined by sign of normalized error
 * </ul>
 */
public class ExpDecayFF {
	private final double maxStaticFF;
	private final double ffExponent;
	private final double endingTolerance;

	/**
	 * Creates a new RotationFFController.
	 *
	 * @param maxStaticFF Maximum feedforward value
	 * @param ffExponent Controls how quickly FF drops off (larger = more aggressive)
	 * @param endingTolerance Tolerance in degrees for considering target reached
	 */
	public ExpDecayFF(double maxStaticFF, double ffExponent, double endingTolerance) {
		this.maxStaticFF = maxStaticFF;
		this.ffExponent = ffExponent;
		this.endingTolerance = endingTolerance;
	}

	/** Normalizes an angle to be within -180 to 180 degrees */
	private double normalizeAngle(double angle) {
		angle = angle % 360;
		if (angle > 180) angle -= 360;
		if (angle < -180) angle += 360;
		return angle;
	}

	/** Calculates the shortest angular distance between two angles */
	private double getShortestDistance(double from, double to) {
		double normalizedFrom = normalizeAngle(from);
		double normalizedTo = normalizeAngle(to);
		double error = normalizedTo - normalizedFrom;

		if (error > 180) error -= 360;
		if (error < -180) error += 360;

		return error;
	}

	public double calculate(double currentAngle, double targetAngle) {

		// Calculate shortest path error
		double error = getShortestDistance(currentAngle, targetAngle);

		// If within tolerance, stop rotating
		if (Math.abs(error) < endingTolerance) {
			return 0.0;
		}

		// Scale error to 0-1 range, but only consider up to 90 degrees for max speed
		// This means we'll hit max speed at 90 degrees error instead of 180
		double errorRatio = Math.min(Math.abs(error) / 90.0, 1.0);

		// Invert the ratio so power decreases as we get closer to target
		double scaleFactor = Math.pow(1.0 - errorRatio, ffExponent);

		// The scale factor is now 0 when far away and 1 when close,
		// so we need to invert it when applying it to max FF
		double ffValue = maxStaticFF * (1.0 - scaleFactor);
		// Apply direction
		return Math.copySign(ffValue, error);
	}

	/**
	 * Checks if the current angle is within tolerance of the target angle
	 *
	 * @param currentAngle The current angle in degrees
	 * @return true if within tolerance or in NONE state, false otherwise
	 */
	public boolean atTarget(double currentAngle, double targetAngle) {
		return Math.abs(getShortestDistance(currentAngle, targetAngle)) < endingTolerance;
	}
}
