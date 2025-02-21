// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Circle2d {

	private double x;
	private double y;
	private double radius;

	public Circle2d(double x, double y, double radius) {
		this.x = x;
		this.y = y;
		this.radius = radius;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public Translation2d getTranslation2d() {
		return new Translation2d(x, y);
	}

	public double getRadius() {
		return radius;
	}

	public double getArea() {
		return Math.PI * radius * radius;
	}

	public double getCircumference() {
		return 2 * Math.PI * radius;
	}

	public double getDistanceToCenter(Translation2d from) {
		return from.getDistance(new Translation2d(x, y));
	}

	public boolean isWithinArea(Translation2d from) {
		return getDistanceToCenter(from) <= radius;
	}

	public double getDistanceFromEdge(Translation2d from) {
		return Math.max(getDistanceToCenter(from) - radius, 0);
	}

	public Translation2d[] getEstimatedEdgePoints(double vertices) {
		if (vertices < 3) {
			throw new IllegalArgumentException("Number of vertices must be at least 3");
		}

		Translation2d[] edgePoints = new Translation2d[(int) vertices];

		for (int i = 0; i < vertices; i++) {
			// Calculate angle for each vertex
			double angle = 2 * Math.PI * i / vertices;

			// Calculate x and y coordinates on the circle's edge
			double edgeX = x + radius * Math.cos(angle);
			double edgeY = y + radius * Math.sin(angle);

			edgePoints[i] = new Translation2d(edgeX, edgeY);
		}

		return edgePoints;
	}

	public Pose2d[] getEstimatedEdgePoses(double vertices) {
		if (vertices < 3) {
			throw new IllegalArgumentException("Number of vertices must be at least 3");
		}

		Translation2d[] edgePoints = getEstimatedEdgePoints(vertices);
		Pose2d[] edgePoses = new Pose2d[edgePoints.length];

		for (int i = 0; i < edgePoints.length; i++) {
			edgePoses[i] = new Pose2d(edgePoints[i], Rotation2d.fromDegrees(0));
		}

		return edgePoses;
	}
}
