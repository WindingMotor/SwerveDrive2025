// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Triangle2d {
	private Translation2d point1;
	private Translation2d point2;
	private Translation2d point3;

	public Triangle2d(Translation2d point1, Translation2d point2, Translation2d point3) {
		this.point1 = point1;
		this.point2 = point2;
		this.point3 = point3;
	}

	public Translation2d getPoint1() {
		return point1;
	}

	public Translation2d getPoint2() {
		return point2;
	}

	public Translation2d getPoint3() {
		return point3;
	}

	public Translation2d getCentroid() {
		double centerX = (point1.getX() + point2.getX() + point3.getX()) / 3;
		double centerY = (point1.getY() + point2.getY() + point3.getY()) / 3;
		return new Translation2d(centerX, centerY);
	}

	public double getArea() {
		// Using the formula: Area = |x1(y2 - y3) + x2(y3 - y1) + x3(y1 - y2)| / 2
		return Math.abs(
						point1.getX() * (point2.getY() - point3.getY())
								+ point2.getX() * (point3.getY() - point1.getY())
								+ point3.getX() * (point1.getY() - point2.getY()))
				/ 2;
	}

	public double getPerimeter() {
		return point1.getDistance(point2) + point2.getDistance(point3) + point3.getDistance(point1);
	}

	public double getDistanceToCenter(Translation2d from) {
		return from.getDistance(getCentroid());
	}

	public boolean isWithinArea(Translation2d point) {
		// Calculate areas of three triangles formed by the point and two vertices
		Triangle2d t1 = new Triangle2d(point, point2, point3);
		Triangle2d t2 = new Triangle2d(point1, point, point3);
		Triangle2d t3 = new Triangle2d(point1, point2, point);

		// Point is inside if sum of three triangle areas equals the original triangle area
		double totalArea = t1.getArea() + t2.getArea() + t3.getArea();
		return Math.abs(totalArea - getArea())
				< 1e-10; // Using small epsilon for floating-point comparison
	}

	public double getDistanceFromEdge(Translation2d point) {
		if (isWithinArea(point)) {
			return 0;
		}

		// Calculate minimum distance to any edge
		double d1 = distanceToLine(point, point1, point2);
		double d2 = distanceToLine(point, point2, point3);
		double d3 = distanceToLine(point, point3, point1);

		return Math.min(Math.min(d1, d2), d3);
	}

	private double distanceToLine(
			Translation2d point, Translation2d lineStart, Translation2d lineEnd) {
		double numerator =
				Math.abs(
						(lineEnd.getY() - lineStart.getY()) * point.getX()
								- (lineEnd.getX() - lineStart.getX()) * point.getY()
								+ lineEnd.getX() * lineStart.getY()
								- lineEnd.getY() * lineStart.getX());

		double denominator =
				Math.sqrt(
						Math.pow(lineEnd.getY() - lineStart.getY(), 2)
								+ Math.pow(lineEnd.getX() - lineStart.getX(), 2));

		return numerator / denominator;
	}

	public Translation2d[] getEstimatedEdgePoints(double vertices) {
		// Ignore the vertices parameter as we're just returning the corner points
		Translation2d[] edgePoints = new Translation2d[4]; // 3 points plus the first point repeated
		edgePoints[0] = point1;
		edgePoints[1] = point2;
		edgePoints[2] = point3;
		edgePoints[3] = point1; // Close the shape by repeating the first point
		return edgePoints;
	}

	public Pose2d[] getEstimatedEdgePoses(double vertices) {
		Translation2d[] edgePoints = getEstimatedEdgePoints(vertices);
		Pose2d[] edgePoses = new Pose2d[edgePoints.length];
		for (int i = 0; i < edgePoints.length; i++) {
			edgePoses[i] = new Pose2d(edgePoints[i], Rotation2d.fromDegrees(0));
		}
		return edgePoses;
	}
}
