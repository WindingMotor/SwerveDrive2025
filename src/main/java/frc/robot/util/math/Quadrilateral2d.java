// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Quadrilateral2d {
	private Translation2d point1; // Top-left
	private Translation2d point2; // Top-right
	private Translation2d point3; // Bottom-right
	private Translation2d point4; // Bottom-left

	public Quadrilateral2d(
			Translation2d point1, Translation2d point2, Translation2d point3, Translation2d point4) {
		this.point1 = point1;
		this.point2 = point2;
		this.point3 = point3;
		this.point4 = point4;
	}

	// Alternative constructor using center point, width, height, and rotation
	public Quadrilateral2d(Translation2d center, double width, double height, Rotation2d rotation) {
		double halfWidth = width / 2;
		double halfHeight = height / 2;
		double cos = Math.cos(rotation.getRadians());
		double sin = Math.sin(rotation.getRadians());

		// Calculate corner points from center
		this.point1 =
				new Translation2d(
						center.getX() - halfWidth * cos + halfHeight * sin,
						center.getY() - halfWidth * sin - halfHeight * cos);
		this.point2 =
				new Translation2d(
						center.getX() + halfWidth * cos + halfHeight * sin,
						center.getY() + halfWidth * sin - halfHeight * cos);
		this.point3 =
				new Translation2d(
						center.getX() + halfWidth * cos - halfHeight * sin,
						center.getY() + halfWidth * sin + halfHeight * cos);
		this.point4 =
				new Translation2d(
						center.getX() - halfWidth * cos - halfHeight * sin,
						center.getY() - halfWidth * sin + halfHeight * cos);
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

	public Translation2d getPoint4() {
		return point4;
	}

	public Translation2d getCenter() {
		double centerX = (point1.getX() + point2.getX() + point3.getX() + point4.getX()) / 4;
		double centerY = (point1.getY() + point2.getY() + point3.getY() + point4.getY()) / 4;
		return new Translation2d(centerX, centerY);
	}

	public double getArea() {
		// Using the shoelace formula for area of a polygon
		return Math.abs(
						point1.getX() * point2.getY()
								+ point2.getX() * point3.getY()
								+ point3.getX() * point4.getY()
								+ point4.getX() * point1.getY()
								- point2.getX() * point1.getY()
								- point3.getX() * point2.getY()
								- point4.getX() * point3.getY()
								- point1.getX() * point4.getY())
				/ 2;
	}

	public double getPerimeter() {
		return point1.getDistance(point2)
				+ point2.getDistance(point3)
				+ point3.getDistance(point4)
				+ point4.getDistance(point1);
	}

	public double getDistanceToCenter(Translation2d from) {
		return from.getDistance(getCenter());
	}

	public boolean isWithinArea(Translation2d point) {
		// Using the winding number algorithm for point-in-polygon test
		int wn = 0; // Winding number
		Translation2d[] points = {point1, point2, point3, point4, point1}; // Close the polygon

		for (int i = 0; i < points.length - 1; i++) {
			if (points[i].getY() <= point.getY()) {
				if (points[i + 1].getY() > point.getY()) {
					if (isLeft(points[i], points[i + 1], point) > 0) {
						wn++;
					}
				}
			} else {
				if (points[i + 1].getY() <= point.getY()) {
					if (isLeft(points[i], points[i + 1], point) < 0) {
						wn--;
					}
				}
			}
		}
		return wn != 0;
	}

	private double isLeft(Translation2d p0, Translation2d p1, Translation2d point) {
		return ((p1.getX() - p0.getX()) * (point.getY() - p0.getY())
				- (point.getX() - p0.getX()) * (p1.getY() - p0.getY()));
	}

	public double getDistanceFromEdge(Translation2d point) {
		if (isWithinArea(point)) {
			return 0;
		}

		// Calculate minimum distance to any edge
		double d1 = distanceToLine(point, point1, point2);
		double d2 = distanceToLine(point, point2, point3);
		double d3 = distanceToLine(point, point3, point4);
		double d4 = distanceToLine(point, point4, point1);

		return Math.min(Math.min(d1, d2), Math.min(d3, d4));
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
		// Return the four corner points plus the first point again to close the shape
		Translation2d[] edgePoints = new Translation2d[5];
		edgePoints[0] = point1;
		edgePoints[1] = point2;
		edgePoints[2] = point3;
		edgePoints[3] = point4;
		edgePoints[4] = point1; // Close the shape by repeating the first point
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
