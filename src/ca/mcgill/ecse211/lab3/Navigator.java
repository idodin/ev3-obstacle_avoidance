package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class contains methods to allow the robot to navigate to specified
 * waypoints.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Navigator {

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	private static final EV3LargeRegulatedMotor leftMotor = Lab3.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Lab3.rightMotor;
	private static Odometer odo;
	private static double[] currentPosition;
	private static boolean isNavigating;
	private static SampleProvider usDistance = Lab3.usDistance;
	private static float[] usData = Lab3.usData;

	/**
	 * This method makes the robot travel to the specified way point
	 * 
	 * @param x x-coordinate of the specified waypoint.
	 * @param y y-coordinate of the specified waypoint.
	 */
	public static void travelTo(double x, double y) {
		isNavigating = true;
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		currentPosition = odo.getXYT();

		double deltaX = x * TILE_SIZE - currentPosition[0];
		double deltaY = y * TILE_SIZE - currentPosition[1];
		double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		System.out.println("MOVING \nCurrent x: " + currentPosition[0] + "\nCurrent y: " + currentPosition[1]
				+ "\nDesired x: " + x + "\nDesired y: " + y + "\ndeltaX: " + deltaX + "\ndeltaY: " + deltaY + "\n");

		if (deltaX == 0 && deltaY != 0) {
			turnTo(deltaY < 0 ? 180 : 0);
		} else {
			double baseAngle = deltaX > 0 ? 90 : 270;
			double adjustAngle;
			if ((deltaY > 0 && deltaX > 0) || (deltaY < 0 && deltaX < 0)) {
				adjustAngle = -1 * Math.toDegrees(Math.atan(deltaY / deltaX));
			} else {
				adjustAngle = Math.toDegrees(Math.atan(Math.abs(deltaY) / Math.abs(deltaX)));
			}

			turnTo(baseAngle + adjustAngle);
			System.out.println("Base Angle: " + baseAngle + "\n Adjust Angle: " + adjustAngle);
		}

		leftMotor.forward();
		rightMotor.forward();

		while (isNavigating) {

			usDistance.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int

			currentPosition = odo.getXYT();

			deltaX = x * TILE_SIZE - currentPosition[0];
			deltaY = y * TILE_SIZE - currentPosition[1];
			distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
			System.out.println("Distance: " + distance);

			if (distance < 2) {
				leftMotor.stop(true);
				rightMotor.stop();
				isNavigating = false;
				return;
			}

		}
	}

	/**
	 * This method makes the robot turn to the specified bearing.
	 * 
	 * @param theta Bearing for the robot to readjust its heading to.
	 */
	public static void turnTo(double theta) {

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		currentPosition = odo.getXYT();

		double deltaT = (((theta - currentPosition[2]) % 360) + 360) % 360;

		System.out.println("TURNING \nCurrent theta: " + currentPosition[2] + "\nDesired theta: " + theta
				+ "\nDelta theta (clockwise): " + deltaT);

		if (deltaT < 180) {
			leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, deltaT), true);
			rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, deltaT), false);
		} else {
			leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 360 - deltaT), true);
			rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 360 - deltaT), false);
		}

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the rotation of each wheel
	 * needed to rotate that angle
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
