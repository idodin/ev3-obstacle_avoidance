// Lab2.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.09;
	public static final double TRACK = 12.70;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	public static int mapSelection;
	
	
	private static final int[][][] maps = 
		{{{0,2},{1,1},{2,2},{2,1},{1,0}},
				{{1,1},{0,2},{2,2},{2,1},{1,0}},
				{{1,0},{2,1},{2,2},{0,2},{1,1}},
				{{0,1},{1,2},{1,0},{2,1},{2,2}},
				{{0,0},{0,2},{2,2},{2,0},{0,0}}};
	
	
	@SuppressWarnings("resource")
	public static SampleProvider usDistance = new EV3UltrasonicSensor(usPort).getMode("Distance");
	public static float[] usData = new float[usDistance.sampleSize()];
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete
																							// implementation
		Display odometryDisplay = new Display(lcd); // No need to change

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Float | Drive  ", 0, 2);
			lcd.drawString("motors | in a   ", 0, 3);
			lcd.drawString("       | square ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			// Float the motors
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();

			// Display changes in position as wheels are (manually) moved

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

		} else {
			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("< Left |        ", 0, 0);
			lcd.drawString("  No   |        ", 0, 1);
			lcd.drawString(" corr- |        ", 0, 2);
			lcd.drawString(" ection|        ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			// spawn a new Thread to avoid SquareDriver.drive() from blocking
			(new Thread() {
				public void run() {
					mapSelection = 0;
					int[][] positions = maps[mapSelection];
					for (int[] position : positions) {
						Navigator.travelTo(position[0], position[1]);
					}
				}
			}).start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
