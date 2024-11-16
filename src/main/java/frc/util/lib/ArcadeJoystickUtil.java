// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * WARNING: PLEASE USE UTF-8 and not WINDOWS-1252 character encoding
 * The element symbol (weird e character) is UTF-8 only
 * Thank you future reader :)
 * 
 * This only occured due to the 2024 update
 * 
 * FIX:
 * ADD THIS TO YOUR BUILD.GRADLE (WITHOUT ASTERISKS)
 * compileJava.options.encoding = 'UTF-8' 
 * compileTestJava.options.encoding = 'UTF-8'
 * javadoc.options.encoding = 'UTF-8'
 * 
 * Your error will look like this if you use Window's encoding:
 * ArcadeJoystickUtil.java:47: error: unmappable character (0x9D) for encoding windows-1252
 */

/** A class providing a helper method for converting x-y control (x ∈ [-1, 1], y ∈ [-1, 1]) to scaled polar coordinates (arcade drive/stick). See convertXYToScaledPolar for more details */
public class ArcadeJoystickUtil {

    /**
     * The empty default constructor of this utility class.
     */
    public ArcadeJoystickUtil() {}
    
    double[] output = new double[2];

    // Regular signed angle
    double controlsAngle = 0.0;

    // Angle limited to [0, PI/2]
    double limitedAngle = 0.0;

    // Hypotenuse from controls
    double controlsHypot = 0.0;

    // Unit hypotenuse to scale by
    double unitHypot = 0.0;

    public double[] regularGamePadControls(double xVal, double yVal, double maxMagnitude) {
        controlsAngle = Math.atan2(yVal, xVal);
        controlsHypot = Math.hypot(xVal, yVal);

        SmartDashboard.putNumber("xVal", controlsAngle);
        SmartDashboard.putNumber("yVal", controlsHypot);
  
        double[] output = new double[2];

        output[0] = Math.abs(maxMagnitude) * controlsHypot;
        output[1] = controlsAngle;
        return output;
    }

    /**
     * A function that converts from an input of a joystick (x from -1.0 to 1.0 and y from -1.0 to 1.0), to a scaled polar/radial representation of the joystick. This literally represents the expectation of an arcade stick (polar coordinates). For example, if you physically held the joystick at half the distance radially, this should result in half the magnitude.
     * @param xVal Value of x such that x ∈ [-1, 1]
     * @param yVal Value of y such that y ∈ [-1, 1]
     * @param maxMagnitude Any magnitude of any value
     * @return double[] of radial component and angular component, such that r ∈ [0, <i>magnitude</i>] and 𝜃 ∈ [0, 2𝜋].
     */
    public double[] convertXYToScaledPolar(double xVal, double yVal, double maxMagnitude) {
        // Define control hypothenuse and control signed angle
        controlsAngle = Math.atan2(yVal, xVal);
        controlsHypot = Math.hypot(xVal, yVal);
        // Define limited angle from [0, PI/2]
        limitedAngle = Math.atan2(Math.abs(yVal), Math.abs(xVal));

        // If this is greater, then subtract
        if (limitedAngle > Math.PI/4) {
            // Takes unit hypotenuse reverse of tan function's domain. Shift [PI/4, PI/2] to -> [PI/4, 0]
            unitHypot = Math.hypot(1, Math.tan(Math.PI/2 - limitedAngle));
        } else {
            // Takes unit hypotenuse in line with tan function's domain [0, PI/4]
            unitHypot = Math.hypot(1, Math.tan(limitedAngle));
        }

        // Scales current control (i.e. 1.41) to unit hypotenuse (i.e. 1.41)
        double radialOutput = controlsHypot/unitHypot;
  
        // Get new hypotenuse from limited x and y 
        output[0] = Math.abs(maxMagnitude) * radialOutput;
        output[1] = controlsAngle;
        
        SmartDashboard.putNumber("Radial Output of Joystick:", radialOutput);
        SmartDashboard.putNumber("Angular Output of Joystick:", output[1]);
        // SmartDashboard.putNumber("Radial Output in Magnitude:", output[0]);
        // SmartDashboard.putNumber("Controls Hypot", controlsHypot);
        // SmartDashboard.putNumber("Unit Hypot:", unitHypot);

        return output;

    }
}
