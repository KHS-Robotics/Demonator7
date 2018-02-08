package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.logging.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Class to control the RGB Lights
 */
public class AwesomeLights {
    private static boolean started;

    /**
     * Gets the instance of this class
     */
    public static void start() {
        if(started)
            return;

        Logger.info("Starting the awesome lights...");
        new LightController().start();
    }

    /**
     * Class to control the RGB Lights on a separate thread
     */
    private static class LightController extends Thread implements Runnable {
        private final Solenoid power;
        private final Solenoid red;
        private final Solenoid green;
        private final Solenoid blue;
        
        private boolean[] on = { true, false, false };
        private double interval = 500, lastUpated = 0;

        /**
         * Class to control the RGB Lights on a separate thread
         */
        private LightController() {
            power = new Solenoid(RobotMap.LED_POWER);
            red = new Solenoid(RobotMap.RED_LED);
            green = new Solenoid(RobotMap.GREEN_LED);
            blue = new Solenoid(RobotMap.BLUE_LED);
            power.set(true);
        }

        /**
         * Runs if robot is enabled
         */
        @Override
        public void run() {
            if(RobotState.isEnabled()) {
                long current = System.currentTimeMillis();
                if(current - lastUpated >= interval)
                    cycleLights();
            }
        }

        /**
         * Shifts and sets the lights by calling
         * {@link #shiftLights()} then {@link #setLights()}
         */
        private void cycleLights() {
            shiftLights();
            setLights();
        }

        /**
         * Sets the lights based on the state of the internal array
         */
        private void setLights() {
            red.set(on[0]);
            green.set(on[1]);
            blue.set(on[2]);
        }

        /**
         * Shifts the internal array
         */
        private void shiftLights() {
            boolean temp = on[on.length-1];
            for(int i = on.length-2; i >= 0; i++) {
                on[i+1] = on[i];
            }
            on[0] = temp;
        }
    }
}
