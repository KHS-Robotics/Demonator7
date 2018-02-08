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
        private final Solenoid LedPower;
        private final Solenoid RedLight;
        private final Solenoid GreenLight;
        private final Solenoid BlueLight;
        
        private boolean[] on = { true, false, false };
        private double interval = 500, lastUpated = 0;

        /**
         * Class to control the RGB Lights on a separate thread
         */
        private LightController() {
            LedPower = new Solenoid(RobotMap.LED_POWER);
            RedLight = new Solenoid(RobotMap.RED_LED);
            GreenLight = new Solenoid(RobotMap.GREEN_LED);
            BlueLight = new Solenoid(RobotMap.BLUE_LED);
            LedPower.set(true);
        }

        /**
         * Runs if robot is enabled
         */
        @Override
        public void run() {
            if(RobotState.isEnabled()) {
                long current = System.currentTimeMillis();
                if(current - lastUpated >= interval) {
                    shift();
                    setLights();
                    lastUpated = System.currentTimeMillis();
                }
            }
        }

        /**
         * Sets the lights based on the state of the internal array
         */
        private void setLights() {
            RedLight.set(on[0]);
            GreenLight.set(on[1]);
            BlueLight.set(on[2]);
        }

        /**
         * Shifts the internal array
         */
        private void shift() {
            boolean temp = on[on.length-1];
            for(int i = on.length-2; i >= 0; i++) {
                on[i+1] = on[i];
            }
            on[0] = temp;
        }
    }
}
