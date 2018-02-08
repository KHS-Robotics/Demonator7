package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.logging.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Class to control the RGB Lights
 */
public class AwesomeLights {
    private static Solenoid power, red, green, blue;

    private static boolean started, running;
    private static LightController controller;

    /**
     * Starts the awesome lights
     */
    public static void start() {
        if(running)
            return;
        running = true;

        if(!started) {
            started = true;
            Logger.info("Creating the awesome lights...");
            power = new Solenoid(RobotMap.LED_POWER);
            red = new Solenoid(RobotMap.RED_LED);
            green = new Solenoid(RobotMap.GREEN_LED);
            blue = new Solenoid(RobotMap.BLUE_LED);
        }

        controller = new LightController();
        controller.start();
    }

    /**
     * Stops the awesome lights
     */
    public void stop() {
        running = false;
        controller.interrupt();
    }

    /**
     * Class to control the RGB Lights on a separate thread
     */
    private static class LightController extends Thread implements Runnable {
        private boolean[] on = { true, false, false };
        private double interval = 500, lastUpated = 0;

        /**
         * Runs if robot is enabled
         */
        @Override
        public void run() {
            power.set(true);

            while(!Thread.interrupted()) {
                if(RobotState.isEnabled()) {
                    long current = System.currentTimeMillis();
                    if(current - lastUpated >= interval)
                        cycleLights();
                }
            }

            red.set(false);
            green.set(false);
            blue.set(false);
            power.set(false);
        }

        /**
         * Shifts and sets the lights by calling
         * {@link #shiftLights()} then {@link #setLights()}
         */
        private void cycleLights() {
            shiftLights();
            setLights();
            lastUpated = System.currentTimeMillis();
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
