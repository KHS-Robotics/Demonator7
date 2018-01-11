package org.usfirst.frc.team4342.robot.logging;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;

/**
 * Logs PDP Data to a CSV file on the RoboRIO
 */
public class PDPLogger 
{
	private PDPLogger() {}
	
	private static final String CSV_PATH = "/home/lvuser/pdp.csv";
	private static final int NUM_CHANNELS = 16;
	private static final int LOGGING_SECONDS = 300;
	
	private static Timer timer;
	private static PowerDistributionPanel pdp;
	private static boolean started;
	
	/**
	 * Starts logging PDP data
	 */
	public static void start()
	{
		if(started)
			return;
		started = true;
		
		Logger.info("Starting PDP Logger... logging to \"" + CSV_PATH + "\"");
		
		timer = new Timer();
		pdp = new PowerDistributionPanel();
		
		new PDPLoggingThread().start();
		timer.start();
	}
	
	/**
	 * Frees the resources used by this class
	 */
	private static void free()
	{
		timer.stop();
		timer = null;
		
		pdp.free();
		pdp = null;
		
		started = false;
	}

	/**
	 * The magic behind this class...
	 */
	private static class PDPLoggingThread extends Thread implements Runnable
	{	
		private static final String RETURN_FEED = System.lineSeparator();
		private static final int SLEEP_SECONDS = 3;
		private static boolean errored;
		
		/**
		 * Logs PDP Data to a CSV File
		 */
		@Override
		public void run()
		{
			FileWriter writer = null;
			
			try
			{
				File csvFile = new File(CSV_PATH);
				if(csvFile.exists())
					csvFile.delete();
				csvFile.createNewFile();
				
				writer = new FileWriter(csvFile);
				
				for(int channel = 0; channel < NUM_CHANNELS; channel++) 
				{
		        	writer.write("PDP-A" + channel);
		        	writer.write(',');
		        }
				
				writer.write("PDP-V");
				writer.write(',');
				
				writer.write("Timestamp");
				
				writer.write(RETURN_FEED);
	
				while((int)timer.get() < LOGGING_SECONDS && !errored) 
				{
			        for(int channel = 0; channel < NUM_CHANNELS; channel++) 
			        {
			        	writer.write("" + pdp.getCurrent(channel));
			        	writer.write(',');
			        }
			        
			        writer.write("" + pdp.getVoltage());
			        writer.write(',');
			        
			        writer.write(new Date(System.currentTimeMillis()).toString());
			        
			        writer.write(RETURN_FEED);
			        writer.flush();
			        
			        Thread.sleep(SLEEP_SECONDS * 1000);
				}
			}
			catch(Exception ex)
			{
				Logger.error("Unexpected error while trying to log PDP data", ex);
				errored = true;
			}
			finally
			{
				if(writer != null)
				{
					try 
					{
						writer.close();
					}  
					catch (IOException ex) 
					{
						Logger.error("Unexpected error while trying close stream for PDPLogger", ex);
					}
				}
				
				PDPLogger.free();
			}
		}
	}
}
