package org.usfirst.frc.team4342.robot.logging;

/**
 * Severity for the Logger
 * 
 * @see org.usfirst.frc.team4342.robot.logging.Logger
 */
public enum Severity 
{
	ERROR(3), WARNING(4), INFO(6), DEBUG(7);
	
	private final int severity;
	
	/**
	 * Creates a new severity with the specified level 
	 * following syslog standard
	 * @param severity the severity from 0 to 7 (syslog standard)
	 */
	private Severity(int severity) 
	{
		this.severity = severity;
	}

	/**
	 * Gets the int value of the severity
	 * @return the int value of the severity following syslog standards
	 */
	public int level() 
	{
		return severity;
	}
}
