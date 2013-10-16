package de.tum.stud.phoenix;
public class MaxIterationsException extends Exception {
		private static final long serialVersionUID = -463767697875252942L;
		public MaxIterationsException() { super(); }
		  public MaxIterationsException(String message) { super(message); }
		  public MaxIterationsException(String message, Throwable cause) { super(message, cause); }
		  public MaxIterationsException(Throwable cause) { super(cause); }
}