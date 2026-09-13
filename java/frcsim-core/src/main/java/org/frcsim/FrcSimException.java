package org.frcsim;

/** Thrown when the native simulation library fails to load or reports an internal error. */
public class FrcSimException extends RuntimeException {
  private static final long serialVersionUID = 1L;

  /**
   * Creates an exception with a message.
   *
   * @param message description of the failure
   */
  public FrcSimException(String message) {
    super(message);
  }

  /**
   * Creates an exception with a message and cause.
   *
   * @param message description of the failure
   * @param cause underlying error
   */
  public FrcSimException(String message, Throwable cause) {
    super(message, cause);
  }
}
