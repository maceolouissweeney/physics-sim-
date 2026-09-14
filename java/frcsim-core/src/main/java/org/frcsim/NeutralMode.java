package org.frcsim;

/** Motor behavior at zero commanded voltage. Codes match {@code frcsim_neutral_mode}. */
public enum NeutralMode {
  /** Windings shorted: back-EMF braking. */
  BRAKE,
  /** Windings open: no current, the mechanism spins freely. */
  COAST
}
