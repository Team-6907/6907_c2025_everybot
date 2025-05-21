// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class DualEdgeDelayedBoolean {
  private boolean mLastValue;
  private double mRisingEdgeTimestamp;
  private double mFallingEdgeTimestamp;
  private final double mDelay;
  private final EdgeType mEdgeType;
  private boolean mState;

  public enum EdgeType {
    RISING,
    FALLING
  }

  public DualEdgeDelayedBoolean(double delay, EdgeType edgeType) {
    double timestamp = Timer.getTimestamp();
    mRisingEdgeTimestamp = timestamp;
    mFallingEdgeTimestamp = timestamp;
    mLastValue = false;
    mDelay = delay;
    mEdgeType = edgeType;
    mState = false;
  }

  public boolean get() {
    return mState;
  }

  public boolean update(boolean value) {
    double timestamp = Timer.getTimestamp();
    if (value && !mLastValue) {
      mRisingEdgeTimestamp = timestamp;
    } else if (!value && mLastValue) {
      mFallingEdgeTimestamp = timestamp;
    }

    switch (mEdgeType) {
      case RISING:
        mState = value && (timestamp - mRisingEdgeTimestamp > mDelay);
        break;
      case FALLING:
        mState = value || (timestamp - mFallingEdgeTimestamp < mDelay);
        break;
    }

    mLastValue = value;
    return mState;
  }
}
