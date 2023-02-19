package frc.Autons;

import edu.wpi.first.wpilibj.Timer;

@SuppressWarnings("unused")
public abstract class ThreadRunner implements Runnable, AutoCloseable {
    private final int period;
    private ThreadSignal signal = ThreadSignal.PAUSED;
    public final String subsystemName;
    private Thread thisThread;

    public enum ThreadSignal {
        ALIVE, PAUSED, DEAD
    }

    /**
     * @param period The period when calling update
     */
    public ThreadRunner(int period) {
        this.period = period;
        this.subsystemName = this.getClass().getSimpleName();
    }

    public abstract void selfTest();

    public void pause() {
        signal = ThreadSignal.PAUSED;
    }

    public void kill() {
        signal = ThreadSignal.DEAD;
    }

    public void start() {
        signal = ThreadSignal.ALIVE;
        if ((thisThread == null || !thisThread.isAlive()) && this.period > 0) {
            thisThread = new Thread(this);
            thisThread.start();
        }
    }

    /**
     * This function will be called repeatedly when the thread is alive. The period will be whatever you defined when creating the
     * object
     */
    public void update() {

    }

    int lastLength = 20;

    @Override
    @SuppressWarnings("BusyWait")
    public void run() {
        while (signal != ThreadSignal.DEAD) {
            double startTime = Timer.getFPGATimestamp();
            if (signal == ThreadSignal.ALIVE) {
                update();
            }
            double executionTimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
            try {
                if (period - executionTimeMS > 0) {
                    Thread.sleep((long) (period - executionTimeMS));
                }
            } catch (InterruptedException e) {
                System.out.println("Thread interrupted " + subsystemName + " message: " + e.getMessage());
                return;
            }
        }
    }
}
