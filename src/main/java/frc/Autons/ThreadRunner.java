package frc.Autons;

public abstract class ThreadRunner implements Runnable {
    private final int delay;
    private Thread thread;

    public ThreadRunner(int period) {
        this.delay = period;
    }

    public final void start() {
        if ((thread == null || !thread.isAlive()) && this.delay > 0) {
            thread = new Thread(this);
            thread.start();
        }
    }

    public abstract void update();

    @Override
    public void run() {
        while(true){
            update();
            try{
                Thread.sleep(delay);
            }catch(InterruptedException e){
                System.out.println("Interrupted");
                e.printStackTrace();
            }
        }
    }
}
