package org.firstinspires.ftc.teamcode.algorithms.motioncontrol;

public abstract class kPDVA {

    double[] k = new double[4];
    double[] error = new double[2];
    ProfilePoint setPoint = new ProfilePoint(0,0,0,0);
    boolean active = false ;
    long dt;
    private long time;


    public kPDVA(double kP, double kD, double kV, double kA, long dt){
        k[0] = kP;
        k[1] = kD;
        k[2] = kV;
        k[3] = kA;
        this.dt = dt;
    }

    Thread T= new Thread(new Runnable() {
        @Override
        public void run() {
            while (active){
                error();
                try {Thread.sleep(dt);
                } catch (InterruptedException e){}
            }
        }
    });

    public void activate(){
        active = true;
        time = System.currentTimeMillis();
        T.start();
    }

    public void deactivate(){
        active = false;
    }

    private void error(){
        double prevError = error[0];
        double dt_a = System.currentTimeMillis() - time;
        error[0] = getPosition() - setPoint.getPosition();
        error[1] = (error[0] - prevError)/dt_a;
        time = System.currentTimeMillis();
    }

    public double output(ProfilePoint in){
        setPoint = in;
        return k[0] * error[0] + k[1] * error[1] + k[2] * in.getVelocity() + k[3] * in.getAcceleration();
    }

    protected abstract double getPosition();
    protected abstract void reset();


}
