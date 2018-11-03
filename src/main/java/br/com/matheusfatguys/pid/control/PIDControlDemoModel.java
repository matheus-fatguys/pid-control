/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package br.com.matheusfatguys.pid.control;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.util.Timer;
import java.util.TimerTask;

/**
 *
 * @author y2gh
 */
public class PIDControlDemoModel implements PropertyChangeListener {

    private double P = 0.6 * 0.36 * 10;//0.6 * 0.36;
    private double I = 0; 
    private double D = 10000 * 10; //1000
    private int power = 7000;
    private int maxThrotle = 100;
    private int mass = 1000;
    private double doubleMass = mass / 100d;
    private int friction = 8;
    private double doubleFriction = friction / 100000d;
    private double doublePower = power / 0.1d;
    private int setPoint = 0;
    private int position = 0;
    private double doublePosition = 0;
    private double doubleSetPoint = 0;
    private double acc = 0;
    private double error = 0;
    private double lastError = 0;
    private double sumError = 0;
    private double speed = 0;
    private double pidGain = 0;
    private double doubleP = 0;
    private double doubleD = 0;
    private double doubleI = 0;
    private double pGain = 0;
    private double dGain = 0;
    private double iGain = 0;
    private double gravity = 9.8;
    private double resultingForce;
    private double force;
    private TimerTask task;
    private double timeStep;
    private double lastTimeStep;
    private double timeDelta;
    private int throtle = -100;
    private double doubleThrotle = 0;
    private boolean activatePidController;
    private boolean flyingModel = false;
    private boolean spaceModel = true;
    private boolean groundModel = false;
    private int frequency = 100;
    private double radius = 1;

    public PIDControlDemoModel() {
        Timer timer = new Timer(true);
        task = new TimerTask() {
            @Override
            public void run() {
                timeDelta = timeStep - lastTimeStep;

                setDoubleP(P / 10d);
                setDoubleD(D / 10d);
                setDoubleI(I / 100d);

                setDoubleMass(mass == 0 ? 0.000001 : mass / 100d); //[0.000001 - 100]kg                 
                setDoublePower(power / 5d); //[0 - 2,000.00] watts
                setDoubleSetPoint(setPoint / 10000d * radius); //[-1 - 1]m

                if (flyingModel) {
                    setDoubleThrotle((throtle + 100d) / 2d); // [0% - 100%]
                } else {
                    setDoubleThrotle(throtle); // [-100% - 100%]
                }

                if (activatePidController) {

                    pidGain = pid();

                    if (flyingModel) {
                        if (error < 0 && error > -radius * 0.1 && pidGain < 0 && speed < 0) {
                            pidGain = -0.01 * pidGain;
                        }

                        if (pidGain < 0) {
                            pidGain = 0;
                        }
                        setDoubleThrotle(pidGain);
                        setThrotle((int) doubleThrotle * 2 - 100); // // [-100% - 100%]
                    } else {
                        setDoubleThrotle(pidGain);
                        setThrotle((int) doubleThrotle);
                    }
                }

                double p;
                if (flyingModel) {
                    p = modelFlying();
                } else if (groundModel){
                    p = modelGround();
                }
                else {
                    p = modelSpace();
                }
                setPosition((int) ((p) * 10000 / radius));
                lastTimeStep = timeStep;
                setTimeStep(timeStep + (1000d / frequency) / 1000d);
            }

        };
        timer.scheduleAtFixedRate(task, 0, 1000 / frequency);
    }

    private double pid() {
        lastError = error;
        double newError = doubleSetPoint - doublePosition;
       
        sumError += newError;
        if (sumError > radius) {
            sumError = radius;
        } else if (sumError < -radius) {
            sumError = -radius;
        }
        pGain = doubleP * newError;
        dGain = doubleD * (newError - lastError) * timeDelta;
        iGain = doubleI * sumError;
        pidGain = pGain + iGain + dGain;

        if (pidGain > maxThrotle) { // max =  100%
            pidGain = maxThrotle;
        } else if (pidGain < -maxThrotle) { // min =  -100%
            pidGain = -maxThrotle;
        }

        setError(newError);

        return pidGain;
    }

    private double modelSpace() {

        resultingForce = doublePower * doubleThrotle / maxThrotle;

        if (doublePosition <= -radius && resultingForce < 0 || doublePosition >= radius && resultingForce > 0) {
            resultingForce = 0;
        }

        double newAcc = (resultingForce / doubleMass);
        double newSpeed = speed + newAcc * timeDelta;

        double newPosition = (doublePosition + newSpeed * timeDelta);

        if (newPosition > radius) {
            newPosition = radius;
            newSpeed = 0;
        } else if (newPosition < -radius) {
            newPosition = -radius;
            newSpeed = 0;
        }

        setForce(resultingForce);
        setAcc(newAcc);
        setSpeed(newSpeed);
        setDoublePosition(newPosition);

        return newPosition;
    }
    
    private double modelFlying() {

        double newFriction = (friction / 100000d); //[0 - 0.10]

        doublePower = doublePower * doubleThrotle / maxThrotle;

        double newForce = doublePower;
//        force = speed != 0 ? doublePower / Math.abs(speed) : doublePower;

        double frictionForce = newFriction * speed * speed;
        double weigthForce = doubleMass * gravity;

        resultingForce = newForce - frictionForce - weigthForce;

        if (doublePosition <= -10d && resultingForce < 0 || doublePosition >= 10d && resultingForce > 0) {
            resultingForce = 0;
        }

        double newAcc = (resultingForce / doubleMass);

        double newSpeed = speed + newAcc * timeDelta;
        if ((doublePosition <= -radius && newSpeed < 0 || doublePosition >= radius && newSpeed > 0)
                || (((speed < 0 && newSpeed > 0) || (speed > 0 && newSpeed < 0))
                && Math.abs(newForce) < Math.abs(frictionForce))) {
//        if ((speed < 0 && newSpeed > 0) && Math.abs(force) < Math.abs(frictionForce)) {
            newSpeed = (0);
        }

        double newPosition = (doublePosition + newSpeed * timeDelta);

        if (newPosition > radius) {
            newPosition = (radius);
            newSpeed = (0);
        } else if (newPosition < -radius) {
            newPosition = (-radius);
            newSpeed = (0);
        }

        setDoubleFriction(newFriction); //[0 - 1]
        setForce(newForce); //[0 - 1]
        setAcc(newAcc);
        setSpeed(newSpeed);
        setDoublePosition(newPosition);

        return newPosition;
    }

    private double modelGround() {

        double newFriction = (friction / 10000d); //[0 - 1]

        doublePower = doublePower * doubleThrotle / maxThrotle;

        double newForce = doublePower;

        double frictionForce = newForce < 0 ? newFriction * doubleMass * gravity : -newFriction * doubleMass * gravity;

        resultingForce = newForce - frictionForce;
        if (Math.abs(newForce) < Math.abs(frictionForce)) {
            resultingForce = 0;
        }

        if (doublePosition <= -radius && resultingForce < 0 || doublePosition >= radius && resultingForce > 0) {
            resultingForce = 0;
        }

        double newAcc = (resultingForce / doubleMass);
        double newSpeed = speed + newAcc * timeDelta;

        if ((doublePosition <= -10d && newSpeed < 0 || doublePosition >= 10d && newSpeed > 0)
                || (((newSpeed < 0 && speed > 0) || (newSpeed > 0 && speed < 0))
                && Math.abs(newForce) < Math.abs(frictionForce))) {
//        if ((speed < 0 && newSpeed > 0) && Math.abs(force) < Math.abs(frictionForce)) {
            newSpeed = 0;
        }

        double newPosition = (doublePosition + newSpeed * timeDelta);

        if (newPosition > radius) {
            newPosition = radius;
            newSpeed = 0;
        } else if (newPosition < -radius) {
            newPosition = -radius;
            newSpeed = 0;
        }

        setDoubleFriction(newFriction); //[0 - 1]
        setForce(newForce); //[0 - 1]
        setAcc(newAcc);
        setSpeed(newSpeed);
        setDoublePosition(newPosition);

        return newPosition;
    }

    public boolean isSpaceModel() {
        return spaceModel;
    }

    public void setSpaceModel(boolean spaceModel) {
        Object old = this.spaceModel;
        this.spaceModel = spaceModel;
        firePropertyChange("spaceModel", old, spaceModel);
    }

    public boolean isGroundModel() {
        return groundModel;
    }

    public void setGroundModel(boolean groundModel) {
        Object old = this.groundModel;
        this.groundModel = groundModel;
        firePropertyChange("groundModel", old, groundModel);
    }

    public double getAcc() {
        return acc;
    }

    public void setAcc(double acc) {
        Object old = this.acc;
        this.acc = acc;
        firePropertyChange("acc", old, acc);
    }

    public double getTimeStep() {
        return timeStep;
    }

    public void setTimeStep(double timeStep) {
        Object old = this.timeStep;
        this.timeStep = timeStep;
        firePropertyChange("timeStep", old, timeStep);
    }

    public int getMaxThrotle() {
        return maxThrotle;
    }

    public void setMaxThrotle(int maxThrotle) {
        Object old = this.maxThrotle;
        this.maxThrotle = maxThrotle;
        firePropertyChange("maxThrotle", old, maxThrotle);
    }

    public double getDoubleMass() {
        return doubleMass;
    }

    public void setDoubleMass(double doubleMass) {
        Object old = this.doubleMass;
        this.doubleMass = doubleMass;
        firePropertyChange("doubleMass", old, doubleMass);
    }

    public double getDoubleFriction() {
        return doubleFriction;
    }

    public void setDoubleFriction(double doubleFriction) {
        Object old = this.doubleFriction;
        this.doubleFriction = doubleFriction;
        firePropertyChange("doubleFriction", old, doubleFriction);
    }

    public double getDoublePower() {
        return doublePower;
    }

    public void setDoublePower(double doublePower) {
        Object old = this.doublePower;
        this.doublePower = doublePower;
        firePropertyChange("doublePower", old, doublePower);
    }

    public double getDoublePosition() {
        return doublePosition;
    }

    public void setDoublePosition(double doublePosition) {
        Object old = this.doublePosition;
        this.doublePosition = doublePosition;
        firePropertyChange("doublePosition", old, doublePosition);
    }

    public double getDoubleSetPoint() {
        return doubleSetPoint;
    }

    public void setDoubleSetPoint(double doubleSetPoint) {
        Object old = this.doubleSetPoint;
        this.doubleSetPoint = doubleSetPoint;
        firePropertyChange("doubleSetPoint", old, doubleSetPoint);
    }

    public double getError() {
        return error;
    }

    public void setError(double error) {
        Object old = this.error;
        this.error = error;
        firePropertyChange("error", old, error);
    }

    public double getLastError() {
        return lastError;
    }

    public void setLastError(double lastError) {
        Object old = this.lastError;
        this.lastError = lastError;
        firePropertyChange("lastError", old, lastError);
    }

    public double getSumError() {
        return sumError;
    }

    public void setSumError(double sumError) {
        Object old = this.sumError;
        this.sumError = sumError;
        firePropertyChange("sumError", old, sumError);
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        Object old = this.speed;
        this.speed = speed;
        firePropertyChange("speed", old, speed);
    }

    public double getPidGain() {
        return pidGain;
    }

    public void setPidGain(double pidGain) {
        Object old = this.pidGain;
        this.pidGain = pidGain;
        firePropertyChange("pidGain", old, pidGain);
    }

    public double getDoubleP() {
        return doubleP;
    }

    public void setDoubleP(double doubleP) {
        Object old = this.doubleP;
        this.doubleP = doubleP;
        firePropertyChange("doubleP", old, doubleP);
    }

    public double getDoubleD() {
        return doubleD;
    }

    public void setDoubleD(double doubleD) {
        Object old = this.doubleD;
        this.doubleD = doubleD;
        firePropertyChange("doubleD", old, doubleD);
    }

    public double getDoubleI() {
        return doubleI;
    }

    public void setDoubleI(double doubleI) {
        Object old = this.doubleI;
        this.doubleI = doubleI;
        firePropertyChange("doubleI", old, doubleI);
    }

    public double getpGain() {
        return pGain;
    }

    public void setpGain(double pGain) {
        Object old = this.pGain;
        this.pGain = pGain;
        firePropertyChange("pGain", old, pGain);
    }

    public double getiGain() {
        return iGain;
    }

    public void setiGain(double iGain) {
        Object old = this.iGain;
        this.iGain = iGain;
        firePropertyChange("iGain", old, iGain);
    }

    public double getGravity() {
        return gravity;
    }

    public void setGravity(double gravity) {
        Object old = this.gravity;
        this.gravity = gravity;
        firePropertyChange("gravity", old, gravity);
    }

    public double getResultingForce() {
        return resultingForce;
    }

    public void setResultingForce(double resultingForce) {
        Object old = this.resultingForce;
        this.resultingForce = resultingForce;
        firePropertyChange("resultingForce", old, resultingForce);
    }

    public double getForce() {
        return force;
    }

    public void setForce(double force) {
        Object old = this.force;
        this.force = force;
        firePropertyChange("force", old, force);
    }

    public double getLastTimeStep() {
        return lastTimeStep;
    }

    public void setLastTimeStep(double lastTimeStep) {
        Object old = this.lastTimeStep;
        this.lastTimeStep = lastTimeStep;
        firePropertyChange("lastTimeStep", old, lastTimeStep);
    }

    public double getTimeDelta() {
        return timeDelta;
    }

    public void setTimeDelta(double timeDelta) {
        Object old = this.timeDelta;
        this.timeDelta = timeDelta;
        firePropertyChange("timeDelta", old, timeDelta);
    }

    public double getDoubleThrotle() {
        return doubleThrotle;
    }

    public void setDoubleThrotle(double doubleThrotle) {
        Object old = this.doubleThrotle;
        this.doubleThrotle = doubleThrotle;
        firePropertyChange("doubleThrotle", old, doubleThrotle);
    }

    public int getFrequency() {
        return frequency;
    }

    public void setFrequency(int frequency) {
        Object old = this.frequency;
        this.frequency = frequency;
        firePropertyChange("frequency", old, frequency);
    }

    public boolean isFlyingModel() {
        return flyingModel;
    }

    public void setFlyingModel(boolean flyingModel) {
        Object old = this.flyingModel;
        this.flyingModel = flyingModel;
        firePropertyChange("flyingModel", old, flyingModel);
    }

    public boolean isActivatePidController() {
        return activatePidController;
    }

    public void setActivatePidController(boolean activatePidController) {
        Object old = this.activatePidController;
        this.activatePidController = activatePidController;
        firePropertyChange("activatePidController", old, activatePidController);
    }

    public int getThrotle() {
        return throtle;
    }

    public void setThrotle(int throtle) {
        Object old = this.throtle;
        this.throtle = throtle;
        firePropertyChange("throtle", old, throtle);
    }

    public int getP() {
        return (int)P;
    }

    public void setP(int P) {
        Object old = this.P;
        this.P = P;
        firePropertyChange("P", old, P);
    }

    public int getI() {
        return (int)I;
    }

    public void setI(int I) {
        Object old = this.I;
        this.I = I;
        firePropertyChange("I", old, I);
    }

    public int getD() {
        return (int)D;
    }

    public void setD(int D) {
        Object old = this.D;
        this.D = D;
        firePropertyChange("D", old, D);
    }

    public int getPower() {
        return power;
    }

    public void setPower(int power) {
        Object old = this.power;
        this.power = power;
        firePropertyChange("power", old, power);
    }

    public int getMass() {
        return mass;
    }

    public void setMass(int mass) {
        Object old = this.mass;
        this.mass = mass;
        firePropertyChange("mass", old, mass);
    }

    public int getFriction() {
        return friction;
    }

    public void setFriction(int friction) {
        Object old = this.friction;
        this.friction = friction;
        firePropertyChange("friction", old, friction);
    }

    public void setGravity(int gravity) {
        Object old = this.gravity;
        this.gravity = gravity;
        firePropertyChange("gravity", old, gravity);
    }

    public int getSetPoint() {
        return setPoint;
    }

    public void setSetPoint(int setPoint) {
        Object old = this.setPoint;
        this.setPoint = setPoint;
        firePropertyChange("setPoint", old, setPoint);
    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        Object old = this.position;
        this.position = position;
        firePropertyChange("position", old, position);
    }

    protected PropertyChangeSupport support = new PropertyChangeSupport(this);

    public void addPropertyChangeListener(final PropertyChangeListener listener) {
        support.addPropertyChangeListener(listener);
    }

    public void removePropertyChangeListener(final PropertyChangeListener listener) {
        support.removePropertyChangeListener(listener);
    }

    public void removeAllPropertyChangeListeners() {
        for (PropertyChangeListener propertyChangeListener : support.getPropertyChangeListeners()) {
            support.removePropertyChangeListener(propertyChangeListener);
        }
    }

    public void firePropertyChange(final String prop, final Object oldValue, final Object newValue) {
        support.firePropertyChange(prop, oldValue, newValue);
    }

    @Override
    public void propertyChange(PropertyChangeEvent evt) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

}
