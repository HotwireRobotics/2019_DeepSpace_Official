
package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class HotPot {

    public double potHomeValue = 1126.6;
    public AnalogPotentiometer pot;

    public HotPot(int potIndex){
        pot = new AnalogPotentiometer(potIndex);
    }

    public double get(){
        return((pot.get()  + 1) * 1000  - potHomeValue);
    }
}