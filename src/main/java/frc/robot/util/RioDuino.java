package frc.robot.util;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class RioDuino {

    I2C i2c;
    private byte[] toReceive = new byte[10];
    int toSend = 0;
    boolean ON = false;

    public RioDuino() {
        i2c = new I2C(Port.kMXP, 0xA0);
    }

    public boolean receiveData() {
        return i2c.read(0xA0, 10, toReceive);
    }

    public byte[] getRaw() {
        return toReceive;
    }

}

