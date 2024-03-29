package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap.ROBOT;

public class LEDs {
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLED led;
    private static LEDs instance;

    private LEDs() {
        led = new AddressableLED(ROBOT.LEDS.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDS.numLEDs);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public static LEDs getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new LEDs();
        }
        return instance;
    }

    private void setYellow() {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 239, 190, 0);
        }

        led.setData(ledBuffer);
    }

    private void setPurple() {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 150, 50, 255);
        }

        led.setData(ledBuffer);
    }

    private void setGreen() {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }

        led.setData(ledBuffer);
    }

    public void setTopGreen() {
        for (int i = Constants.LEDS.numLEDs / 2; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }

        led.setData(ledBuffer);
    }

    public void setBottomGreen() {
        for (int i = 0; i < Constants.LEDS.numLEDs / 2; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }

        led.setData(ledBuffer);
    }

    private void setRed() {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }

        led.setData(ledBuffer);
    }

    private void setBlue() {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 0, 0, 255);
        }

        led.setData(ledBuffer);
    }

    private void setWhite() {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 255, 255, 255);
        }

        led.setData(ledBuffer);
    }

    public void setRGBValue(int r, int g, int b) {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setColor(Color color) {
        if (color == Color.kGreen) {
            setGreen();
        } else if (color == Color.kRed) {
            setRed();
        } else if (color == Color.kPurple) {
            setPurple();
        } else if (color == Color.kYellow) {
            setYellow();
        } else if (color == Color.kBlue) {
            setBlue();
        } else if (color == Color.kWhite) {
            setWhite();
        } else {
            setOff();
        }
    }

    public void setOff() {
        for (int i = 0; i < Constants.LEDS.numLEDs; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }
}