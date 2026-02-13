package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkinSubsystem extends SubsystemBase {
    private static Spark m_blinkin;
    private static BlinkinSubsystem m_controller = null;
    private DriverStation.Alliance m_alliance;
    PowerDistribution m_PDH;
    private String gameData = "This will be replaced";
    public Color blue = new Color(60,60,255);
    public Color red = new Color(205,4,4);
    public Color black = new Color(0,0,0);
    public Color purple = new Color(85,21,125);
    public Color green = new Color(0,255,0);

    public BlinkinSubsystem() {
      m_blinkin = new Spark(9);
      Optional<Alliance> m_alliance = DriverStation.getAlliance();
      m_PDH = new PowerDistribution(1, ModuleType.kRev);
    }

    public static BlinkinSubsystem getInstance() {
        if (m_controller == null) {
            m_controller = new BlinkinSubsystem();
        }
        return m_controller;
    }

    public void turnOnLED(){
        m_PDH.setSwitchableChannel(true);
    }

    public void turnOfLED(){
        m_PDH.setSwitchableChannel(false);
    }

public void setColor(Color color) {
    // Map to closest Blinkin solid color
    
    if (isCloseTo(color, Color.kRed)) {
        m_blinkin.set(0.61); // Solid Red
    } else if (isCloseTo(color, Color.kBlue)) {
        m_blinkin.set(0.87); // Solid Blue
    } else if (isCloseTo(color, Color.kGreen)) {
        m_blinkin.set(0.77); // Solid Green
    } else if (isCloseTo(color, Color.kYellow)) {
        m_blinkin.set(0.69); // Solid Yellow
    } else if (isCloseTo(color, Color.kWhite)) {
        m_blinkin.set(0.93); // Solid White
    } else {
        m_blinkin.set(0.99); // Default: Black/Off
    }
}

private boolean isCloseTo(Color c1, Color c2) {
    double threshold = 0.3;
    //Since wpilib colors have RGB values between 0.0 and 1.0, this checks if the color given is within a 0.3 
    return Math.abs(c1.red - c2.red) < threshold &&
           Math.abs(c1.green - c2.green) < threshold &&
           Math.abs(c1.blue - c2.blue) < threshold;
}



// Helper method to convert WPILib Color to hex string
public String colorToHex(Color color) {
    int r = (int)(color.red * 255);
    int g = (int)(color.green * 255);
    int b = (int)(color.blue * 255);
    int rgb = (r << 16) | (g << 8) | b;
    return String.format("0x%06X", rgb & 0xFFFFFF);
}
}

