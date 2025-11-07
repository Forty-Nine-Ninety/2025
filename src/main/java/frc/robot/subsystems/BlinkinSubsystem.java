package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class BlinkinSubsystem extends SubsystemBase {
   private static Spark m_blinkin;
   private static BlinkinSubsystem m_controller = null;
   private DriverStation.Alliance m_alliance;
   PowerDistribution m_PDH;

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

   public void changeLEDColor() {
      // Update the alliance in case it changes
      Optional<Alliance> m_alliance = DriverStation.getAlliance();  
      
      if (m_alliance.isPresent()){
         
         if (m_alliance.get() == Alliance.Red) {
            m_blinkin.set(0.61);  // Set red color (solid colors, red)

         } else if (m_alliance.get() == Alliance.Blue) {
            m_blinkin.set(0.85); // Set blue color (solid colors, dark blue)

         } else {
            m_blinkin.set(-0.99);  // Set default color (rainbow palette)
         }
      }
   }
}

