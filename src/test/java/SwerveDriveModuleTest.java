import static org.junit.Assert.*;
import org.junit.*;

import frc.robot.subsystems.SwerveDriveModule;


public class SwerveDriveModuleTest {
    public static final double DELTA = 1e-2; // acceptable deviation range
    public SwerveDriveModule m_swerveDriveModule;
  
    @Before // this method will run before each test
    public void setup() {
    m_swerveDriveModule = new SwerveDriveModule(0, 0, 0, "Hi!", 1, 2);
    }
  
    @After // this method will run after each test
    public void shutdown() throws Exception {

    }
  
    @Test // marks this method as a test
    public void doesntWorkWhenClosed() {
      assertEquals(15.0, m_swerveDriveModule.closestTarget(10, 15), DELTA); // make sure that the value set to the motor is 0
    }
  }