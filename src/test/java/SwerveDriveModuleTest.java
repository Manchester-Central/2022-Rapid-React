import static org.junit.Assert.*;
import org.junit.*;

import frc.robot.subsystems.AngleUtil;
import frc.robot.subsystems.SwerveDriveModule;

public class SwerveDriveModuleTest {
  public static final double DELTA = 1e-2; // acceptable deviation range

  @Before // this method will run before each test
  public void setup() {
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {

  }

  @Test // marks this method as a test
  public void TestClosestTarget() {
    assertEquals(15.0, AngleUtil.closestTarget(10, 15), DELTA);
    assertEquals(375.0, AngleUtil.closestTarget(350, 15), DELTA);
    assertEquals(15.0, AngleUtil.closestTarget(194, 15), DELTA);
    assertEquals(375.0, AngleUtil.closestTarget(196, 15), DELTA);
    assertEquals(-10.0, AngleUtil.closestTarget(15, 350), DELTA);
    assertEquals(-10.0, AngleUtil.closestTarget(169, 350), DELTA);
    assertEquals(350.0, AngleUtil.closestTarget(171, 350), DELTA);
    assertEquals(540.0, AngleUtil.closestTarget(536, 540), DELTA);
    assertEquals(580.0, AngleUtil.closestTarget(536, 580), DELTA);
    assertEquals(539.0, AngleUtil.closestTarget(536, 179), DELTA);
    assertEquals(541.0, AngleUtil.closestTarget(536, -179), DELTA);
    assertEquals(-539.0, AngleUtil.closestTarget(-536, -179), DELTA);
    assertEquals(-541.0, AngleUtil.closestTarget(-536, 179), DELTA);
  }

  // "Inflation happens man" - JoshuaAllard2022
}