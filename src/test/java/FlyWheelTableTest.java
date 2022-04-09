import static org.junit.Assert.*;
import java.nio.file.Path;

import org.junit.*;

import frc.robot.subsystems.FlywheelTable;

public class FlyWheelTableTest {

  @Before // this method will run before each test
  public void setup() {
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {

  }

  @Test // marks this method as a test
  public void TestCSV() {
    var path = Path.of(System.getProperty("user.dir"), "src/main/deploy", FlywheelTable.PATH);
    assertTrue(new FlywheelTable().readCSV(path)); // If something is wrong, an exception will be thrown
  }

}