package simutils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import frc.robot.simutils.LayerConstants;
import frc.robot.simutils.PhysicsWorld;
import org.junit.jupiter.api.Test;

/**
 * Smoke tests for PhysicsWorld. VERIFY the Jolt names (BodyCreationSettings constructor, BoxShape
 * constructor, RVec3, Quat.sIdentity) against your jolt-jni version.
 */
class PhysicsWorldTest {

  private static final float DT = 0.02f;

  /** Static floor with its top surface at z = 0. */
  private static void addFloor(PhysicsWorld world) {
    BoxShape floorShape = new BoxShape(new Vec3(10f, 10f, 0.5f));
    BodyCreationSettings floor =
        new BodyCreationSettings(
            floorShape,
            new RVec3(0, 0, -0.5), // center 0.5 m below the origin
            Quat.sIdentity(),
            EMotionType.Static,
            LayerConstants.FIELD);
    world.addBody(floor, EActivation.DontActivate);
  }

  /** Dynamic cube (half-extent 0.25 m) starting at the given height. Returns its body id. */
  private static int addBox(PhysicsWorld world, double startZ) {
    BoxShape boxShape = new BoxShape(new Vec3(0.25f, 0.25f, 0.25f));
    BodyCreationSettings box =
        new BodyCreationSettings(
            boxShape,
            new RVec3(0, 0, startZ),
            Quat.sIdentity(),
            EMotionType.Dynamic,
            LayerConstants.ROBOT);
    return world.addBody(box, EActivation.Activate);
  }

  /** A box dropped onto the floor should come to rest on top of it. */
  @Test
  void boxRestsOnFloor() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      addFloor(world);
      int boxId = addBox(world, 2.0);

      for (int i = 0; i < 100; i++) { // 2 seconds of sim time
        world.step(DT, 1);
      }

      BodyInterface bi = world.getBodyInterface();
      double z = bi.getPosition(boxId).zz();
      double speed = bi.getLinearVelocity(boxId).length();

      // Resting height is 0.25 m. Allow a little for Jolt's penetration slop.
      assertEquals(0.25, z, 0.03, "Box should rest on the floor surface");
      assertTrue(speed < 0.05, "Box should be nearly stationary, was " + speed + " m/s");
    }
  }

  /** With no floor, a box must fall toward -Z. This guards the Z-up gravity setup. */
  @Test
  void gravityPointsDownInZ() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      int boxId = addBox(world, 5.0);

      for (int i = 0; i < 25; i++) { // 0.5 seconds
        world.step(DT, 1);
      }

      BodyInterface bi = world.getBodyInterface();
      RVec3 pos = bi.getPosition(boxId);
      // Free fall for 0.5 s: dz = 0.5 * g * t^2, about 1.23 m
      assertEquals(5.0 - 0.5 * 9.81 * 0.5 * 0.5, pos.zz(), 0.1);
      assertEquals(0.0, pos.xx(), 1e-3, "No sideways drift expected");
      assertEquals(0.0, pos.yy(), 1e-3, "No sideways drift expected");
    }
  }

  /** Creating and closing several worlds in one JVM must work (global init guard, close()). */
  @Test
  void worldsCanBeCreatedAndClosedRepeatedly() {
    for (int run = 0; run < 3; run++) {
      try (PhysicsWorld world = new PhysicsWorld(256)) {
        addFloor(world);
        addBox(world, 1.0);
        for (int i = 0; i < 10; i++) {
          world.step(DT, 1);
        }
      }
    }
  }
}
