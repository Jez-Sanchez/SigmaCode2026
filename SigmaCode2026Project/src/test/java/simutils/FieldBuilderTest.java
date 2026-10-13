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
import frc.robot.simutils.FieldBuilder;
import frc.robot.simutils.FieldBuilder.Dims;
import frc.robot.simutils.LayerConstants;
import frc.robot.simutils.PhysicsWorld;
import org.junit.jupiter.api.Test;

/**
 * Geometry checks for FieldBuilder. These drop and slide simple boxes onto the field so a wrong
 * dimension, flipped hull, or bad unit shows up here instead of during swerve debugging.
 *
 * <p>VERIFY: RVec3 accessors xx()/yy()/zz() and BodyInterface.setLinearVelocity. Use whatever your
 * PhysicsWorldTest already uses for reading positions.
 */
class FieldBuilderTest {

  private static final float DT = 0.02f;

  /**
   * Adds a dynamic box with the given half extents, centered at (x, y, z). Friction is set to zero
   * when {@code slippery} is true so a pushed box keeps sliding like a driven robot would.
   */
  private static int addBox(
      PhysicsWorld world,
      double x,
      double y,
      double z,
      float hx,
      float hy,
      float hz,
      boolean slippery) {
    BoxShape shape = new BoxShape(new Vec3(hx, hy, hz));
    BodyCreationSettings bcs =
        new BodyCreationSettings(
            shape, new RVec3(x, y, z), Quat.sIdentity(), EMotionType.Dynamic, LayerConstants.ROBOT);
    if (slippery) {
      bcs.setFriction(0f);
      // Make the box effectively unable to rotate, like a swerve chassis that stays upright.
      // A huge inertia means impacts above the center of mass cannot tip it over.
      bcs.setInertiaMultiplier(1000f); // VERIFY: name on BodyCreationSettings
    }
    return world.addBody(bcs, EActivation.Activate);
  }

  private static void run(PhysicsWorld world, double seconds) {
    int steps = (int) Math.round(seconds / DT);
    for (int i = 0; i < steps; i++) {
      world.step(DT, 1);
    }
  }

  // ---------------------------------------------------------------- sanity on the constants

  /** The trench dimensions must add up: two posts plus the passage equal the structure width. */
  @Test
  void trenchDimensionsAreConsistent() {
    // The manual rounds to about a millimeter, so allow 2 mm of slack
    double total = Dims.TRENCH_PASSAGE_WIDTH + 2.0 * Dims.TRENCH_POST_THICKNESS;
    assertEquals(
        Dims.TRENCH_WIDTH,
        total,
        0.002,
        "passage + 2 posts = " + total + " but TRENCH_WIDTH = " + Dims.TRENCH_WIDTH);

    double height = Dims.TRENCH_CLEARANCE + Dims.TRENCH_BEAM_THICKNESS;
    assertEquals(
        Dims.TRENCH_TOTAL_HEIGHT,
        height,
        0.002,
        "clearance + beam = " + height + " but TRENCH_TOTAL_HEIGHT = " + Dims.TRENCH_TOTAL_HEIGHT);
  }

  // ---------------------------------------------------------------- floor and walls

  /** A box dropped in open floor should rest with its bottom face at z = 0. */
  @Test
  void boxRestsOnFloor() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      // Open floor, far from the bump and the trench
      int id = addBox(world, 2.0, Dims.FIELD_WIDTH / 2.0, 1.0, 0.25f, 0.25f, 0.25f, false);

      run(world, 2.0);

      BodyInterface bi = world.getBodyInterface();
      assertEquals(0.25, bi.getPosition(id).zz(), 0.03, "Box should rest on the floor");
    }
  }

  /** A box pushed at the x = 0 wall must stay inside the field. */
  @Test
  void wallStopsBox() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      int id = addBox(world, 2.0, Dims.FIELD_WIDTH / 2.0, 0.26, 0.25f, 0.25f, 0.25f, true);
      world.getBodyInterface().setLinearVelocity(id, new Vec3(-5f, 0f, 0f));

      run(world, 2.0);

      double x = world.getBodyInterface().getPosition(id).xx();
      // The box center cannot get closer to the wall than its half-length
      assertTrue(x > 0.2, "Box passed through the wall, x = " + x);
    }
  }

  // ---------------------------------------------------------------- bump

  /** A small box dropped on the bump's flat top should rest at BUMP_HEIGHT plus half its height. */
  @Test
  void boxRestsOnBumpTop() {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);
      float half = 0.1f; // 0.2 m wide, fits easily on the flat top
      int id =
          addBox(
              world,
              Dims.BUMP_CENTER_X,
              Dims.BUMP_CENTER_Y,
              Dims.BUMP_HEIGHT + 1.0,
              half,
              half,
              half,
              false);

      run(world, 2.5);

      double z = world.getBodyInterface().getPosition(id).zz();
      assertEquals(
          Dims.BUMP_HEIGHT + half, z, 0.03, "Box should rest on the bump top, not the floor");
    }
  }

  // ---------------------------------------------------------------- trench

  /** Slides a box along +X through the trench passage and returns its final X position. */
  private static double slideThroughTrench(float halfHeight) {
    try (PhysicsWorld world = new PhysicsWorld(1024)) {
      FieldBuilder.build(world);

      float halfLength = 0.2f;
      float halfWidth = 0.2f;
      double startX = Dims.TRENCH_CENTER_X - Dims.TRENCH_LENGTH / 2.0 - 1.0;
      int id =
          addBox(
              world,
              startX,
              Dims.TRENCH_CENTER_Y,
              halfHeight + 0.01,
              halfLength,
              halfWidth,
              halfHeight,
              true);
      BodyInterface bi = world.getBodyInterface();
      bi.setLinearVelocity(id, new Vec3(3f, 0f, 0f));

      run(world, 2.0);

      return bi.getPosition(id).xx();
    }
  }

  /** A box shorter than the clearance must pass completely under the beam. */
  @Test
  void shortBoxPassesUnderTrench() {
    float halfHeight = 0.2f; // 0.4 m tall, below the clearance
    assertTrue(2 * halfHeight < Dims.TRENCH_CLEARANCE, "test box must be shorter than clearance");

    double x = slideThroughTrench(halfHeight);

    double trenchEnd = Dims.TRENCH_CENTER_X + Dims.TRENCH_LENGTH / 2.0;
    assertTrue(x > trenchEnd, "Short box should clear the trench, ended at x = " + x);
  }

  /** A box taller than the clearance must be stopped by the overhead beam. */
  @Test
  void tallBoxIsBlockedByTrench() {
    float halfHeight = 0.4f; // 0.8 m tall, above the clearance
    assertTrue(2 * halfHeight > Dims.TRENCH_CLEARANCE, "test box must be taller than clearance");

    double x = slideThroughTrench(halfHeight);

    double trenchStart = Dims.TRENCH_CENTER_X - Dims.TRENCH_LENGTH / 2.0;
    assertTrue(x < trenchStart, "Tall box should be blocked, ended at x = " + x);
  }
}
