package frc.robot.simutils;

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.ConvexHullShapeSettings;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.ShapeResult;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.readonly.ConstShape;

/**
 * Builds the static field geometry (floor, walls, bumps, trenches) as Jolt bodies
 *
 * <p>Coordinates follow WPILIB's field frame: origin at the blue alliance corner, +X along the
 * field length, +Y along the width, +Z up, All units are meters
 */
public class FieldBuilder {
  private FieldBuilder() {}

  public static final class Dims {
    private Dims() {}

    // ---- Overall field
    public static final double FIELD_LENGTH = 16.54; // X, meters
    public static final double FIELD_WIDTH = 8.07; // Y meters

    // Floor and Perimeter walls
    public static final double FLOOR_THICKNESS = 0.1; // thick to prevent tunneling
    public static final double WALL_HEIGHT = 0.51; // Height of guardrails
    public static final double WALL_THICKNESS = 0.3;

    // Bump Measurements
    public static final double BUMP_HEIGHT = 0.165; // meters
    public static final double BUMP_TOTAL_LENGTH = 1.128; // Base of bump, Along the X axis, meters
    public static final double BUMP_TOP_LENGTH =
        0.02; // Top length of bump, Along the X axis, meters
    public static final double BUMP_WIDTH = 1.85; // Along the Y axis, meters

    // Center Position for Bumps
    public static final double BUMP_CENTER_X = 4.6;
    public static final double BUMP_CENTER_Y = 2.2;

    // Trench Dimensions
    public static final double TRENCH_CLEARANCE =
        0.5652; // Space between floor to overhead of the beam
    public static final double TRENCH_WIDTH = 1.668;
    public static final double TRENCH_PASSAGE_WIDTH =
        1.279; // Width between the post and guardrail (width of the trench arm)
    public static final double TRENCH_LENGTH = 1.194; // Along the x
    public static final double TRENCH_TOTAL_HEIGHT = 1.022;

    public static final double TRENCH_BEAM_THICKNESS = TRENCH_TOTAL_HEIGHT - TRENCH_CLEARANCE;
    public static final double TRENCH_POST_THICKNESS = (TRENCH_WIDTH - TRENCH_PASSAGE_WIDTH) / 2.0;

    // Trench Positions
    public static final double TRENCH_CENTER_X = 4.6;
    public static final double TRENCH_CENTER_Y = TRENCH_WIDTH / 2;

    // Hubs
    public static final double HUB_CENTER_X = 4.6;
    public static final double HUB_CENTER_Y =
        8.07
            / 2.0; // Width of field divided by half since Hub is in center of the field Y axis wise

    public static final double HUB_WIDTH = 1.1938;
    public static final double HUB_HEIGHT = HUB_WIDTH;

    // Surface frictions
    public static final double FLOOR_FRICTION = 0.8f;
    public static final double WALL_FRICTION = 0.3f;
    public static final double BUMP_FRICTION = 0.6f;

    // Tiny convex radius so edges stau sharp. Jolt's default (5cm) would visibly round the bump
    public static final float CONVEX_RADIUS = 0.001f;
  }

  /** Create all static field bodies in the world. Call once, right after creating the world */
  public static void build(PhysicsWorld world) {
    addFloor(world);
    addPerimeterWalls(world);

    double[] bumpXs = {Dims.BUMP_CENTER_X, Dims.FIELD_LENGTH - Dims.BUMP_CENTER_X};
    double[] bumpYs = {Dims.BUMP_CENTER_Y, Dims.FIELD_WIDTH - Dims.BUMP_CENTER_Y};
    for (double x : bumpXs) {
      for (double y : bumpYs) {
        addBump(world, x, y);
      }
    }

    double[] trenchXs = {Dims.TRENCH_CENTER_X, Dims.FIELD_LENGTH - Dims.TRENCH_CENTER_X};
    double[] trenchYs = {Dims.TRENCH_CENTER_Y, Dims.FIELD_WIDTH - Dims.TRENCH_CENTER_Y};

    for (double x : trenchXs) {
      for (double y : trenchYs) {
        addTrench(world, x, y);
      }
    }
  }

  // Adding the floor for the Field
  private static void addFloor(PhysicsWorld world) {
    double hx = Dims.FIELD_LENGTH / 2.0;
    double hy = Dims.FIELD_WIDTH / 2.0;
    double hz = Dims.FLOOR_THICKNESS / 2.0;

    addStaticBox(world, hx, hy, -hz, hx, hy, hz, (float) Dims.FLOOR_FRICTION);
  }

  // Adding the outer walls for the field containment
  private static void addPerimeterWalls(PhysicsWorld world) {
    double L = Dims.FIELD_LENGTH;
    double W = Dims.FIELD_WIDTH;
    double t = Dims.WALL_THICKNESS / 2.0;
    double h = Dims.WALL_HEIGHT / 2.0;
    float f = (float) Dims.WALL_FRICTION;

    addStaticBox(world, L / 2.0, -t, h, L / 2.0 + t, t, h, f); // y = 0 side
    addStaticBox(world, L / 2.0, W + t, h, L / 2.0 + t, t, h, f); // y = W side
    addStaticBox(world, -t, W / 2.0, h, t, W / 2.0, h, f); // x = 0 end
    addStaticBox(world, L + t, W / 2.0, h, t, W / 2.0, h, f); // x = L end
  }

  // Function to create a convex shape of a bump (Trapezoid style)
  private static void addBump(PhysicsWorld world, double centerX, double centerY) {
    float halfBase = (float) (Dims.BUMP_TOTAL_LENGTH / 2.0);
    float halfTop = (float) (Dims.BUMP_TOP_LENGTH / 2.0);
    float h = (float) Dims.BUMP_HEIGHT;
    float halfW = (float) (Dims.BUMP_WIDTH / 2.0);

    Vec3[] points = {
      new Vec3(-halfBase, -halfW, 0f), new Vec3(halfBase, -halfW, 0f),
      new Vec3(-halfTop, -halfW, h), new Vec3(halfTop, -halfW, h),
      new Vec3(-halfBase, halfW, 0f), new Vec3(halfBase, halfW, 0f),
      new Vec3(-halfTop, halfW, h), new Vec3(halfTop, halfW, h)
    };

    ConvexHullShapeSettings hull = new ConvexHullShapeSettings(points);
    hull.setMaxConvexRadius(Dims.CONVEX_RADIUS);
    ShapeResult result = hull.create();
    if (result.hasError()) {
      throw new IllegalStateException("Bump hull failed: " + result.getError());
    }
    ConstShape shape = result.get();

    BodyCreationSettings bcs =
        new BodyCreationSettings(
            shape,
            new RVec3(centerX, centerY, 0),
            Quat.sIdentity(),
            EMotionType.Static,
            LayerConstants.FIELD);
    bcs.setFriction((float) Dims.BUMP_FRICTION);
    world.addBody(bcs, EActivation.DontActivate);
  }

  /**
   * One trench: ttwo posts, and an overhead beam. The passage runs along X. The beam's underside is
   * at TRENCH_CLEARANCE, so a robot taller than that physically hits it, with no special rule
   */
  private static void addTrench(PhysicsWorld world, double centerX, double centerY) {
    double halfLen = Dims.TRENCH_LENGTH / 2.0;
    double halfPassage = Dims.TRENCH_PASSAGE_WIDTH / 2.0;
    double postT = Dims.TRENCH_POST_THICKNESS;
    double beamT = Dims.TRENCH_BEAM_THICKNESS;
    double clearance = Dims.TRENCH_CLEARANCE;
    float f = (float) Dims.WALL_FRICTION;

    double beamHalfWidth = halfPassage + postT;
    addStaticBox(
        world, centerX, centerY, clearance + beamT / 2.0, halfLen, beamHalfWidth, beamT / 2, f);

    double postHeight = clearance + beamT;
    for (int side = -1; side <= 1l; side += 2) {
      double y = centerY + side * (halfPassage + postT / 2);
      addStaticBox(world, centerX, y, postHeight / 2, halfLen, postT / 2, postHeight / 2, f);
    }
  }

  /** Static box given its center and half extents */
  private static void addStaticBox(
      PhysicsWorld world,
      double cx,
      double cy,
      double cz,
      double hx,
      double hy,
      double hz,
      float friction) {
    BoxShape shape = new BoxShape(new Vec3((float) hx, (float) hy, (float) hz), Dims.CONVEX_RADIUS);

    BodyCreationSettings bcs =
        new BodyCreationSettings(
            shape,
            new RVec3(cx, cy, cz),
            Quat.sIdentity(),
            EMotionType.Static,
            LayerConstants.FIELD);
    bcs.setFriction(friction);
    world.addBody(bcs, EActivation.DontActivate);
  }
}
