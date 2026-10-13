package frc.robot.simutils;

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BroadPhaseLayerInterfaceTable;
import com.github.stephengold.joltjni.JobSystem;
import com.github.stephengold.joltjni.JobSystemThreadPool;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.JoltPhysicsObject;
import com.github.stephengold.joltjni.ObjectLayerPairFilterTable;
import com.github.stephengold.joltjni.ObjectVsBroadPhaseLayerFilterTable;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.TempAllocator;
import com.github.stephengold.joltjni.TempAllocatorImpl;
import com.github.stephengold.joltjni.enumerate.EActivation;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

public final class PhysicsWorld implements AutoCloseable {
  private static boolean globalInitDone = false;

  private final TempAllocator tempAllocator;
  private final JobSystem jobSystem;
  private final PhysicsSystem physics;

  // Keep references to the layer objects too, so they aren't garbage collected
  private final BroadPhaseLayerInterfaceTable bpLayerInterface;
  private final ObjectVsBroadPhaseLayerFilterTable objVsBpFilter;
  private final ObjectLayerPairFilterTable pairFilter;

  private final List<Integer> bodyIds = new ArrayList<>();
  private boolean closed = false;

  private static synchronized void initGlobal() {
    if (globalInitDone) return;

    loadJoltNatives();
    // Start the cleaner
    JoltPhysicsObject.startCleaner();
    // 3. Global registration
    Jolt.registerDefaultAllocator();
    Jolt.installDefaultAssertCallback();
    Jolt.installDefaultTraceCallback();
    if (!Jolt.newFactory()) throw new IllegalStateException("Jolt factory failed");
    Jolt.registerTypes();

    globalInitDone = true;
  }

  public PhysicsWorld(int maxBodies) {
    initGlobal();

    tempAllocator = new TempAllocatorImpl(32 * 1024 * 1024); // 32MB Scratch
    jobSystem =
        new JobSystemThreadPool(
            Jolt.cMaxPhysicsJobs,
            Jolt.cMaxPhysicsBarriers,
            Math.max(1, Runtime.getRuntime().availableProcessors() - 2));

    // Building the layers here
    pairFilter = new ObjectLayerPairFilterTable(LayerConstants.NUM_OBJECT_LAYERS);
    pairFilter.enableCollision(LayerConstants.FIELD, LayerConstants.ROBOT);
    pairFilter.enableCollision(LayerConstants.FIELD, LayerConstants.GAME_PIECE);
    pairFilter.enableCollision(LayerConstants.ROBOT, LayerConstants.ROBOT);
    pairFilter.enableCollision(LayerConstants.ROBOT, LayerConstants.GAME_PIECE);
    pairFilter.enableCollision(LayerConstants.GAME_PIECE, LayerConstants.GAME_PIECE);
    pairFilter.enableCollision(LayerConstants.SENSOR, LayerConstants.GAME_PIECE);

    // Map each object to a broad-phase layer
    bpLayerInterface =
        new BroadPhaseLayerInterfaceTable(
            LayerConstants.NUM_OBJECT_LAYERS, LayerConstants.NUM_BP_LAYERS);
    bpLayerInterface.mapObjectToBroadPhaseLayer(LayerConstants.FIELD, LayerConstants.BP_STATIC);
    bpLayerInterface.mapObjectToBroadPhaseLayer(LayerConstants.ROBOT, LayerConstants.BP_MOVING);
    bpLayerInterface.mapObjectToBroadPhaseLayer(
        LayerConstants.GAME_PIECE, LayerConstants.BP_MOVING);
    bpLayerInterface.mapObjectToBroadPhaseLayer(LayerConstants.SENSOR, LayerConstants.BP_MOVING);

    // Which object layers can hit which broad-phase layers
    objVsBpFilter =
        new ObjectVsBroadPhaseLayerFilterTable(
            bpLayerInterface,
            LayerConstants.NUM_BP_LAYERS,
            pairFilter,
            LayerConstants.NUM_OBJECT_LAYERS);

    physics = new PhysicsSystem();

    physics.init(maxBodies, 0, 65536, 20480, bpLayerInterface, objVsBpFilter, pairFilter);

    physics.setGravity(0f, 0f, -9.81f);
  }

  public void step(float dt, int collisionSteps) {
    physics.update(dt, collisionSteps, tempAllocator, jobSystem);
  }

  /** Create a body, add it to the world, and track it for cleanup. */
  public int addBody(BodyCreationSettings settings, EActivation activation) {
    int id = physics.getBodyInterface().createAndAddBody(settings, activation);
    bodyIds.add(id);
    return id;
  }

  public PhysicsSystem getPhysicsSystem() {
    return physics;
  }

  public BodyInterface getBodyInterface() {
    return physics.getBodyInterface();
  }

  private static void loadJoltNatives() {
    String os = System.getProperty("os.name").toLowerCase();
    String arch = System.getProperty("os.arch").toLowerCase();

    String osDir;
    String fileName;
    if (os.contains("win")) {
      osDir = "windows";
      fileName = "joltjni.dll";
    } else if (os.contains("mac")) {
      osDir = "osx";
      fileName = "libjoltjni.dylib";
    } else {
      osDir = "linux";
      fileName = "libjoltjni.so";
    }
    String archDir = (arch.equals("aarch64") || arch.equals("arm64")) ? "aarch64" : "x86-64";
    String resource = "/" + osDir + "/" + archDir + "/com/github/stephengold/" + fileName;

    try (InputStream in = PhysicsWorld.class.getResourceAsStream(resource)) {
      if (in == null) {
        throw new IllegalStateException(
            "Jolt native not found on the classpath at "
                + resource
                + " (os="
                + os
                + ", arch="
                + arch
                + "). Is the jolt-jni native jar for your"
                + " platform in build.gradle, and does it use implementation, not compileOnly?");
      }
      Path dir = Files.createTempDirectory("joltjni");
      Path out = dir.resolve(fileName);
      long bytes = Files.copy(in, out, StandardCopyOption.REPLACE_EXISTING);
      if (bytes == 0) {
        throw new IllegalStateException(
            "Jolt native at "
                + resource
                + " is 0 bytes inside the jar. Delete the jolt-jni"
                + " folders in ~/.gradle/caches/modules-2/files-2.1/com.github.stephengold and refresh.");
      }
      out.toFile().deleteOnExit();
      dir.toFile().deleteOnExit();
      System.load(out.toAbsolutePath().toString());
      System.out.println("Loaded Jolt native (" + bytes + " bytes) from " + out);
    } catch (IOException e) {
      throw new IllegalStateException("Failed to extract the Jolt native library", e);
    }
  }

  @Override
  public synchronized void close() {
    if (closed) return;
    closed = true;

    // Remove bodies from the world then destroy it
    BodyInterface bi = physics.getBodyInterface();
    for (int id : bodyIds) {
      if (bi.isAdded(id)) {
        bi.removeBody(id);
      }
      bi.destroyBody(id);
    }

    // Close out the physics world, memory allocator, and job system
    physics.close();
    jobSystem.close();
    tempAllocator.close();
  }
}
