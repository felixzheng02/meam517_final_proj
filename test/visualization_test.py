import numpy as np
import time
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    SceneGraph,
    Simulator,
    StartMeshcat,
    MultibodyPlant,
    RotationalInertia,
    SpatialInertia,
    PlanarJoint,
    UnitInertia,
    Cylinder,
    RigidTransform,
    AddMultibodyPlantSceneGraph
)

# from underactuated.meshcat_utils import MeshcatSliders

meshcat = StartMeshcat()
builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Step 2: Set gravity to zero if desired
plant.mutable_gravity_field().set_gravity_vector([0, 0, 0])

# Step 3: Define properties of the rod (robot)
length = 1.0  # Length of the line (rod)
mass = 1.0    # Mass of the rod
radius = 0.01 # Radius for visualization purposes

# Step 4: Compute spatial inertia for the rod
Izz = (1/12) * length**2
rotational_inertia = UnitInertia(0, 0, Izz)
spatial_inertia = SpatialInertia(
    mass=mass,
    p_PScm_E=np.zeros(3),
    G_SP_E=rotational_inertia,
    skip_validity_check=True
)

# Step 5: Add the rod (rigid body) to the plant
rod = plant.AddRigidBody("rod", spatial_inertia)

# Step 6: Add visual geometry to the rod for visualization
rod_shape = Cylinder(radius, length)
plant.RegisterVisualGeometry(
    rod, RigidTransform(), rod_shape, "rod_visual", [0.5, 0.5, 0.5, 1.0]
)

# Step 7: Attach the rod to the world via a PlanarJoint
joint = plant.AddJoint(PlanarJoint(
    name="planar_joint",
    frame_on_parent=plant.world_frame(),
    frame_on_child=rod.body_frame()
))

# Step 8: Add an actuator to the joint to control the robot
# actuator = plant.AddJointActuator("actuator", joint)

# Step 9: Finalize the plant
plant.Finalize()

# saturation = builder.AddSystem(Saturation(min_value=[-10], max_value=[10]))
# builder.Connect(saturation.get_output_port(0), acrobot.get_input_port(0))
# wrapangles = WrapToSystem(4)
# wrapangles.set_interval(0, 0, 2.0 * np.pi)
# wrapangles.set_interval(1, -np.pi, np.pi)
# wrapto = builder.AddSystem(wrapangles)
# builder.Connect(acrobot.get_output_port(0), wrapto.get_input_port(0))
# controller = builder.AddSystem(BalancingLQR())
# builder.Connect(wrapto.get_output_port(0), controller.get_input_port(0))
# builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))

# Setup visualization
# scene_graph = builder.AddSystem(SceneGraph())
# AcrobotGeometry.AddToBuilder(builder, acrobot.get_output_port(0), scene_graph)
meshcat.Delete()
meshcat.Set2dRenderMode(xmin=-4, xmax=4, ymin=-4, ymax=4)
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

diagram = builder.Build()

# Set up a simulator to run this diagram
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
# Step 14: Set initial conditions
plant_context = plant.GetMyContextFromRoot(context)

# # Define initial positions
# x0 = 1.0           # Initial X position in meters
# y0 = 2.0           # Initial Y position in meters
# theta0 = np.pi / 4 # Initial orientation in radians (45 degrees)

# # Set the initial positions for the joint
# plant.SetPositions(plant_context, joint, [x0, y0, theta0])
# Simulate
simulator.set_target_realtime_rate(1.0)
duration = 4.0
for i in range(10):
    context.SetTime(0.0)
    # context.SetContinuousState(
    #     UprightState().CopyToVector()+ 0.05 * np.random.randn(4,)
    # )
    simulator.Initialize()
    simulator.AdvanceTo(duration)
    time.sleep(1)