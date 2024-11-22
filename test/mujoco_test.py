import mujoco
import mujoco.viewer
import numpy as np

# Define the XML model as a string
model_xml = """
<mujoco>
    <option timestep="0.01" gravity="0 0 0"/>

    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
        rgb2=".2 .3 .4" width="300" height="300" mark="edge" markrgb=".2 .3 .4"/>
        <material name="grid" texture="grid" texrepeat="2 2" texuniform="true"/>
    </asset>

    <worldbody>
        <!-- Ground plane -->
        <geom name="ground" type="plane" pos="0 0 0" size="10 10 0.1" material="grid" friction=".1" rgba="0.8 0.9 0.8 1"/>

        <!-- Square block -->
        <body name="block" pos="0 0 .5">
            <!-- Joints to allow movement in x and z axes and rotation around y axis -->
            <joint name="slide_x" type="slide" axis="1 0 0"/>
            <joint name="slide_z" type="slide" axis="0 0 1"/>
            <joint name="hinge_y" type="hinge" axis="0 1 0"/>
            <!-- Square represented as a box -->
            <geom name="block_geom" type="box" size="0.3 0.5 0.5" density="100" friction=".1" rgba="0.9 0.5 0.5 1"/>
        </body>

        <!-- Robot base -->
        <body name="robot_base" pos="0 0 2">
            <!-- Joints to allow movement in x and z axes -->
            <joint name="robot_slide_x" type="slide" axis="1 0 0"/>
            <joint name="robot_slide_z" type="slide" axis="0 0 1"/>
            <joint name="robot_hinge_y" type="hinge" damping="1" axis="0 1 0"/>
            <geom name="robot_geom" type="box" size=".5 .5 0.05" density="100" friction=".1" rgba="0.5 0.5 0.9 1"/>
        </body>
    </worldbody>

    <actuator>
        <!-- Actuator at the midpoint controlling movement and rotation -->
        <motor joint="robot_slide_x" ctrlrange="-10 10" ctrllimited="true"/>
        <motor joint="robot_slide_z" ctrlrange="-50 50" ctrllimited="true"/>
        <motor joint="robot_hinge_y" ctrlrange="-10 10" ctrllimited="true"/>
    </actuator>

</mujoco>
"""
        


# Load the model from the XML string
model = mujoco.MjModel.from_xml_string(model_xml)
data = mujoco.MjData(model)
options = mujoco.MjvOption()
mujoco.mjv_defaultOption(options)
options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True
# Create the viewer to visualize the simulation
viewer = mujoco.viewer.launch_passive(model, data)

# Simulation parameters
time_step = 0.01
duration = 10  # Run the simulation for 10 seconds

# Run the simulation
while viewer.is_running():

    data.ctrl[:] = [0, -1, 0]

    # Step the simulation
    mujoco.mj_step(model, data)

    # Render the scene
    viewer.sync()

    # if data.time >= duration:
    #     break

# Close the viewer properly
viewer.close()