# Here we hope to chieve simulation of our biped robot. Options with and without control added.

using RigidBodyDynamics
using MeshCatMechanisms
using RigidBodyDynamics.Contact
using Random
using StaticArrays
using Symbolics

## first we will try do define the robot mechanism with symbolic variables. Then we will export the robot model. 
## there are some limitations with this method. for example, only inertia tags are supported for the links
## second we will replace the symbolic variables by float numbers and export the URDF
## third we will add contact points and simulate the robot  

## First step
# inertias = @variables m_hip m_r_thigh m_r_leg m_r_foot m_l_thigh m_l_leg m_l_foot  I_hip I_r_thigh I_r_leg I_r_foot I_l_thigh I_l_leg I_l_foot positive = true
inertias = @variables m_h m_r_t m_r_l m_r_f m_l_t m_l_l m_l_f  I_h I_r_t I_r_l I_r_f I_l_t I_l_l I_l_f positive = true
lengths = @variables l_h_x l_h_y l_h_z l_r_t c_r_t l_r_l c_r_l c_r_f l_l_t c_l_t l_l_l c_l_l c_l_f real = true
## the variables are, in order, the x,y,z lengths of the hips (a rectangular box for the moment), 
## the lenght and circunference of right thigh and of the right leg (cilindrical for the moment)
## the lenght and circunference of left thigh and of the left leg (cilindrical for the moment)

gravitational_acceleration = @variables g real = true
params = [inertias..., lengths..., gravitational_acceleration...]
transpose(params)

# ## Create robot `Mechanism`
# A `Mechanism` contains the joint layout and inertia parameters, but no state information.

T = Num # the 'type' of the Mechanism we'll construct
axis = SVector(zero(T), one(T), zero(T)) # axis of rotation for each for the knee and hips joints (around y-axis)
robot = Mechanism(RigidBody{T}("world"); gravity = SVector(zero(T), zero(T), g))
world = root_body(robot) # the fixed 'world' rigid body

## modify the mechanism

# Attach the hip link to the world via a floating joint named 'world_to_hips'
## unfortunately we cannot define floating joint through RigidBodyDynamics because the method was not create in RigidBodyDynamics but through URDF yes. Lets try to create the boom mechanism 

## creates a base_link
# axis_fix = SVector( zero(T), zero(T),  zero(T)) ## special axis to make revolute joint become fixed joint
# inertia1 = SpatialInertia(CartesianFrame3D("base_link"),
#     moment=I_h * axis * transpose(axis),
#     com=SVector(l_h_x,l_h_y,l_h_z),
#     mass=m_h)
# body1 = RigidBody(inertia1)
# joint1 = Joint("world_to_base_link", Revolute(axis_fix))
# joint1_to_world = one(Transform3D{T}, frame_before(joint1), default_frame(world));
# attach!(robot, world, body1, joint1,
#     joint_pose = joint1_to_world);

# ##
# axis_hips = SVector( one(T), one(T),  one(T)) ## special axis since the hips can rotate in the 3 axis
# inertia1 = SpatialInertia(CartesianFrame3D("hips_link"),
#     moment=I_h * axis * transpose(axis),
#     com=SVector(l_h_x,l_h_y,l_h_z),
#     mass=m_h)
# body1 = RigidBody(inertia1)
# joint1 = Joint("world_to_hips", Prismatic(axis_hips))
# joint1_to_world = one(Transform3D{T}, frame_before(joint1), default_frame(world));
# attach!(robot, world, body1, joint1,
#     joint_pose = joint1_to_world);

# # Attach the right thigh link to the hips link via a revolute joint named 'r_hips_joint'
# inertia2 = SpatialInertia(CartesianFrame3D("r_thigh_link"),
#     moment=I_r_t * axis * transpose(axis),
#     com=SVector(zero(T), zero(T), c_r_t),
#     mass=m_r_t)
# body2 = RigidBody(inertia2)
# joint2 = Joint("r_hips_joint", Revolute(axis))
# joint2_to_body1 = Transform3D(
#     frame_before(joint2), default_frame(body1), SVector(zero(T), zero(T), l_r_t))
# attach!(robot, body1, body2, joint2,
#     joint_pose = joint2_to_body1)

# ## to be continued...

## Export URDF
# write_urdf("test.urdf", double_pendulum; robot_name="double_pendulum", include_root=true)
## the problem with the write_urdf function is that is does not include some tags as the visual one

# # ## Create `MechanismState` associated with the double pendulum `Mechanism`
# # A `MechanismState` stores all state-dependent information associated with a `Mechanism`.

# x = MechanismState(robot);

# # Joints vector
# q = configuration(x)

# # Velocities vector
# v = velocity(x)

# # ## Compute dynamical quantities in symbolic form

# # Mass matrix
# simplify.(mass_matrix(x)) # This gives you the general inertia matrix M(x) of the robot

# #### If we want to load the URDF instead of creating the robot with RigidBodyDynamics
## describe path to urdf file
packagepath() = joinpath(@__DIR__, "..","..", "deps")
urdfpath() = joinpath(packagepath(), "BipedRobot", "biped_robot.urdf")

## load mechanism 'robot' from the URDF file
robot = parse_urdf(Float64, urdfpath())

# list_of_joints = joints(robot)
# list_of_links = bodies(robot)

## here create function to read all the bodies and change their masses, center of mass, and moment of inertia to symbolic variables in order to obtain model
# for body in bodies(robot)
#     l = 0
#     if body.name != "world"
#     body.inertia.mass = inertias[l+1] # mass of the link
#     body.inertia.cross_part = SVector(l_h_x,l_h_y,l_h_z) # center of mass of the link
#     body.inertia.moment = I_h * axis * transpose(axis); #moment of mass matrix (3 by 3 SMatrix)
#     l = l+1
#     end
# end

## add flat ground or not
add_flat_ground=true

if add_flat_ground
    frame = root_frame(robot)
    ground = HalfSpace3D(Point3D(frame, 0., 0., 0.), FreeVector3D(frame, 0., 0., 1.))
    add_environment_primitive!(robot, ground)
end

## add contact points in the links

# define contact model
function default_contact_model()
    SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(0.8, 20e3, 100.))
end
contactmodel = default_contact_model()

# # create contact point in right leg_link
right_leg_link = findbody(robot,"r_leg_link")
frame_lower_link = default_frame(right_leg_link)
add_contact_point!(right_leg_link, ContactPoint(Point3D(frame_lower_link, 0.0, 0.0, -1.05), contactmodel))

# # create contact point in left leg_link
left_leg_link = findbody(robot,"l_leg_link")
frame_lower_link = default_frame(left_leg_link)
add_contact_point!(left_leg_link, ContactPoint(Point3D(frame_lower_link, 0.0, 0.0, -1.05), contactmodel))


## the state of the mechanism is a type that contains many information about the mechanism. most importantly, the angles, velocities, etc of the joints
const state = MechanismState(robot)

## visualisation and simulation

vis = MechanismVisualizer(robot, URDFVisuals(urdfpath()));
open(vis)

## set the configurations and velocities of the joints (i.e., initial angles (called configuration in julia robotics) and initial velocities):
set_configuration!(state, [1,0,0,0,0,0,0,0,0,0,-pi/4]) # starting a pass initial configuration
set_configuration!(vis, configuration(state)) ## update the configuration also in the visualiser

# ## Basic simulation is easy (but see RigidBodySim.jl for a more featureful simulator). 

# ## This simulation option has no control
# ts, qs, vs = simulate(state, 0.22, Δt = 1e-3);

# ## with control. This serves to illustrate how to create feedback controllers in the simulation

function control!(torques::AbstractVector, t, state::MechanismState)
    # rand!(torques) # for example
    l = 1
    for joint in joints(robot)
        l = l+1
        # print(l)
        # torques[velocity_range(state, joint)] .= -1.0*velocity(state, joint) # feedbacking the velocity in each joint
        # if configuration_range(state, joint) < 11:11  # feedbacking the angle in each joint. using the if because there is one more angle than torques in this atls robot, which I don't know why
        # torques[configuration_range(state, joint)] .= -1*configuration(state,joint)
        # end
        torques[velocity_range(state, joint)] .= 0 # no control
    end
end

ts, qs, vs = simulate(state, 1, control!; Δt = 1e-3);

## After which we can animate the results:
MeshCatMechanisms.animate(vis, ts, qs; realtimerate = 0.2)

##########
