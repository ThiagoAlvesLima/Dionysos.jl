using RigidBodyDynamics
using RigidBodyDynamics.Contact
using Plots
using MeshCatMechanisms

## describe path to urdf file
packagepath() = joinpath(@__DIR__, "..","..", "deps")
urdfpath() = joinpath(packagepath(), "BipedRobot", "doublependulum.urdf")

## load mechanism 'doublependulum' from the URDF file
doublependulum = parse_urdf(Float64, urdfpath())

## add flat ground or not
add_flat_ground=true

if add_flat_ground
    frame = root_frame(doublependulum)
    ground = HalfSpace3D(Point3D(frame, 0., 0., 0.), FreeVector3D(frame, 0., 0., 1.))
    add_environment_primitive!(doublependulum, ground)
end

## add contact points in the links

# define contact model
function default_contact_model()
    SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(0.8, 20e3, 100.))
end
contactmodel = default_contact_model()

# create contact point in lower_link
lower_link_body = findbody(doublependulum,"lower_link")
frame_lower_link = default_frame(lower_link_body)
add_contact_point!(lower_link_body, ContactPoint(Point3D(frame_lower_link, 0.0, 0.0, 2.0), contactmodel))

# create contact point in upper_link
# upper_link_body = findbody(doublependulum,"upper_link")
# frame_upper_link = default_frame(upper_link_body)
# add_contact_point!(upper_link_body, ContactPoint(Point3D(frame_upper_link, 0.0, 0.0, 0.0), contactmodel))

# foot = findbody(mechanism, "$(first(string(side)))_foot")
#             frame = default_frame(foot)
#             z = -0.07645

#             # heel
#             add_contact_point!(foot, ContactPoint(Point3D(frame, -0.0876, flipsign_if_right(0.066, side), z), contactmodel))
#             add_contact_point!(foot, ContactPoint(Point3D(frame, -0.0876, flipsign_if_right(-0.0626, side), z), contactmodel))

## the state of the mechanism is a type that contains many information about the mechanism. most importantly, the angles, velocities, etc of the joints
const state = MechanismState(doublependulum)

## create visualisation of the mechanism 
vis = MechanismVisualizer(doublependulum, URDFVisuals(urdfpath()));
open(vis) ## open in the browser

## set the configurations and velocities of the joints (i.e., initial angles (called configuration in julia robotics) and velocities):
set_configuration!(state, [π,0.]) # up straight initial configuration
set_configuration!(vis, configuration(state)) ## update the configuration also in the visualiser

## Basic simulation is easy (but see RigidBodySim.jl for a more featureful simulator):

ts, qs, vs = simulate(state, 10., Δt = 1e-3);

## After which we can animate the results:
MeshCatMechanisms.animate(vis, ts, qs)

## Or plot them using e.g. Plots.jl:
# shoulder_angles = collect(q[1] for q in qs)
# plot(ts, shoulder_angles, 
#     xlabel = "Time [s]", 
#     ylabel = "Angle [rad]", 
#     label = "Shoulder")
