WIDTH = 1.0
LENGTH = 1.0

function add_robot!(sim, params)
    MASS_BODY = params[1]
    MASS_WHEEL = params[2]
    MASS_CASTOR = params[3]

    WHEEL_RADIUS = params[4]
    LO_HI = params[5]*π
    SPEED = params[6]*π
    TORQUE = params[7]*π
    

    body = sim.send_box(x=0, y=0, z=WHEEL_RADIUS, length=LENGTH, width=WIDTH, height=WHEEL_RADIUS/2, mass=MASS_BODY, r=0.1, g=0.2, b=1, collision_group="robot")
    #camera = sim.film_body(body, method="follow")
    pos = sim.send_position_sensor(body)
    axels = Vector{Any}(undef, 6)

    left_wheel = sim.send_sphere(x=-WIDTH/2, y=-0.025, z=WHEEL_RADIUS, radius=WHEEL_RADIUS, mass=MASS_WHEEL, r=0, g=0, b=0, collision_group="robot")
    axels[1] = sim.send_hinge_joint(first_body_id=left_wheel, second_body_id=body,
                                x=-WIDTH/2, y=-0.025, z=WHEEL_RADIUS,
                                n1=-1, n2=0, n3=0,
                                position_control=false,
                                lo=-LO_HI, hi=LO_HI,
                                torque=TORQUE,
                                speed=SPEED
                                )


    right_wheel = sim.send_sphere(x=WIDTH/2, y=-0.025, z=WHEEL_RADIUS, radius=WHEEL_RADIUS, mass=MASS_WHEEL, r=0, g=0, b=0, collision_group="robot")
    axels[2] = sim.send_hinge_joint(first_body_id=right_wheel, second_body_id=body,
                                x=WIDTH/2, y=-0.025, z=WHEEL_RADIUS,
                                n1=-1, n2=0, n3=0,
                                position_control=false,
                                lo=-LO_HI, hi=LO_HI,
                                torque=TORQUE,
                                speed=SPEED
                                )

    s1 = sim.send_sphere(x=-WIDTH/2, y=LENGTH/2, z=WHEEL_RADIUS*2, radius=0.02, r=1, g=0, b=0, collision_group="none")
    sim.send_fixed_joint(body, s1)
    s2 = sim.send_sphere(x=WIDTH/2, y=LENGTH/2, z=WHEEL_RADIUS*2, radius=0.02, r=1, g=0, b=0, collision_group="none")
    sim.send_fixed_joint(body, s2)

    castor = sim.send_sphere(x=0, y=LENGTH/2, z=WHEEL_RADIUS, radius=WHEEL_RADIUS, mass=MASS_CASTOR, collision_group="robot")
    axels[3] = sim.send_hinge_joint(first_body_id=castor, second_body_id=body,
                                x=0, y=LENGTH/2, z=WHEEL_RADIUS,
                                n1=1, n2=0, n3=0,
                                position_control=false,
                                speed=SPEED
                                )

    sim.assign_collision("robot", "env")
    sim.assign_collision("env", "env")


    light_sensors = Vector{Any}(undef, 2)
    light_sensors[1] = sim.send_light_sensor(s1)
    light_sensors[2] = sim.send_light_sensor(s2)

    sensor_neurons = Vector{Any}(undef, 2)
    sensor_neurons[1] = sim.send_sensor_neuron(light_sensors[1])
    sensor_neurons[2] = sim.send_sensor_neuron(light_sensors[2])

    motor_neurons = Vector{Any}(undef, 2)
    motor_neurons[1] = sim.send_motor_neuron(axels[1], alpha=0)
    motor_neurons[2] = sim.send_motor_neuron(axels[2], alpha=0)

    sim.send_synapse(sensor_neurons[1], motor_neurons[2], weight=1.0)
    sim.send_synapse(sensor_neurons[2], motor_neurons[1], weight=1.0)

    p1 = sim.send_position_sensor(body)
    p2 = sim.send_position_sensor(castor)


    return (p1, p2)
end