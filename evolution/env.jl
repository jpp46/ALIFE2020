using LinearAlgebra

function add_env!(sim; idx=5)
	if idx == 1
		coords = [4, 4]
	elseif idx == 2
		coords = [-4, 4]
	elseif idx == 3
		coords = [-4, -4]
	elseif idx == 4
		coords = [4, -4]
	else
		coords = rand(2)
		coords ./= norm(coords)
		coords .*= 6
	end
	
	light_source = sim.send_sphere(x=coords[1], y=coords[2], z=0.201, radius=0.2,
	                           r=0.9921568627450981, g=0.7215686274509804, b=0.07450980392156863,
	                           collision_group="env")
	sim.send_fixed_joint(light_source, -1)
	light_source = sim.send_light_source(light_source)
	return coords
end