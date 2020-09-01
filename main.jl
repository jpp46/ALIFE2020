using PyCall
using BSON: @save

arg = parse(Int, ARGS[1])
num = parse(Int, ARGS[2])
st = div(6561, num)
rng = (arg*st+1):(arg*st)+st

include("robot.jl")
include("env.jl")

pyrosim = PyNULL()
copy!(pyrosim, pyimport("pyrosim"))

function new_sim(pp=false, pb=true, t=1000)
	return pyrosim.Simulator(play_paused=pp, play_blind=pb, eval_time=t, xyz=[0, 0, 4], hpr=[0, -90, 0])
end

function dist(a, b)
	return norm(a.-b)
end

function fitness(wts, v1, v2; pp=false, pb=true, t=2500)
	sims = []
	lights = []
	sensors = []
	for i in 1:4
		sim = new_sim(pp, pb, t)
		push!(sims, sim)
		push!(lights, add_env!(sim; idx=i))
		push!(sensors, add_robot!(sim, wts, v1, v2))
		sims[i].start()
	end

	fits = []
	for i in 1:4
		sim = sims[i]
		sim.wait_to_finish()
		
		sensor_data = Vector{Any}()
		x = sim.get_sensor_data(sensors[i], svi=0)
		y = sim.get_sensor_data(sensors[i], svi=1)
		light = lights[i]
		for j in 1:length(x)
			push!(sensor_data, dist([x[j], y[j]], light))
		end
		push!(fits, minimum(sensor_data))
	end
	return [fits...]
end

#CN fitness([1.0, 1.0], [-0.5, 0.5], [0.5, 0.5]; pp=true, pb=false, t=2500)
#M1 fitness([1.0, 1.0], [-0.125, -0.5], [0.375, 0.5]; pp=true, pb=false, t=2500)
#M2 fitness([1.0, 1.0], [-0.125, -0.5], [0.375, 0.5]; pp=true, pb=false, t=2500)

r = range(-LENGTH/2, stop=LENGTH/2, length=9)
vs = [[x, y] for x=r, y=r]
r = range(-1.0, stop=1.0, length=121)
wts = [[x, y] for x=r, y=r]

n = 1
for v1 in vs, v2 in vs
	global n, rng
	if n in rng
		dir = "results/$(lpad(n, 4, '0'))"
		if !Base.Filesystem.isdir(dir)
			raw = Array{Float64, 3}(undef, 4, 121, 121)
			for i in CartesianIndices(wts)
				fits = fitness(wts[i], v1, v2)
				raw[1, i] = fits[1]
				raw[2, i] = fits[2]
				raw[3, i] = fits[3]
				raw[4, i] = fits[4]
			end
			Base.Filesystem.mkdir("$dir")
			@save "$dir/raw.bson" raw
			@save "$dir/vs.bson" vs=(v1, v2)

			idx1 = findall(f -> f <= (LENGTH/2 + WHEEL_RADIUS + LIGHT_RADIUS)+eps(Float16), raw)
			idx0 = findall(f -> f > (LENGTH/2 + WHEEL_RADIUS + LIGHT_RADIUS)+eps(Float16), raw)
			raw[idx1] .= 1
			raw[idx0] .= 0
			@save "$dir/W.bson" W=raw
			
			Ao = count(f -> f >= 4, sum(raw, dims=1))
			Aθ = count(f -> f >= 1, sum(raw, dims=1))
			At = count(f -> true, sum(raw, dims=1))

			@save "$dir/r1.bson" r1=Ao/At
			r2 = 0
			if Aθ > 0
				r2 = Ao/Aθ
			end
			@save "$dir/r2.bson" r2
		end
	end
	n+=1
end

