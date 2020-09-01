using PyCall
using BlackBoxOptim
using BSON: @save, @load

include("evorobot.jl")
include("env.jl")

pyrosim = PyNULL()
copy!(pyrosim, pyimport("pyrosim"))

function new_sim(pp=false, pb=true, t=1000)
	return pyrosim.Simulator(play_paused=pp, play_blind=pb, eval_time=t)
end

function dist(a, b)
	return norm(a.-b)
end

function fitness(params; pp=false, pb=true, t=2500)
	sims = []
	lights = []
	sensors = []
	for i in 1:4
		sim = new_sim(pp, pb, t)
		push!(sims, sim)
		push!(lights, add_env!(sim; idx=i))
		push!(sensors, add_robot!(sim, params))
		sims[i].start()
	end

	total = 0.0
	for i in 1:4
		sim = sims[i]
		sim.wait_to_finish()
		
		sensor_data = Vector{Any}()
		x = sim.get_sensor_data(sensors[i][1], svi=0)
		y = sim.get_sensor_data(sensors[i][1], svi=1)
		light = lights[i]
		for j in 1:length(x)
			push!(sensor_data, dist([x[j], y[j]], light))
		end
		r1 = minimum(sensor_data)

		sensor_data = Vector{Any}()
		x = sim.get_sensor_data(sensors[i][2], svi=0)
		y = sim.get_sensor_data(sensors[i][2], svi=1)
		light = lights[i]
		for j in 1:length(x)
			push!(sensor_data, dist([x[j], y[j]], light))
		end
		r2 = minimum(sensor_data)
		total += Float64(min(r1, r2))
	end
	return (total, sum(params[5:7]))
end

ranges = [(0.001, 1000.0), (0.001, 1000.0), (0.001, 1000.0),
		  (0.075, 0.3), (0.25, 10.0), (0.25, 10.0), (0.25, 10.0)]

res = bboptimize(fitness; SearchRange=ranges, NumDimensions=7, Method=:borg_moea,
						  FitnessScheme=ParetoFitnessScheme{2}(is_minimizing=true),
						  ϵ=0.05, PopulationSize=200,
            			  MaxSteps=20000, TraceInterval=10.0, TraceMode=:verbose)

println(best_fitness(res))
params = best_candidate(res)
fitness(params; pp=false, pb=false)
@save "morphologies.bson" res

#fitness([1000.0, 1000.0, 1000.0, 1000.0, 0.2, 5π, 5π, 5π, 5π]; pp=false, pb=false)
