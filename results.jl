using CSV, DataFrames
using BSON: @load

LENGTH = 1.0
WHEEL_RADIUS = 0.260017
LIGHT_RADIUS = 0.2

R1 = Matrix{Float64}(undef, 1, 6561)
R2 = Matrix{Float64}(undef, 1, 6561)
m = []
S = []
for i in 1:6561
	global R1, R2, m
	dir = "results_low/$(lpad(i, 4, '0'))"
	@load "$dir/raw.bson" raw
	@load "$dir/vs.bson" vs
	idx1 = findall(f -> f <= (LENGTH/2 + WHEEL_RADIUS + LIGHT_RADIUS)+eps(Float16), raw)
	idx0 = findall(f -> f > (LENGTH/2 + WHEEL_RADIUS + LIGHT_RADIUS)+eps(Float16), raw)
	raw[idx1] .= 1
	raw[idx0] .= 0
	Ao = count(f -> f >= 4, sum(raw, dims=1))
	Aθ = count(f -> f >= 1, sum(raw, dims=1))
	At = count(f -> true, sum(raw, dims=1))
	r1 = Ao/At
	r2 = 0
	if Aθ > 0
		r2 = Ao/Aθ
	end

	R1[1, i] = r1
	R2[1, i] = r2
	push!(m, vs)
	push!(S, sum(raw, dims=1)[1, :, :])
end

CSV.write("data_low/m1hist.csv",  DataFrame(R1), writeheader=false)
CSV.write("data_low/m2hist.csv",  DataFrame(R2), writeheader=false)


i1 = findfirst(x -> x == ([-0.5, 0.5], [0.5, 0.5]), m)
i2 = findmax(R1)[2][2]
i3 = findmax(R2)[2][2]
CSV.write("data_low/can.csv",  DataFrame(S[i1]), writeheader=false)
CSV.write("data_low/m1.csv",  DataFrame(S[i2]), writeheader=false)
CSV.write("data_low/m2.csv",  DataFrame(S[i3]), writeheader=false)


R1 = Matrix{Float64}(undef, 1, 6561)
R2 = Matrix{Float64}(undef, 1, 6561)
m = []
S = []
for i in 1:6561
	global R1, R2, m
	dir = "results_high/$(lpad(i, 4, '0'))"
	if Base.Filesystem.isdir(dir)
		@load "$dir/raw.bson" raw
		@load "$dir/vs.bson" vs
		idx1 = findall(f -> f <= (LENGTH/2 + WHEEL_RADIUS + LIGHT_RADIUS)+eps(Float16), raw)
		idx0 = findall(f -> f > (LENGTH/2 + WHEEL_RADIUS + LIGHT_RADIUS)+eps(Float16), raw)
		raw[idx1] .= 1
		raw[idx0] .= 0
		Ao = count(f -> f >= 4, sum(raw, dims=1))
		Aθ = count(f -> f >= 1, sum(raw, dims=1))
		At = count(f -> true, sum(raw, dims=1))
		r1 = Ao/At
		r2 = 0
		if Aθ > 0
			r2 = Ao/Aθ
		end
	else
		r1 = 0
		r2 = 0
		vs = ([0., 0.], [0., 0.])
		raw = rand(4, 121, 121)
	end

	R1[1, i] = r1
	R2[1, i] = r2
	push!(m, vs)
	push!(S, sum(raw, dims=1)[1, :, :])
end

CSV.write("data_high/m1hist.csv",  DataFrame(R1), writeheader=false)
CSV.write("data_high/m2hist.csv",  DataFrame(R2), writeheader=false)


i1 = findfirst(x -> x == ([-0.5, 0.5], [0.5, 0.5]), m)
i2 = findmax(R1)[2][2]
i3 = findmax(R2)[2][2]
CSV.write("data_high/can.csv",  DataFrame(S[i1]), writeheader=false)
CSV.write("data_high/m1.csv",  DataFrame(S[i2]), writeheader=false)
CSV.write("data_high/m2.csv",  DataFrame(S[i3]), writeheader=false)

println(m[i2])
println(m[i3])