local Module = {}
-- 4thAxis

local function CalculateOverlap(Min1, Max1, Min2, Max2)
	return math.max(0, math.min(Max1, Max2) - math.max(Min1, Min2))
end
function EntireCellInsideRadius(Origin, CellX, CellZ, CellSize, Radius)
	local Distance = math.sqrt((Origin.Position.X - CellX * CellSize)^2 + (Origin.Position.Z - CellZ * CellSize)^2)
	return Distance + CellSize <= Radius
end

function EntireAABBInsideRadius(AABB, CircleCenter, Radius)
	local ClosestX = math.max(AABB.MinX, math.min(CircleCenter.X, AABB.MaxX))
	local ClosestZ = math.max(AABB.MinZ, math.min(CircleCenter.Z, AABB.MaxZ))

	local Distance = math.sqrt((ClosestX - CircleCenter.X)^2 + (ClosestZ - CircleCenter.Z)^2)
	return Distance + math.max(AABB.MaxX - AABB.MinX, AABB.MaxZ - AABB.MinZ)/2 <= Radius
end

local function VisualizeClusterAABB(ClusterGroup, MaxX, MinX, MaxZ, MinZ, Subdivision)
	local Height = 5 

	local AABBVisual = Instance.new("Part")
	AABBVisual.Size = Vector3.new(MaxX - MinX, Height, MaxZ - MinZ)
	AABBVisual.Position = Vector3.new((MaxX + MinX)/2, Height/2, (MaxZ + MinZ)/2)
	AABBVisual.Anchored = true
	AABBVisual.Parent = workspace
	AABBVisual.BrickColor = BrickColor.Black()
	AABBVisual.Transparency = 1-(1/Subdivision) -- Quick maffs

	ClusterGroup.AABBVisual = AABBVisual
end

local function VisualizeAABBGrid(ClusterRegions, ClusterRegionsCellSize, Subdivisons)
	local Height = 5 
	local Size = Vector3.new(ClusterRegionsCellSize, Height, ClusterRegionsCellSize)

	for X, Rows in ClusterRegions do
		for Z in Rows do
			local CellArea = Instance.new("Part")
			CellArea.Size = Size
			CellArea.Position = Vector3.new(X * ClusterRegionsCellSize, Height, Z * ClusterRegionsCellSize)
			CellArea.Anchored = true
			CellArea.Parent = workspace
			CellArea.Transparency = 1-(1/Subdivisons) -- Quick maffs
			CellArea.BrickColor = BrickColor.new("Pink")
		end
	end
end

---------------------------------------------------------------------
--- Cluster Tree constructer, requires linear time: O(nm), (m<<n) --- 
---------------------------------------------------------------------
local VisualizeAABBs = false
local VisualizeCells = false
function Module.NewClusterTree(Points, Epsilon, MinSamples, Subdivisions)
	local Grid = {} -- TODO: Implement GriT-DBSCAN, instead of this approach as this has not been proved to be linear to dataset
	for Index, Point in Points do
		local X = math.floor(Point.Position.X/Epsilon)
		local Z = math.floor(Point.Position.Z/Epsilon)
		Grid[X] = Grid[X] or {}; if not Grid[X][Z] then Grid[X][Z] = {} end
		table.insert(Grid[X][Z], Index)
	end

	local Labels, ClusterID = {}, 0
	for RawIndex, Point in Points do
		if Labels[RawIndex] then continue end

		local Neighbors = {}
		local XCell = math.floor(Point.Position.X/Epsilon)
		local ZCell = math.floor(Point.Position.Z/Epsilon)
		for NearX = XCell-1, XCell+1 do
			for NearZ = ZCell-1, ZCell+1 do
				if Grid[NearX] and Grid[NearX][NearZ] then 
					for _, NearPoint in Grid[NearX][NearZ] do
						local Diff=Points[NearPoint].Position-Point.Position
						if Diff:Dot(Diff) <= Epsilon*Epsilon then
							table.insert(Neighbors, NearPoint)
						end
					end
				end
			end
		end

		if #Neighbors < MinSamples then
			Labels[RawIndex] = -1 
		else
			ClusterID = ClusterID + 1
			Labels[RawIndex] = ClusterID 
			local Iterations = 1; while Iterations<= #Neighbors do
				local NeighborIndex = Neighbors[Iterations]
				if Labels[NeighborIndex] then Iterations = Iterations + 1 continue end
				Labels[NeighborIndex] = ClusterID 

				local CurrentPoint = Points[NeighborIndex]
				local NeighborNeighbors = {}

				local XCell = math.floor(CurrentPoint.Position.X/Epsilon)
				local ZCell = math.floor(CurrentPoint.Position.Z/Epsilon)
				for NearX = XCell-1, XCell+1 do
					for NearZ = ZCell-1, ZCell+1 do
						if Grid[NearX] and Grid[NearX][NearZ] then 
							for _, NearPoint in Grid[NearX][NearZ] do
								local Diff=Points[NearPoint].Position-CurrentPoint.Position
								if Diff:Dot(Diff) <= Epsilon*Epsilon then
									table.insert(NeighborNeighbors, NearPoint)
								end
							end
						end
					end
				end

				if #NeighborNeighbors >= MinSamples then
					for _, NeighborNeighborsIndex in NeighborNeighbors do
						table.insert(Neighbors, NeighborNeighborsIndex) 
					end
				end
			end

			Iterations = Iterations + 1
		end
	end

	local ClusterGroups = {}
	for Index, ClusterID in Labels do
		if ClusterGroups[ClusterID] then
			table.insert(ClusterGroups[ClusterID].Cluster, Points[Index])
		else
			ClusterGroups[ClusterID] = {Cluster = {Points[Index]}}
		end
	end

	for ClusterID, ClusterGroup in ClusterGroups do
		if ClusterID == -1 or ClusterGroup.Cluster == nil then continue end

		local MinX, MinZ = math.huge, math.huge
		local MaxX, MaxZ = -math.huge, -math.huge
		for _, Point in ClusterGroup.Cluster do
			local X, Z = Point.Position.X, Point.Position.Z
			MinX, MaxX = math.min(MinX, X), math.max(MaxX, X)
			MinZ, MaxZ = math.min(MinZ, Z), math.max(MaxZ, Z)
		end

		if VisualizeAABBs then
			VisualizeClusterAABB(ClusterGroup, MaxX, MinX, MaxZ, MinZ, Subdivisions)
		end
		ClusterGroup.AABB = {MinX = MinX, MaxX = MaxX, MinZ = MinZ, MaxZ = MaxZ}
	end

	local MaxAABBExtents, LargestClusterAABB = 1, next(ClusterGroups); for ClusterID, ClusterGroup in ClusterGroups do
		if ClusterID == -1 or ClusterGroup.Cluster == nil then continue end
		local Volume = math.max(math.abs(ClusterGroup.AABB.MaxX - ClusterGroup.AABB.MinX), math.abs(ClusterGroup.AABB.MaxZ - ClusterGroup.AABB.MinZ))
		if Volume > MaxAABBExtents then
			MaxAABBExtents = Volume
			LargestClusterAABB = ClusterGroup.AABB
		end
	end

	ClusterGroups.AABBGridCellSize = MaxAABBExtents
	ClusterGroups.AABBGrid = {}

	for ClusterID, ClusterGroup in ClusterGroups do
		if ClusterID == -1 or ClusterID == "AABBGrid" or ClusterID == "AABBGridCellSize" then continue end

		local XCell = math.floor(((ClusterGroup.AABB.MinX + ClusterGroup.AABB.MaxX)/2)/ClusterGroups.AABBGridCellSize)
		local ZCell = math.floor(((ClusterGroup.AABB.MinZ + ClusterGroup.AABB.MaxZ)/2)/ClusterGroups.AABBGridCellSize)
		ClusterGroups.AABBGrid[XCell] = ClusterGroups.AABBGrid[XCell] or {}
		ClusterGroups.AABBGrid[XCell][ZCell] = ClusterGroups.AABBGrid[XCell][ZCell] or {}

		table.insert(ClusterGroups.AABBGrid[XCell][ZCell], ClusterGroup)
	end

	if ClusterGroups[-1] then
		for _, UnclusteredPoint in ClusterGroups[-1].Cluster do
			local XCell = math.floor((UnclusteredPoint.Position.X)/ClusterGroups.AABBGridCellSize)
			local ZCell = math.floor((UnclusteredPoint.Position.Z)/ClusterGroups.AABBGridCellSize)
			ClusterGroups.AABBGrid[XCell] = ClusterGroups.AABBGrid[XCell] or {}

			if ClusterGroups.AABBGrid[XCell][ZCell] then 
				table.insert(ClusterGroups.AABBGrid[XCell][ZCell][1].Cluster, UnclusteredPoint)
			else
				ClusterGroups.AABBGrid[XCell][ZCell] = {{["Cluster"] = {UnclusteredPoint}}}
			end
		end 
	end

	if VisualizeCells then VisualizeAABBGrid(ClusterGroups.AABBGrid, ClusterGroups.AABBGridCellSize, Subdivisions) end

	if Subdivisions and Subdivisions>1 then -- TODO: Balance sub clustering
		for ClusterID, ClusterGroup in ClusterGroups do
			if ClusterID == -1 or ClusterID == "AABBGrid" or ClusterID == "AABBGridCellSize" then continue end
			ClusterGroup.SubCluster = Module.NewClusterTree(ClusterGroup.Cluster, Epsilon/2, MinSamples, Subdivisions-1) -- O(nm), where m will always be << n
		end
	end

	return ClusterGroups
end

-------------------------------------------------------------------------------------------------
--- Query all points in a OBB-like Radius, falls to a near-lookup when Radius is not provided ---
--- Currently, ~avg Θ(log n), ~amortized O(n) (tree is unbalanced as of now)		      ---
-------------------------------------------------------------------------------------------------

function Module.Query(ClusterTreeTable, Point, Radius, Near, AABBsVisited)
	Radius = Radius or 1
	Near = Near or {}
	AABBsVisited = AABBsVisited or {}

	local AABBGrid = ClusterTreeTable.AABBGrid
	local XCell = math.floor(Point.Position.X/ClusterTreeTable.AABBGridCellSize)
	local ZCell = math.floor(Point.Position.Z/ClusterTreeTable.AABBGridCellSize)
	if not AABBGrid[XCell] or not AABBGrid[XCell][ZCell] then return Near end

	local NeighborRadius = math.ceil(Radius/ClusterTreeTable.AABBGridCellSize)
	for NearX = XCell-NeighborRadius, XCell+NeighborRadius do
		for NearZ = ZCell-NeighborRadius, ZCell+NeighborRadius do
			if not AABBGrid[NearX] or not AABBGrid[NearX][NearZ] then continue end

			local OtherCell = AABBGrid[NearX][NearZ]
			if EntireCellInsideRadius(Point, NearX, NearZ, ClusterTreeTable.AABBGridCellSize, Radius) then
				for _, ClusterGroup in OtherCell do
					for _, Point in ClusterGroup.Cluster do
						table.insert(Near, Point)
					end
				end
			else
				for _, ClusterGroup in OtherCell do
					if AABBsVisited[ClusterGroup] then AABBsVisited[ClusterGroup.SubCluster or ClusterGroup] = true continue end -- Continue to mark the entire lineage to avoid extra work.
					if not ClusterGroup.AABB then continue end

					if EntireAABBInsideRadius(ClusterGroup.AABB, Point.Position, Radius) then
						AABBsVisited[ClusterGroup.SubCluster or ClusterGroup] = true -- Mark by next sub-group
						for _, Point in ClusterGroup.Cluster do
							table.insert(Near, Point)
						end
					else
						if ClusterGroup.SubCluster then
							local OverlapX = CalculateOverlap(Point.Position.X - Radius, Point.Position.X + Radius, ClusterGroup.AABB.MinX, ClusterGroup.AABB.MaxX)
							local OverlapZ = CalculateOverlap(Point.Position.Z - Radius, Point.Position.Z + Radius, ClusterGroup.AABB.MinZ, ClusterGroup.AABB.MaxZ)
							local IntersectionArea = OverlapX * OverlapZ
							local TotalCellArea = (ClusterGroup.AABB.MaxX - ClusterGroup.AABB.MinX) * (ClusterGroup.AABB.MaxZ - ClusterGroup.AABB.MinZ)
							local SubQueryRadius = Radius * math.sqrt(IntersectionArea/TotalCellArea)*2

							Module.Query(ClusterGroup.SubCluster, Point, SubQueryRadius, Near)
						end
					end
				end
			end
		end
	end

	return Near
end

return Module
