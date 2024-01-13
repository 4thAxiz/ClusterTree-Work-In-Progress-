local ClusterTree = {}
---------------------------------------------------------------------
--- Cluster Tree constructer, requires linear time: O(nm), (m<<n) --- 
---------------------------------------------------------------------
function ClusterTree.new(Points, Epsilon, MinSamples, Subdivisons)
	------------------------------------------------------------
	--- Cluster Points, using a custom O(n) DBSCAN approach  ---
	------------------------------------------------------------
	local Grid = {} -- Will not be factored into space complexity, simply an auxilary  
	for Index, Point in Points do
		local X = math.floor(Point.Position.X/Epsilon)
		local Z = math.floor(Point.Position.Z/Epsilon)
		Grid[X] = Grid[X] or {}; if not Grid[X][Z] then Grid[X][Z] = {} end
		table.insert(Grid[X][Z], Index)
	end

	local Labels, ClusterID = {}, 0
	for RawIndex, Point in Points do
		if Labels[RawIndex] then continue end
		---------------------------------------------------
		--- Finding Neighbors, looking for core points  ---
		---------------------------------------------------
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
			---------------------------------------------------
			--- 			Expand The Cluster 				---
			---------------------------------------------------
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
	------------------------------------------------------------
	---		Group all assigned points into formal clusters   ---
	------------------------------------------------------------
	local Clusters = {}
	for Index, ClusterID in Labels do
		if Clusters[ClusterID] then
			table.insert(Clusters[ClusterID].Cluster, Points[Index])
		else
			Clusters[ClusterID] = {Cluster = {Points[Index]}}
		end
	end
	
	------------------------------------------------------------
	---		 Find Cluster axis-aligned bounding boxes     	 ---
	------------------------------------------------------------
	for ClusterID, ClusterCell in Clusters do
		if ClusterID == -1 or ClusterCell.Cluster == nil then continue end

		local MinX, MinZ = math.huge, math.huge
		local MaxX, MaxZ = -math.huge, -math.huge
		for _, Point in ClusterCell.Cluster do
			local X, Z = Point.Position.X, Point.Position.Z
			MinX, MaxX = math.min(MinX, X), math.max(MaxX, X)
			MinZ, MaxZ = math.min(MinZ, Z), math.max(MaxZ, Z)
		end

		ClusterCell.AABB = {MinX = MinX, MaxX = MaxX, MinZ = MinZ, MaxZ = MaxZ}
	end

	local MaxAABBExtents = 1; for ClusterID, ClusterCell in Clusters do
		if ClusterID == -1 or ClusterCell.Cluster == nil then continue end

		local SizeX = ClusterCell.AABB.MaxX - ClusterCell.AABB.MinX
		local SizeZ = ClusterCell.AABB.MaxZ - ClusterCell.AABB.MinZ
		MaxAABBExtents = math.max(MaxAABBExtents, math.abs(SizeX), math.abs(SizeZ))
	end
	------------------------------------------------------------
	---		 Track AABBs as a grid-hierachy set up     	 	 ---
	------------------------------------------------------------
	MaxAABBExtents/=2; Clusters.ClusterRegionsCellSize = MaxAABBExtents
	Clusters.ClusterRegions = {}
	for ClusterID, ClusterCell in Clusters do
		if ClusterID == -1 or ClusterID == "ClusterRegions" or ClusterID == "ClusterRegionsCellSize" then continue end

		local XCell = math.floor(((ClusterCell.AABB.MinX + ClusterCell.AABB.MaxX)/2)/MaxAABBExtents)
		local ZCell = math.floor(((ClusterCell.AABB.MinZ + ClusterCell.AABB.MaxZ)/2)/MaxAABBExtents)
		Clusters.ClusterRegions[XCell] = Clusters.ClusterRegions[XCell] or {}
		Clusters.ClusterRegions[XCell][ZCell] = Clusters.ClusterRegions[XCell][ZCell] or {}

		table.insert(Clusters.ClusterRegions[XCell][ZCell], ClusterCell)
	end

	if Clusters[-1] then
		for _, UnclusteredPoint in Clusters[-1].Cluster do
			local XCell = math.floor(UnclusteredPoint.Position.X/Clusters.ClusterRegionsCellSize)
			local ZCell = math.floor(UnclusteredPoint.Position.Z/Clusters.ClusterRegionsCellSize)
			Clusters.ClusterRegions[XCell] = Clusters.ClusterRegions[XCell] or {}

			if Clusters.ClusterRegions[XCell][ZCell] then 
				table.insert(Clusters.ClusterRegions[XCell][ZCell][1].Cluster, UnclusteredPoint)
			else
				Clusters.ClusterRegions[XCell][ZCell] = { {["Cluster"]={UnclusteredPoint}} }
			end
		end 
	end
	------------------------------------------------------------
	---	 	Sub-divide the clusters into sub-clusters 		 ---
	------------------------------------------------------------
	if Subdivisons and Subdivisons>1 then -- TODO: Balance sub clustering
		for ClusterID, ClusterCell in Clusters do
			if ClusterID == -1 or ClusterID == "ClusterRegions" or ClusterID == "ClusterRegionsCellSize" then continue end
			ClusterCell.SubCluster = ClusterTree.new(ClusterCell.Cluster, Epsilon/2, MinSamples, Subdivisons-1) -- O(nm), where m will always be << n
		end
	end

	return Clusters
end

-------------------------------------------------------------------------------------------------
--- Query all points in a OBB-like radius, falls to a near-lookup when radius is not provided ---
--- Currently, ~avg Θ(log n), ~amortized O(n) (tree is unbalanced as of now)		      ---
-------------------------------------------------------------------------------------------------
function ClusterTree.Query(ClusterTreeTable, Point, Radius, Near)
	Radius = Radius or 1
	Near = Near or {}

	local ClusterRegions = ClusterTreeTable.ClusterRegions
	local XCell = math.floor(Point.Position.X/ClusterTreeTable.ClusterRegionsCellSize)
	local ZCell = math.floor(Point.Position.Z/ClusterTreeTable.ClusterRegionsCellSize)
	if not ClusterRegions[XCell] then return Near end
	
	if ClusterTreeTable.ClusterRegionsCellSize>Radius then -- Adapative searching technique
		if not ClusterRegions[XCell][ZCell] then return Near end -- Nothing in this generious radius
		for ClusterID, ClusterCell in ClusterRegions[XCell][ZCell] do
			if ClusterCell.SubCluster then
				for SubClusterID, SubClusterCell in ClusterCell.SubCluster do
					if SubClusterID == "ClusterRegions" or SubClusterID == "ClusterRegionsCellSize" then continue end

					if SubClusterCell.SubCluster then-- Add all children of the overlapping sub-cluster to the result
						ClusterTree.Query(SubClusterCell.SubCluster, Point, Radius, Near)
					else
						if SubClusterCell.Cluster then -- Add all children of the leaf sub-cluster to the result
							for _, ClusteredObject in SubClusterCell.Cluster do
								local Diff=ClusteredObject.Position-Point.Position
								if Diff:Dot(Diff)<Radius*Radius then
									table.insert(Near, ClusteredObject)
								end
							end
						end
					end
				end
			else
				for _, ClusteredObject in ClusterCell.Cluster do
					local Diff=ClusteredObject.Position-Point.Position
					if Diff:Dot(Diff)<Radius*Radius then
						table.insert(Near, ClusteredObject)
					end
				end
			end
		end
	else
		local Cell = ClusterRegions[XCell] and ClusterRegions[XCell][ZCell]
		if Cell then
			for ClusterID, ClusterCell in Cell do
				for _, Point in ClusterCell.Cluster do
					table.insert(Near, Point)
				end
			end
		end

		if ClusterTreeTable.ClusterRegionsCellSize == Radius then -- Your lucky day
			return Near
		end 

		local NeighborRadius = math.ceil(Radius/ClusterTreeTable.ClusterRegionsCellSize)
		for NearX = XCell-NeighborRadius, XCell+NeighborRadius do
			for NearZ = ZCell-NeighborRadius, ZCell+NeighborRadius do
				if not ClusterRegions[NearX] or not ClusterRegions[NearX][NearZ] or NearX == XCell and NearZ == ZCell then continue end -- Only include neighbor existing cells 
				
				local DistanceX = (NearX + 0.5) * ClusterTreeTable.ClusterRegionsCellSize - Point.Position.X
				local DistanceZ = (NearZ + 0.5) * ClusterTreeTable.ClusterRegionsCellSize - Point.Position.Z
				local DistanceSquared = DistanceX * DistanceX + DistanceZ * DistanceZ

				if DistanceSquared < Radius * Radius then -- The cell falls inside the query radius
					for ClusterID, ClusterCell in  ClusterRegions[NearX][NearZ] do
						for _, ClusteredObject in ClusterCell.Cluster do
							local Diff=ClusteredObject.Position-Point.Position
							if Diff:Dot(Diff)<Radius*Radius then
								table.insert(Near, ClusteredObject)
							end
						end
					end
				else -- The cell falls outside the query radius
					for ClusterID, ClusterCell in ClusterRegions[NearX][NearZ] do
						if ClusterCell.SubCluster then
							for SubClusterID, SubClusterCell in ClusterCell.SubCluster do
								if SubClusterID == "ClusterRegions" or SubClusterID == "ClusterRegionsCellSize" then continue end

								if SubClusterCell.SubCluster then-- Add all children of the overlapping sub-cluster to the result
									local CellMinX, CellMaxX = NearX * ClusterTreeTable.ClusterRegionsCellSize, (NearX + 1) * ClusterTreeTable.ClusterRegionsCellSize
									local CellMinZ, CellMaxZ = NearZ * ClusterTreeTable.ClusterRegionsCellSize, (NearZ + 1) * ClusterTreeTable.ClusterRegionsCellSize
									local ClosestX, ClosestZ = math.max(CellMinX, math.min(CellMaxX, Point.Position.X)), math.max(CellMinZ, math.min(CellMaxZ, Point.Position.Z))

									local SubDistanceX, SubDistanceZ = ClosestX - Point.Position.X, ClosestZ - Point.Position.Z
									local SubQueryRadius = Radius - math.sqrt(SubDistanceX * SubDistanceX + SubDistanceZ * SubDistanceZ)

									ClusterTree.Query(SubClusterCell.SubCluster, Point, SubQueryRadius, Near)
								else
									if SubClusterCell.Cluster then -- Add all children of the leaf sub-cluster to the result
										for _, ClusteredObject in SubClusterCell.Cluster do
											local Diff=ClusteredObject.Position-Point.Position
											if Diff:Dot(Diff)<Radius*Radius then
												table.insert(Near, ClusteredObject)
											end
										end
									end
								end
							end
						else
							for _, ClusteredObject in ClusterCell.Cluster do
								local Diff=ClusteredObject.Position-Point.Position
								if Diff:Dot(Diff)<Radius*Radius then
									table.insert(Near, ClusteredObject)
								end
							end
						end
					end
				end
			end
		end
	end

	return Near
end
