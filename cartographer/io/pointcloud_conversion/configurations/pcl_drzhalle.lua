

options = {
    generateCubicPointcloud = false,

    --pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",
    pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09-part-part.ply",

    uniformDownSample = true,
    sampleRateUniformDownSample = 10,

    voxelDownSample = true,
    voxelSizeVoxelDownSample = 0.03,

    removeRadiusOutliers = true,
    sphereSizeRadiusOutliers = 0.3,
    neighborsInSphereRadiusOutlier = 100,

    cutRoofZAxis = false,
    cutoffSize = 3.0,

    normalOrientationNearestNeighbours = 100,

    absoluteVoxelSize = 0.1,
    absoluteTruncationDistance = 0.3,

    imageSliceIndex = -55,






}

return options