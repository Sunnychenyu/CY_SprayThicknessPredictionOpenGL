#include <WorkpieceCore/WorkpieceModel.h>

namespace sprayworkpiece
{
    bool WorkpieceModel::empty() const
    {
        return samples.empty();
    }

    size_t WorkpieceModel::sampleCount() const
    {
        return samples.size();
    }

    void WorkpieceModel::addRegion(const SurfaceRegion& region)
    {
        regions.push_back(region);
    }

    void WorkpieceModel::addSample(const SurfaceSample& sample)
    {
        samples.push_back(sample);
    }

    const SurfaceRegion* WorkpieceModel::findRegion(int regionId) const
    {
        for (const auto& region : regions)
        {
            if (region.id == regionId)
                return &region;
        }
        return nullptr;
    }

    void WorkpieceModel::applyRegionTargetThickness()
    {
        for (auto& sample : samples)
        {
            const SurfaceRegion* region = findRegion(sample.regionId);
            if (region != nullptr && region->enabled)
                sample.targetThickness = region->targetThickness;
        }
    }
}
