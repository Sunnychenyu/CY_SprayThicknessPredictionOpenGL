#include <SprayThicknessPredictionOpenGL/AxisymmetricProfileReduction.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace spraythickness::opengl
{
    namespace
    {
        constexpr double kToleranceFloor = 1.0e-8;

        struct Segment
        {
            AxisymmetricProfilePoint first;
            AxisymmetricProfilePoint second;
        };

        struct Node
        {
            Eigen::Vector3d positionSum = Eigen::Vector3d::Zero();
            Eigen::Vector3d normalSum = Eigen::Vector3d::Zero();
            Eigen::Vector2d sectionSum = Eigen::Vector2d::Zero();
            std::size_t count{ 0 };
            std::vector<std::size_t> segments;

            AxisymmetricProfilePoint point() const
            {
                AxisymmetricProfilePoint result;
                const double divisor = static_cast<double>(std::max<std::size_t>(1, count));
                result.position = positionSum / divisor;
                result.sectionPosition = sectionSum / divisor;
                result.normal = normalSum.squaredNorm() > 1.0e-16
                    ? normalSum.normalized()
                    : Eigen::Vector3d::UnitZ();
                return result;
            }
        };

        struct IndexedSegment
        {
            std::size_t firstNode{ 0 };
            std::size_t secondNode{ 0 };
            bool used{ false };
        };

        Eigen::Vector3d normalizedOr(const Eigen::Vector3d& vector, const Eigen::Vector3d& fallback)
        {
            return vector.squaredNorm() > 1.0e-16 ? vector.normalized() : fallback;
        }

        Eigen::Vector3d projectToPlane(const Eigen::Vector3d& vector, const Eigen::Vector3d& normal)
        {
            return vector - normal * vector.dot(normal);
        }

        std::uint64_t nodeKey(const Eigen::Vector2d& point, double tolerance)
        {
            const std::int32_t x = static_cast<std::int32_t>(std::llround(point.x() / tolerance));
            const std::int32_t y = static_cast<std::int32_t>(std::llround(point.y() / tolerance));
            return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U)
                | static_cast<std::uint32_t>(y);
        }

        AxisymmetricProfilePoint interpolate(
            const AxisymmetricProfilePoint& first,
            const AxisymmetricProfilePoint& second,
            double t)
        {
            AxisymmetricProfilePoint result;
            result.position = first.position + t * (second.position - first.position);
            result.normal = normalizedOr(
                first.normal + t * (second.normal - first.normal),
                Eigen::Vector3d::UnitZ());
            result.sectionPosition = first.sectionPosition
                + t * (second.sectionPosition - first.sectionPosition);
            return result;
        }

        bool clipSegmentToSelection(
            const AxisymmetricProfileSelection& selection,
            const Eigen::Vector2d& first,
            const Eigen::Vector2d& second,
            double& minimumT,
            double& maximumT)
        {
            minimumT = 0.0;
            maximumT = 1.0;
            if(!selection.enabled) {
                return true;
            }
            const Eigen::Vector2d minimum = selection.minimum.cwiseMin(selection.maximum);
            const Eigen::Vector2d maximum = selection.minimum.cwiseMax(selection.maximum);
            const Eigen::Vector2d delta = second - first;
            for(int dimension = 0; dimension < 2; ++dimension) {
                if(std::abs(delta[dimension]) <= 1.0e-15) {
                    if(first[dimension] < minimum[dimension]
                        || first[dimension] > maximum[dimension]) {
                        return false;
                    }
                    continue;
                }
                double entry = (minimum[dimension] - first[dimension]) / delta[dimension];
                double exit = (maximum[dimension] - first[dimension]) / delta[dimension];
                if(entry > exit) {
                    std::swap(entry, exit);
                }
                minimumT = std::max(minimumT, entry);
                maximumT = std::min(maximumT, exit);
                if(minimumT > maximumT) {
                    return false;
                }
            }
            minimumT = std::clamp(minimumT, 0.0, 1.0);
            maximumT = std::clamp(maximumT, 0.0, 1.0);
            return minimumT <= maximumT;
        }

    }

    bool AxisymmetricProfileSelection::contains(const Eigen::Vector2d& point) const
    {
        if(!enabled) {
            return true;
        }
        return point.x() >= std::min(minimum.x(), maximum.x())
            && point.x() <= std::max(minimum.x(), maximum.x())
            && point.y() >= std::min(minimum.y(), maximum.y())
            && point.y() <= std::max(minimum.y(), maximum.y());
    }

    AxisymmetricProfileSlice buildAxisymmetricProfileSlice(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const Eigen::Vector3d& axisOrigin,
        const Eigen::Vector3d& axisDirection,
        const Eigen::Vector3d& referenceDirection)
    {
        AxisymmetricProfileSlice result;
        if(workpiece.samples.empty() || workpiece.triangleIndices.size() < 3) {
            result.failureReason = "The workpiece has no triangle mesh for profile extraction.";
            return result;
        }

        result.axisOrigin = axisOrigin;
        result.axisDirection = normalizedOr(axisDirection, Eigen::Vector3d::UnitZ());
        result.radialDirection = normalizedOr(
            projectToPlane(referenceDirection, result.axisDirection),
            std::abs(result.axisDirection.x()) < 0.9
                ? Eigen::Vector3d::UnitX()
                : Eigen::Vector3d::UnitY());
        result.radialDirection = normalizedOr(
            projectToPlane(result.radialDirection, result.axisDirection),
            Eigen::Vector3d::UnitX());
        const Eigen::Vector3d planeNormal = normalizedOr(
            result.axisDirection.cross(result.radialDirection), Eigen::Vector3d::UnitY());

        Eigen::Vector3d minimum3 = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d maximum3 = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
        for(const auto& sample : workpiece.samples) {
            minimum3 = minimum3.cwiseMin(sample.position);
            maximum3 = maximum3.cwiseMax(sample.position);
        }
        const double tolerance = std::max(kToleranceFloor, (maximum3 - minimum3).norm() * 1.0e-7);
        std::vector<Segment> segments;
        segments.reserve(workpiece.triangleIndices.size() / 3);

        for(std::size_t offset = 0; offset + 2 < workpiece.triangleIndices.size(); offset += 3) {
            const std::uint32_t indices[3] = {
                workpiece.triangleIndices[offset],
                workpiece.triangleIndices[offset + 1],
                workpiece.triangleIndices[offset + 2]};
            if(indices[0] >= workpiece.samples.size() || indices[1] >= workpiece.samples.size()
                || indices[2] >= workpiece.samples.size()) {
                continue;
            }
            const auto& a = workpiece.samples[indices[0]];
            const auto& b = workpiece.samples[indices[1]];
            const auto& c = workpiece.samples[indices[2]];
            const sprayworkpiece::SurfaceSample* vertices[3] = { &a, &b, &c };
            const double distance[3] = {
                (a.position - axisOrigin).dot(planeNormal),
                (b.position - axisOrigin).dot(planeNormal),
                (c.position - axisOrigin).dot(planeNormal)};
            if(std::abs(distance[0]) <= tolerance && std::abs(distance[1]) <= tolerance
                && std::abs(distance[2]) <= tolerance) {
                continue;
            }

            std::vector<AxisymmetricProfilePoint> intersections;
            for(int edge = 0; edge < 3; ++edge) {
                const int next = (edge + 1) % 3;
                const double firstDistance = distance[edge];
                const double secondDistance = distance[next];
                if(std::abs(firstDistance) <= tolerance) {
                    AxisymmetricProfilePoint point;
                    point.position = vertices[edge]->position;
                    point.normal = normalizedOr(vertices[edge]->normal, Eigen::Vector3d::UnitZ());
                    const Eigen::Vector3d relative = point.position - axisOrigin;
                    point.sectionPosition = Eigen::Vector2d(
                        relative.dot(result.radialDirection),
                        relative.dot(result.axisDirection));
                    intersections.push_back(point);
                }
                if((firstDistance < -tolerance && secondDistance > tolerance)
                    || (firstDistance > tolerance && secondDistance < -tolerance)) {
                    const double t = firstDistance / (firstDistance - secondDistance);
                    AxisymmetricProfilePoint point;
                    point.position = vertices[edge]->position
                        + t * (vertices[next]->position - vertices[edge]->position);
                    point.normal = normalizedOr(
                        vertices[edge]->normal + t * (vertices[next]->normal - vertices[edge]->normal),
                        Eigen::Vector3d::UnitZ());
                    const Eigen::Vector3d relative = point.position - axisOrigin;
                    point.sectionPosition = Eigen::Vector2d(
                        relative.dot(result.radialDirection),
                        relative.dot(result.axisDirection));
                    intersections.push_back(point);
                }
            }
            std::vector<AxisymmetricProfilePoint> unique;
            for(const auto& point : intersections) {
                const bool duplicate = std::any_of(unique.begin(), unique.end(),
                    [&point, tolerance](const AxisymmetricProfilePoint& other) {
                        return (point.sectionPosition - other.sectionPosition).norm() <= tolerance;
                    });
                if(!duplicate) {
                    unique.push_back(point);
                }
            }
            if(unique.size() < 2) {
                continue;
            }
            std::size_t first = 0;
            std::size_t second = 1;
            double largestDistance = -1.0;
            for(std::size_t i = 0; i < unique.size(); ++i) {
                for(std::size_t j = i + 1; j < unique.size(); ++j) {
                    const double distanceSquared =
                        (unique[i].sectionPosition - unique[j].sectionPosition).squaredNorm();
                    if(distanceSquared > largestDistance) {
                        largestDistance = distanceSquared;
                        first = i;
                        second = j;
                    }
                }
            }
            const AxisymmetricProfilePoint midpoint = [&unique, first, second]() {
                AxisymmetricProfilePoint point;
                point.sectionPosition = (unique[first].sectionPosition + unique[second].sectionPosition) * 0.5;
                return point;
            }();
            if(midpoint.sectionPosition.x() < -tolerance) {
                continue;
            }
            if(unique[first].sectionPosition.x() < -tolerance
                || unique[second].sectionPosition.x() < -tolerance) {
                continue;
            }
            segments.push_back({ unique[first], unique[second] });
        }

        if(segments.empty()) {
            result.failureReason = "The reference plane did not intersect the positive radial half of the mesh.";
            return result;
        }

        std::unordered_map<std::uint64_t, std::size_t> nodeByKey;
        std::vector<Node> nodes;
        std::vector<IndexedSegment> indexedSegments;
        const auto addNode = [&nodeByKey, &nodes, tolerance](const AxisymmetricProfilePoint& point) {
            const std::uint64_t key = nodeKey(point.sectionPosition, tolerance);
            auto iterator = nodeByKey.find(key);
            if(iterator == nodeByKey.end()) {
                const std::size_t index = nodes.size();
                nodeByKey.emplace(key, index);
                nodes.emplace_back();
                iterator = nodeByKey.find(key);
            }
            Node& node = nodes[iterator->second];
            node.positionSum += point.position;
            node.normalSum += point.normal;
            node.sectionSum += point.sectionPosition;
            ++node.count;
            return iterator->second;
        };
        for(const Segment& segment : segments) {
            const std::size_t first = addNode(segment.first);
            const std::size_t second = addNode(segment.second);
            if(first == second) {
                continue;
            }
            const std::size_t index = indexedSegments.size();
            indexedSegments.push_back({ first, second, false });
            nodes[first].segments.push_back(index);
            nodes[second].segments.push_back(index);
        }

        const auto appendContour = [&result, &nodes, &indexedSegments](std::size_t startNode) {
            AxisymmetricProfileContour contour;
            std::size_t currentNode = startNode;
            std::size_t previousSegment = std::numeric_limits<std::size_t>::max();
            for(;;) {
                contour.points.push_back(nodes[currentNode].point());
                std::size_t nextSegment = std::numeric_limits<std::size_t>::max();
                for(const std::size_t candidate : nodes[currentNode].segments) {
                    if(candidate != previousSegment && !indexedSegments[candidate].used) {
                        nextSegment = candidate;
                        break;
                    }
                }
                if(nextSegment == std::numeric_limits<std::size_t>::max()) {
                    break;
                }
                indexedSegments[nextSegment].used = true;
                const IndexedSegment& segment = indexedSegments[nextSegment];
                const std::size_t nextNode = segment.firstNode == currentNode
                    ? segment.secondNode : segment.firstNode;
                previousSegment = nextSegment;
                currentNode = nextNode;
                if(currentNode == startNode) {
                    contour.closed = true;
                    break;
                }
            }
            if(contour.points.size() >= 2) {
                result.contours.push_back(std::move(contour));
            }
        };
        for(std::size_t nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
            if(nodes[nodeIndex].segments.size() == 1) {
                appendContour(nodeIndex);
            }
        }
        for(std::size_t segmentIndex = 0; segmentIndex < indexedSegments.size(); ++segmentIndex) {
            if(!indexedSegments[segmentIndex].used) {
                appendContour(indexedSegments[segmentIndex].firstNode);
            }
        }
        if(result.contours.empty()) {
            result.failureReason = "Profile segment stitching produced no usable contours.";
            return result;
        }

        result.minimum = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
        result.maximum = Eigen::Vector2d::Constant(std::numeric_limits<double>::lowest());
        for(const auto& contour : result.contours) {
            for(const auto& point : contour.points) {
                result.minimum = result.minimum.cwiseMin(point.sectionPosition);
                result.maximum = result.maximum.cwiseMax(point.sectionPosition);
            }
        }
        return result;
    }

    AxisymmetricProfileReduction buildAxisymmetricProfileReduction(
        const AxisymmetricProfileSlice& slice,
        const AxisymmetricProfileSelection& selection,
        std::size_t sampleCount)
    {
        AxisymmetricProfileReduction result;
        if(!slice.valid()) {
            result.failureReason = slice.failureReason.empty()
                ? "The axisymmetric profile slice is unavailable."
                : slice.failureReason;
            return result;
        }
        if(!selection.enabled) {
            result.failureReason = "Select a profile region before prediction.";
            return result;
        }
        result.requestedSampleCount = std::max<std::size_t>(2, sampleCount);
        struct SelectedPath
        {
            std::vector<AxisymmetricProfilePoint> points;
            double arcLength{ 0.0 };
        };
        std::vector<SelectedPath> selectedPaths;
        selectedPaths.reserve(slice.contours.size());
        const double mergeTolerance = std::max(kToleranceFloor,
            (slice.maximum - slice.minimum).norm() * 1.0e-7);
        for(const AxisymmetricProfileContour& source : slice.contours) {
            if(source.points.size() < 2) {
                continue;
            }
            std::vector<SelectedPath> contourPaths;
            const std::size_t segmentCount = source.closed
                ? source.points.size() : source.points.size() - 1;
            for(std::size_t segment = 0; segment < segmentCount; ++segment) {
                const AxisymmetricProfilePoint& first = source.points[segment];
                const AxisymmetricProfilePoint& second = source.points[(segment + 1) % source.points.size()];
                double minimumT = 0.0;
                double maximumT = 1.0;
                if(!clipSegmentToSelection(selection, first.sectionPosition,
                    second.sectionPosition, minimumT, maximumT)
                    || maximumT - minimumT <= 1.0e-12) {
                    continue;
                }
                const AxisymmetricProfilePoint clippedFirst = interpolate(
                    first, second, minimumT);
                const AxisymmetricProfilePoint clippedSecond = interpolate(
                    first, second, maximumT);
                if(!contourPaths.empty()
                    && (contourPaths.back().points.back().position - clippedFirst.position).norm()
                        <= mergeTolerance) {
                    contourPaths.back().points.push_back(clippedSecond);
                } else {
                    SelectedPath path;
                    path.points.push_back(clippedFirst);
                    path.points.push_back(clippedSecond);
                    contourPaths.push_back(std::move(path));
                }
            }
            for(SelectedPath& path : contourPaths) {
                for(std::size_t index = 1; index < path.points.size(); ++index) {
                    path.arcLength += (path.points[index].sectionPosition
                        - path.points[index - 1].sectionPosition).norm();
                    result.displayLineSegments.push_back(path.points[index - 1]);
                    result.displayLineSegments.push_back(path.points[index]);
                }
                if(path.arcLength > 1.0e-12) {
                    selectedPaths.push_back(std::move(path));
                }
            }
        }
        if(selectedPaths.empty()) {
            result.failureReason = "The selected profile region contains no prediction samples.";
            return result;
        }

        result.selectedArcLengthMeters = 0.0;
        for(const SelectedPath& path : selectedPaths) {
            result.selectedArcLengthMeters += path.arcLength;
        }
        if(result.selectedArcLengthMeters <= 1.0e-12) {
            result.failureReason = "The selected profile region has zero length.";
            return result;
        }
        const std::size_t minimumSamplesPerSegment = 2;
        const std::size_t minimumSampleCount = selectedPaths.size() * minimumSamplesPerSegment;
        const std::size_t actualSampleCount = std::max(
            result.requestedSampleCount, minimumSampleCount);
        result.actualSampleCount = actualSampleCount;
        const std::size_t extraSampleCount = actualSampleCount - minimumSampleCount;
        std::size_t assignedExtraSamples = 0;
        for(std::size_t pathIndex = 0; pathIndex < selectedPaths.size(); ++pathIndex) {
            const SelectedPath& path = selectedPaths[pathIndex];
            std::size_t extra = static_cast<std::size_t>(std::floor(
                static_cast<double>(extraSampleCount) * path.arcLength
                / result.selectedArcLengthMeters));
            if(pathIndex + 1 == selectedPaths.size()) {
                extra = extraSampleCount - assignedExtraSamples;
            } else {
                extra = std::min(extra, extraSampleCount - assignedExtraSamples);
            }
            assignedExtraSamples += extra;
            const std::size_t samplesInSegment = minimumSamplesPerSegment + extra;
            const std::uint32_t firstSampleIndex = static_cast<std::uint32_t>(
                result.predictionSamples.size());
            std::size_t activeSegment = 1;
            double completedLength = 0.0;
            for(std::size_t sampleIndex = 0; sampleIndex < samplesInSegment; ++sampleIndex) {
                const double targetLength = samplesInSegment > 1
                    ? path.arcLength * static_cast<double>(sampleIndex)
                        / static_cast<double>(samplesInSegment - 1)
                    : 0.0;
                while(activeSegment < path.points.size()
                    && completedLength
                        + (path.points[activeSegment].sectionPosition
                            - path.points[activeSegment - 1].sectionPosition).norm()
                        < targetLength) {
                    completedLength += (path.points[activeSegment].sectionPosition
                        - path.points[activeSegment - 1].sectionPosition).norm();
                    ++activeSegment;
                }
                const std::size_t segmentEnd = std::min(
                    activeSegment, path.points.size() - 1);
                const AxisymmetricProfilePoint& segmentFirst = path.points[segmentEnd - 1];
                const AxisymmetricProfilePoint& segmentSecond = path.points[segmentEnd];
                const double segmentLength = (segmentSecond.sectionPosition
                    - segmentFirst.sectionPosition).norm();
                const double t = segmentLength > 1.0e-12
                    ? std::clamp((targetLength - completedLength) / segmentLength, 0.0, 1.0)
                    : 0.0;
                const AxisymmetricProfilePoint point = interpolate(segmentFirst, segmentSecond, t);
                sprayworkpiece::SurfaceSample sample;
                sample.position = point.position;
                sample.normal = point.normal;
                sample.areaWeight = 1.0;
                result.predictionSamples.push_back(sample);
                ++result.selectedContourPointCount;
                if(sampleIndex > 0) {
                    spraythickness::AxisymmetricProfileSampleSegment mappingSegment;
                    mappingSegment.firstSampleIndex = firstSampleIndex
                        + static_cast<std::uint32_t>(sampleIndex - 1);
                    mappingSegment.secondSampleIndex = firstSampleIndex
                        + static_cast<std::uint32_t>(sampleIndex);
                    const sprayworkpiece::SurfaceSample& previousSample =
                        result.predictionSamples[result.predictionSamples.size() - 2];
                    const Eigen::Vector3d previousRelative = previousSample.position - slice.axisOrigin;
                    const double previousAxial = previousRelative.dot(slice.axisDirection);
                    mappingSegment.firstSectionPosition = Eigen::Vector2d(
                        previousRelative.dot(slice.radialDirection), previousAxial);
                    mappingSegment.secondSectionPosition = point.sectionPosition;
                    result.sampleSegments.push_back(mappingSegment);
                }
            }
        }
        return result;
    }
}
