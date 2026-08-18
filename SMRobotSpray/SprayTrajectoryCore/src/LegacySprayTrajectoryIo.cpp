#include <SprayTrajectoryCore/LegacySprayTrajectoryIo.h>

#include <Eigen/Core>

#include <fstream>
#include <sstream>

namespace spraytrajectory
{
    namespace
    {
        std::vector<double> parseNumbers(const std::string& line)
        {
            std::vector<double> values;
            std::istringstream stream(line);
            std::string token;
            while(stream >> token) {
                try {
                    values.push_back(std::stod(token));
                } catch(const std::exception&) {
                }
            }
            return values;
        }
    }

    LegacySprayTrajectoryLoadResult LegacySprayTrajectoryIo::loadMatrixText(
        const std::filesystem::path& path,
        const LegacySprayTrajectoryLoadOptions& options)
    {
        LegacySprayTrajectoryLoadResult result;
        std::ifstream input(path);
        if(!input) {
            result.warnings.push_back("Failed to open the legacy spray trajectory file.");
            return result;
        }

        SpraySegment segment;
        segment.processId = options.processId;
        segment.sprayEnabled = true;
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        int matrixRow = 0;
        std::string line;
        while(std::getline(input, line)) {
            const std::vector<double> values = parseNumbers(line);
            if(values.empty()) {
                continue;
            }
            if(values.size() >= 4 && matrixRow < 4) {
                for(int column = 0; column < 4; ++column) {
                    matrix(matrixRow, column) = values[static_cast<std::size_t>(column)];
                }
                ++matrixRow;
                continue;
            }
            if(values.size() == 1 && matrixRow == 4) {
                SprayPathPoint point;
                point.time = values.front();
                point.tcpPose = Eigen::Isometry3d(matrix);
                point.tcpPose.translation() *= options.lengthScaleToMeters;
                point.sprayEnabled = true;
                point.processId = options.processId;
                segment.points.push_back(point);
                matrix = Eigen::Matrix4d::Identity();
                matrixRow = 0;
            }
        }

        if(matrixRow != 0) {
            result.warnings.push_back("Ignored an incomplete trajectory matrix at the end of the file.");
        }
        if(segment.points.empty()) {
            result.warnings.push_back("The legacy trajectory file contained no complete matrix/timestamp blocks.");
            return result;
        }

        result.trajectory.name = path.stem().string();
        result.trajectory.segments.push_back(std::move(segment));
        result.success = true;
        return result;
    }
}
