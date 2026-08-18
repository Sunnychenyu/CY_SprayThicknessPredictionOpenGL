#include <RobotSDK/RobotSdkApi.h>
#include <RobotSDK/IRobotLoader.h>
#include <RobotSDK/IRobotInstance.h>
#include <RobotSDK/IRobotModel.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    constexpr const char* kDefaultRobotRelativePath =
        "drake_models/iiwa_description/urdf/iiwa14_primitive_collision.urdf";

    struct RobotInput
    {
        std::string path;
        smrobotgen2::sdk::RobotSourceType sourceType = smrobotgen2::sdk::RobotSourceType::Urdf;
        bool usingDefaultPath = true;
    };

    const char* sourceTypeName(smrobotgen2::sdk::RobotSourceType sourceType)
    {
        switch (sourceType)
        {
        case smrobotgen2::sdk::RobotSourceType::Urdf:
            return "urdf";
        case smrobotgen2::sdk::RobotSourceType::Simscape:
            return "simscape";
        default:
            return "unknown";
        }
    }

    smrobotgen2::sdk::RobotSourceType parseSourceType(const char* value)
    {
        if (value != nullptr && std::string(value) == "simscape")
        {
            return smrobotgen2::sdk::RobotSourceType::Simscape;
        }

        return smrobotgen2::sdk::RobotSourceType::Urdf;
    }

    std::string defaultRobotPath()
    {
        const char* dataRoot = std::getenv("SMROBOT_DATA_ROOT");
        if (dataRoot == nullptr || dataRoot[0] == '\0')
        {
            return {};
        }

        std::filesystem::path path(dataRoot);
        path /= kDefaultRobotRelativePath;
        return path.generic_string();
    }

    RobotInput parseInput(int argc, char** argv)
    {
        RobotInput input;
        input.path = defaultRobotPath();

        if (argc > 1)
        {
            input.path = argv[1];
            input.usingDefaultPath = false;
        }

        if (argc > 2)
        {
            input.sourceType = parseSourceType(argv[2]);
        }

        return input;
    }

    void printTransformTranslation(
        const char* label,
        const smrobotgen2::sdk::Transform& transform)
    {
        std::cout << label << " translation: "
                  << transform.values[3] << ", "
                  << transform.values[7] << ", "
                  << transform.values[11] << std::endl;
    }

    void printResult(const char* label, const smrobotgen2::sdk::Result& result)
    {
        if (smrobotgen2::sdk::succeeded(result))
        {
            std::cout << label << ": OK" << std::endl;
        }
        else
        {
            std::cout << label << ": FAILED - " << result.message << std::endl;
        }
    }

    void printRobotSummary(const smrobotgen2::sdk::IRobotModel& model)
    {
        std::cout << "Load robot: OK" << std::endl;
        std::cout << "Robot name: " << model.name() << std::endl;
        std::cout << "Root link: " << model.rootLink() << std::endl;
        std::cout << "Links: " << model.linkCount() << std::endl;
        std::cout << "Joints: " << model.jointCount() << std::endl;
        std::cout << "DOF: " << model.dof() << std::endl;
        std::cout << "Root link index: " << model.linkIndex(model.rootLink()) << std::endl;

        for (std::size_t i = 0; i < model.jointCount(); ++i)
        {
            std::cout << "Joint[" << i << "]: " << model.jointName(i)
                      << ", dofIndex=" << model.jointDofIndex(i) << std::endl;
        }
    }

    void runJointUpdates(
        smrobotgen2::sdk::IRobotInstance& instance,
        const smrobotgen2::sdk::IRobotModel& model)
    {
        const char* probeLink = model.rootLink();
        if (model.linkCount() > 0)
        {
            probeLink = model.linkName(model.linkCount() - 1);
        }

        std::cout << "Probe link: " << probeLink << std::endl;

        std::vector<double> joints(instance.dof(), 0.0);
        const std::size_t activeDof = std::min<std::size_t>(joints.size(), 6);

        for (int step = 0; step < 4; ++step)
        {
            for (std::size_t i = 0; i < activeDof; ++i)
            {
                const double sign = (i % 2 == 0) ? 1.0 : -1.0;
                joints[i] = sign * 0.15 * static_cast<double>(step);
            }

            smrobotgen2::sdk::Result setResult = instance.setJoints(joints.data(), joints.size());
            smrobotgen2::sdk::Result updateResult = instance.update();

            std::cout << "Update step " << step << std::endl;
            printResult("  setJoints", setResult);
            printResult("  update", updateResult);

            smrobotgen2::sdk::Transform transform;
            smrobotgen2::sdk::Result transformResult =
                instance.linkTransform(probeLink, &transform);
            if (smrobotgen2::sdk::succeeded(transformResult))
            {
                printTransformTranslation("  probe", transform);
            }
            else
            {
                std::cout << "  linkTransform: FAILED - "
                          << transformResult.message << std::endl;
            }
        }
    }
}

int main(int argc, char** argv)
{
    smrobotgen2::sdk::IRobotSdk* sdk = createRobotSdk();
    if (sdk == nullptr)
    {
        std::cerr << "Failed to create RobotSDK." << std::endl;
        return 1;
    }

    std::cout << "RobotSDK version: " << sdk->version() << std::endl;
    std::cout << "RobotSDK ABI: " << sdk->abiVersion() << std::endl;
    std::cout << "Usage: RobotSDKQuickStart [robot.urdf|robot.xml] [urdf|simscape]" << std::endl;

    smrobotgen2::sdk::IRobotLoader* loader = sdk->createRobotLoader();
    if (loader == nullptr)
    {
        std::cerr << "Failed to create robot loader." << std::endl;
        destroyRobotSdk(sdk);
        return 1;
    }

    const RobotInput input = parseInput(argc, argv);
    if (input.path.empty())
    {
        std::cerr << "Pass a robot file path or set SMROBOT_DATA_ROOT." << std::endl;
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 2;
    }
    std::cout << "Robot source type: " << sourceTypeName(input.sourceType) << std::endl;
    std::cout << "Robot path: " << input.path << std::endl;
    if (input.usingDefaultPath)
    {
        std::cout << "Robot path source: SMROBOT_DATA_ROOT" << std::endl;
    }
    else
    {
        std::cout << "Robot path source: command line" << std::endl;
    }

    if (!std::filesystem::exists(input.path))
    {
        std::cerr << "Robot file does not exist: " << input.path << std::endl;
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 2;
    }

    smrobotgen2::sdk::LoadRobotResult loadResult =
        loader->load(input.sourceType, input.path.c_str());
    if (loadResult.model == nullptr)
    {
        std::cerr << "Load robot: FAILED - " << loadResult.status.message << std::endl;
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 3;
    }

    smrobotgen2::sdk::IRobotModel* model = loadResult.model;
    printRobotSummary(*model);

    smrobotgen2::sdk::IRobotInstance* instance =
        sdk->createRobotInstance(model, "quick_start_robot");
    if (instance == nullptr)
    {
        std::cerr << "Create robot instance: FAILED" << std::endl;
        sdk->destroyRobotModel(model);
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 4;
    }

    std::cout << "Create robot instance: OK" << std::endl;
    std::cout << "Instance name: " << instance->instanceName() << std::endl;
    std::cout << "Instance DOF: " << instance->dof() << std::endl;

    runJointUpdates(*instance, *model);

    sdk->destroyRobotInstance(instance);
    sdk->destroyRobotModel(model);
    sdk->destroyRobotLoader(loader);
    destroyRobotSdk(sdk);

    return 0;
}
