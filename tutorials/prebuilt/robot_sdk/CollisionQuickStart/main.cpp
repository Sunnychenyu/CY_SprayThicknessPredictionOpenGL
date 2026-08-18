#include <RobotSDK/RobotSdkApi.h>
#include <RobotSDK/ICollisionWorld.h>
#include <RobotSDK/IRobotInstance.h>
#include <RobotSDK/IRobotLoader.h>
#include <RobotSDK/IRobotModel.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{
    constexpr const char* kDefaultRobotRelativePath =
        "drake_models/iiwa_description/urdf/iiwa14_spheres_collision.urdf";
    constexpr const char* kRobotInstanceName = "sdk_robot";
    constexpr const char* kObstacleName = "moving_obstacle";

    struct DemoInput
    {
        std::string robotPath;
        smrobotgen2::sdk::RobotSourceType sourceType = smrobotgen2::sdk::RobotSourceType::Urdf;
        std::string obstacleType = "sphere";
        bool usingDefaultPath = true;
    };

    smrobotgen2::sdk::Transform translated(double x, double y, double z)
    {
        smrobotgen2::sdk::Transform transform;
        transform.values[3] = x;
        transform.values[7] = y;
        transform.values[11] = z;
        return transform;
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

    smrobotgen2::sdk::RobotSourceType parseSourceType(const char* value)
    {
        if (value != nullptr && std::string(value) == "simscape")
        {
            return smrobotgen2::sdk::RobotSourceType::Simscape;
        }

        return smrobotgen2::sdk::RobotSourceType::Urdf;
    }

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

    std::string normalizeObstacleType(const char* value)
    {
        if (value == nullptr)
        {
            return "sphere";
        }

        const std::string type = value;
        if (type == "box" || type == "sphere" || type == "cylinder")
        {
            return type;
        }

        return "sphere";
    }

    DemoInput parseInput(int argc, char** argv)
    {
        DemoInput input;
        input.robotPath = defaultRobotPath();

        if (argc > 1)
        {
            input.robotPath = argv[1];
            input.usingDefaultPath = false;
        }

        if (argc > 2)
        {
            input.sourceType = parseSourceType(argv[2]);
        }

        if (argc > 3)
        {
            input.obstacleType = normalizeObstacleType(argv[3]);
        }

        return input;
    }

    bool printResult(const char* label, const smrobotgen2::sdk::Result& result)
    {
        if (smrobotgen2::sdk::failed(result))
        {
            std::cerr << label << " failed: " << result.message << std::endl;
            return false;
        }

        std::cout << label << ": OK" << std::endl;
        return true;
    }

    bool printFailure(const char* label, const smrobotgen2::sdk::Result& result)
    {
        if (smrobotgen2::sdk::failed(result))
        {
            std::cerr << label << " failed: " << result.message << std::endl;
            return false;
        }

        return true;
    }

    bool addObstacle(smrobotgen2::sdk::ICollisionWorld& world, const std::string& type)
    {
        if (type == "box")
        {
            return printResult("addEnvironmentBox",
                world.addEnvironmentBox(kObstacleName, 0.35, 0.35, 0.35));
        }

        if (type == "cylinder")
        {
            return printResult("addEnvironmentCylinder",
                world.addEnvironmentCylinder(kObstacleName, 0.2, 0.45));
        }

        return printResult("addEnvironmentSphere",
            world.addEnvironmentSphere(kObstacleName, 0.24));
    }

    void setAnimatedJoints(smrobotgen2::sdk::IRobotInstance& instance, int frame)
    {
        std::vector<double> joints(instance.dof(), 0.0);
        const std::size_t activeDof = std::min<std::size_t>(joints.size(), 7);
        const double t = static_cast<double>(frame) * 0.12;

        for (std::size_t i = 0; i < activeDof; ++i)
        {
            const double phase = t + static_cast<double>(i) * 0.45;
            joints[i] = 0.35 * std::sin(phase);
        }

        printFailure("setJoints", instance.setJoints(joints.data(), joints.size()));
        printFailure("update robot", instance.update());
    }

    smrobotgen2::sdk::Transform obstacleTransformForFrame(int frame, int frameCount)
    {
        const double ratio = static_cast<double>(frame) / static_cast<double>(frameCount - 1);
        const double x = -0.55 + 1.10 * ratio;
        const double y = 0.05 * std::sin(static_cast<double>(frame) * 0.25);
        const double z = 0.55 + 0.10 * std::sin(static_cast<double>(frame) * 0.17);
        return translated(x, y, z);
    }

    bool isObstacleName(const char* name)
    {
        return name != nullptr && std::strcmp(name, kObstacleName) == 0;
    }

    bool isObstacleContactSide(const char* linkName, const char* elementName)
    {
        return isObstacleName(linkName) || isObstacleName(elementName);
    }

    std::string robotLinkFromContact(
        const smrobotgen2::sdk::CollisionContact& contact,
        const smrobotgen2::sdk::IRobotModel& model)
    {
        if (!isObstacleContactSide(contact.linkA, contact.elementA) &&
            model.linkIndex(contact.linkA) >= 0)
        {
            return contact.linkA;
        }

        if (!isObstacleContactSide(contact.linkB, contact.elementB) &&
            model.linkIndex(contact.linkB) >= 0)
        {
            return contact.linkB;
        }

        return model.rootLink();
    }

    void disableRobotSelfCollision(
        smrobotgen2::sdk::ICollisionWorld& world,
        const smrobotgen2::sdk::IRobotModel& model)
    {
        std::size_t disabledPairs = 0;
        std::size_t skippedPairs = 0;

        for (std::size_t i = 0; i < model.linkCount(); ++i)
        {
            for (std::size_t j = i; j < model.linkCount(); ++j)
            {
                const smrobotgen2::sdk::Result result =
                    world.disableCollisionPair(
                        kRobotInstanceName,
                        model.linkName(i),
                        kRobotInstanceName,
                        model.linkName(j));
                if (smrobotgen2::sdk::succeeded(result))
                {
                    ++disabledPairs;
                }
                else
                {
                    ++skippedPairs;
                }
            }
        }

        std::cout << "Self-collision filter pairs disabled: " << disabledPairs
                  << ", skipped: " << skippedPairs << std::endl;
    }

    void printContactSide(const char* linkName, const char* elementName)
    {
        if (linkName != nullptr && linkName[0] != '\0')
        {
            std::cout << linkName;
        }
        else
        {
            std::cout << "(no-link)";
        }

        if (elementName != nullptr && elementName[0] != '\0')
        {
            std::cout << "." << elementName;
        }
    }

    void printFrameStatus(
        int frame,
        double motion,
        bool colliding,
        std::size_t contactCount,
        smrobotgen2::sdk::ICollisionWorld& world,
        const smrobotgen2::sdk::CollisionContact& firstContact)
    {
        const double distance = world.distance();

        std::cout << "Frame " << std::setw(2) << frame
                  << " motion=" << std::fixed << std::setprecision(3) << motion
                  << " collision=" << (colliding ? "yes" : "no")
                  << " contacts=" << contactCount
                  << " distance=";

        if (distance < std::numeric_limits<double>::max())
        {
            std::cout << distance;
        }
        else
        {
            std::cout << "n/a";
        }

        if (colliding)
        {
            std::cout << " first=";
            printContactSide(firstContact.linkA, firstContact.elementA);
            std::cout << " <-> ";
            printContactSide(firstContact.linkB, firstContact.elementB);
            std::cout << " depth=" << firstContact.penetrationDepth;
        }

        std::cout << std::endl;
    }

    bool updateAndCheck(
        smrobotgen2::sdk::ICollisionWorld& world,
        smrobotgen2::sdk::CollisionContact* firstContact,
        bool printUpdateResult)
    {
        const smrobotgen2::sdk::Result updateResult = world.update();
        if (printUpdateResult)
        {
            printResult("update collision world", updateResult);
        }
        else if (smrobotgen2::sdk::failed(updateResult))
        {
            std::cerr << "update collision world failed: "
                      << updateResult.message << std::endl;
        }

        const bool colliding = world.checkCollision();
        if (colliding && firstContact != nullptr)
        {
            printFailure("contact(0)", world.contact(0, firstContact));
        }

        return colliding;
    }
}

int main(int argc, char** argv)
{
    const DemoInput input = parseInput(argc, argv);

    std::cout << "Usage: RobotSDKCollisionQuickStart [robot.urdf|robot.xml] [urdf|simscape] [box|sphere|cylinder]" << std::endl;
    std::cout << "Robot source type: " << sourceTypeName(input.sourceType) << std::endl;
    std::cout << "Robot path: " << input.robotPath << std::endl;
    std::cout << "Robot path source: "
              << (input.usingDefaultPath ? "built-in default" : "command line") << std::endl;
    std::cout << "Obstacle type: " << input.obstacleType << std::endl;

    if (input.robotPath.empty())
    {
        std::cerr << "Pass a robot file path or set SMROBOT_DATA_ROOT." << std::endl;
        return 1;
    }

    if (!std::filesystem::exists(input.robotPath))
    {
        std::cerr << "Robot file does not exist: " << input.robotPath << std::endl;
        return 1;
    }

    smrobotgen2::sdk::IRobotSdk* sdk = createRobotSdk();
    if (sdk == nullptr)
    {
        std::cerr << "Failed to create RobotSDK." << std::endl;
        return 1;
    }

    smrobotgen2::sdk::IRobotLoader* loader = sdk->createRobotLoader();
    if (loader == nullptr)
    {
        std::cerr << "Failed to create robot loader." << std::endl;
        destroyRobotSdk(sdk);
        return 1;
    }

    smrobotgen2::sdk::LoadRobotResult loadResult =
        loader->load(input.sourceType, input.robotPath.c_str());
    if (loadResult.model == nullptr)
    {
        std::cerr << "Load robot failed: " << loadResult.status.message << std::endl;
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 2;
    }

    smrobotgen2::sdk::IRobotModel* model = loadResult.model;
    std::cout << "Load robot: OK" << std::endl;
    std::cout << "Robot name: " << model->name()
              << ", links=" << model->linkCount()
              << ", joints=" << model->jointCount()
              << ", dof=" << model->dof() << std::endl;

    smrobotgen2::sdk::IRobotInstance* instance =
        sdk->createRobotInstance(model, kRobotInstanceName);
    if (instance == nullptr)
    {
        std::cerr << "Failed to create robot instance." << std::endl;
        sdk->destroyRobotModel(model);
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 3;
    }

    printResult("setBaseTransform", instance->setBaseTransform(translated(0.0, 0.0, 0.0)));

    smrobotgen2::sdk::ICollisionWorld* world = sdk->createCollisionWorld();
    if (world == nullptr)
    {
        std::cerr << "Failed to create collision world." << std::endl;
        sdk->destroyRobotInstance(instance);
        sdk->destroyRobotModel(model);
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 4;
    }

    if (!printResult("addRobotInstance", world->addRobotInstance(instance, nullptr)))
    {
        sdk->destroyCollisionWorld(world);
        sdk->destroyRobotInstance(instance);
        sdk->destroyRobotModel(model);
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 5;
    }

    disableRobotSelfCollision(*world, *model);

    if (!addObstacle(*world, input.obstacleType))
    {
        sdk->destroyCollisionWorld(world);
        sdk->destroyRobotInstance(instance);
        sdk->destroyRobotModel(model);
        sdk->destroyRobotLoader(loader);
        destroyRobotSdk(sdk);
        return 6;
    }

    constexpr int kFrameCount = 41;
    bool capturedFirstCollision = false;
    int capturedCollisionFrame = -1;
    std::string capturedRobotLink = model->rootLink();

    std::cout << "Running animated collision check..." << std::endl;
    for (int frame = 0; frame < kFrameCount; ++frame)
    {
        setAnimatedJoints(*instance, frame);

        const smrobotgen2::sdk::Transform obstacleTransform =
            obstacleTransformForFrame(frame, kFrameCount);
        printFailure("setEnvironmentTransform",
            world->setEnvironmentTransform(kObstacleName, obstacleTransform));

        smrobotgen2::sdk::CollisionContact firstContact;
        const bool colliding = updateAndCheck(*world, &firstContact, frame == 0);
        const std::size_t contactCount = world->contactCount();
        const double motion = obstacleTransform.values[3];

        printFrameStatus(frame, motion, colliding, contactCount, *world, firstContact);

        if (colliding && !capturedFirstCollision)
        {
            capturedRobotLink = robotLinkFromContact(firstContact, *model);
            capturedCollisionFrame = frame;
            capturedFirstCollision = true;
        }
    }

    if (capturedFirstCollision)
    {
        std::cout << "Replaying captured collision frame: "
                  << capturedCollisionFrame << std::endl;
        setAnimatedJoints(*instance, capturedCollisionFrame);
        const smrobotgen2::sdk::Transform capturedObstacleTransform =
            obstacleTransformForFrame(capturedCollisionFrame, kFrameCount);
        printFailure("setEnvironmentTransform",
            world->setEnvironmentTransform(kObstacleName, capturedObstacleTransform));

        smrobotgen2::sdk::CollisionContact baselineContact;
        const bool baselineCollision = updateAndCheck(*world, &baselineContact, true);
        const std::size_t baselineContactCount = world->contactCount();
        const double baselineDistance = world->distance();
        std::cout << "Collision before filtering pair: "
                  << (baselineCollision ? "yes" : "no")
                  << ", contacts=" << baselineContactCount
                  << ", distance=" << baselineDistance << std::endl;

        std::cout << "Disabling robot-environment pair: "
                  << capturedRobotLink << " <-> " << kObstacleName << std::endl;
        printResult("disableRobotEnvironmentCollision",
            world->disableRobotEnvironmentCollision(
                kRobotInstanceName,
                capturedRobotLink.c_str(),
                kObstacleName));

        smrobotgen2::sdk::CollisionContact disabledContact;
        const bool disabledCollision = updateAndCheck(*world, &disabledContact, true);
        const std::size_t disabledContactCount = world->contactCount();
        const double disabledDistance = world->distance();
        std::cout << "Collision after disabling pair: "
                  << (disabledCollision ? "yes" : "no")
                  << ", contacts=" << disabledContactCount
                  << ", distance=" << disabledDistance << std::endl;

        std::cout << "Re-enabling robot-environment pair: "
                  << capturedRobotLink << " <-> " << kObstacleName << std::endl;
        printResult("enableRobotEnvironmentCollision",
            world->enableRobotEnvironmentCollision(
                kRobotInstanceName,
                capturedRobotLink.c_str(),
                kObstacleName));

        smrobotgen2::sdk::CollisionContact enabledContact;
        const bool enabledCollision = updateAndCheck(*world, &enabledContact, true);
        const std::size_t enabledContactCount = world->contactCount();
        const double enabledDistance = world->distance();
        std::cout << "Collision after re-enabling pair: "
                  << (enabledCollision ? "yes" : "no")
                  << ", contacts=" << enabledContactCount
                  << ", distance=" << enabledDistance << std::endl;
    }
    else
    {
        std::cout << "No collision was found during the sweep; pair filtering demo skipped." << std::endl;
    }

    sdk->destroyCollisionWorld(world);
    sdk->destroyRobotInstance(instance);
    sdk->destroyRobotModel(model);
    sdk->destroyRobotLoader(loader);
    destroyRobotSdk(sdk);

    return 0;
}
