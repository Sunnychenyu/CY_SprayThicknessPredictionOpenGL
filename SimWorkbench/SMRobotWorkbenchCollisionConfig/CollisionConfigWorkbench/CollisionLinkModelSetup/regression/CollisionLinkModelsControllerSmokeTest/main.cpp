#include "CollisionLinkModelsController.h"
#include "CollisionLinkModelsCommandController.h"
#include "RobotQtViewerViewportServices.h"

#include <SimulationProject/CollisionModelSelectionIds.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectIo.h>

#include <QString>

#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace
{
    class FakeViewportServices : public robot_qt_viewer::RobotQtViewerViewportServices
    {
    public:
        mutable QString generatedObjectId;
        mutable int objectCoacdCalls = 0;

        void selectRobotMount(const QString&, const QString&, const QString&) override {}
        bool setActivePreviewRobotMount(const QString&) override { return false; }
        bool previewRobotMountTransform(const QString&, const simulation_project::TransformDesc&) override { return false; }
        bool previewRobotMountLink(const QString&, const QString&) override { return false; }
        bool upsertPreviewRobotMount(const simulation_project::RobotMountDesc&) override { return false; }
        bool removePreviewRobotMount(const QString&) override { return false; }
        void previewRobotBaseTransform(const QString&, const simulation_project::TransformDesc&) override {}
        void previewSceneObjectTransform(const QString&, const simulation_project::TransformDesc&) override {}
        bool commitSceneObjectTransform(const QString&, const simulation_project::TransformDesc&) override { return false; }
        bool removeSceneObject(const QString&) override { return false; }
        void selectObjectFrame(const QString&, const QString&) override {}
        bool previewObjectFrameTransform(
            const QString&,
            const QString&,
            const simulation_project::TransformDesc&) override { return false; }
        bool upsertPreviewObjectFrame(const QString&, const simulation_project::ObjectFrameDesc&) override { return false; }
        void selectRobotLink(const QString&, const QString&) override {}
        void selectRobotJointFrame(const QString&, const QString&) override {}
        void setActiveToolFrameRobot(const QString&) override {}
        void selectSceneObject(const QString&) override {}
        void selectMountedAttachment(const QString&) override {}
        bool setActiveMountedAttachment(const QString&) override { return true; }
        void setToolFrameVisibility(const robot_qt_viewer::RobotQtViewerToolFrameVisibility&) override {}
        void setRobotMountFrameVisibility(bool, bool) override {}
        void setPinnedRobotMountFrames(const QStringList&) override {}
        void focusMountFrameLink(const QString&, const QString&) override {}
        void clearMountFrameLinkFocus() override {}
        void focusObjectFrameObject(const QString&) override {}
        void clearObjectFrameObjectFocus() override {}
        void focusMountedAttachment(const QString&) override {}
        void clearMountedAttachmentFocus() override {}
        void previewObjectCollisionModelVariant(const QString&, const QString&) override {}
        void clearObjectCollisionModelVariantPreview() override {}
        void previewCollisionPairTargets(
            const QString&,
            const QString&,
            const QString&,
            const QString&,
            const QString&,
            const QString&,
            const QString&,
            const QString&) override {}
        robot_qt_viewer::RobotQtViewerViewportLoadResult loadProjectDocument(
            const simulation_project::ProjectDocument&,
            const std::filesystem::path&) override { return {}; }
        bool refreshCollisionConfiguration(
            const simulation_project::ProjectDocument&,
            const std::filesystem::path&) override { return true; }
        void setCollisionGeometryVisible(bool) override {}
        bool collisionQueriesEnabled() const override { return false; }
        bool setCollisionQueriesEnabled(bool) override { return false; }
        bool setActiveCollisionDetector(const QString&) override { return false; }
        bool setCollisionDetectorEnabled(const QString&, bool) override { return false; }
        bool setCollisionDetectorVisible(const QString&, bool) override { return false; }
        bool updateCollisionDetectorRuntimeOptions(const simulation_project::CollisionDetectorDesc&) override
        {
            return false;
        }
        bool rebuildCollisionDetectorsFromDocument(const simulation_project::ProjectDocument&) override { return false; }
        bool removeCollisionDetector(const QString&) override { return false; }
        bool setVisibleRobotCollisionVariant(const QString&, const QString&, const QString&) override { return false; }
        QString visibleRobotCollisionVariant(const QString&, const QString&) const override { return {}; }
        CollisionRuntimeRobotSummary robotCollisionSummary(
            const QString&,
            const QString& = QString(),
            const QString& = QString()) const override { return {}; }
        std::vector<CollisionRuntimeDetectorInfo> collisionRuntimeDetectors() const override { return {}; }
        bool generateRobotCollisionProxies(
            const QString&,
            const QString&,
            const robot_qt_viewer::CollisionRuntimeProxyRequest&,
            std::vector<simulation_project::CollisionElementOverrideDesc>&) const override { return false; }
        bool generateRobotCollisionProxiesFromExistingCollision(
            const QString&,
            const QString&,
            const robot_qt_viewer::CollisionRuntimeProxyRequest&,
            std::vector<simulation_project::CollisionElementOverrideDesc>&) const override { return false; }
        bool generateRobotCollisionProxiesFromExistingCollision(
            const QString&,
            const robot_qt_viewer::CollisionRuntimeProxyRequest&,
            std::vector<simulation_project::CollisionElementOverrideDesc>&) const override { return false; }
        bool generateRobotCollisionCoacdFromVisual(
            const QString&,
            const QString&,
            std::vector<simulation_project::CollisionElementOverrideDesc>&) const override { return false; }
        bool generateRobotCollisionCoacdFromExistingCollision(
            const QString&,
            const QString&,
            std::vector<simulation_project::CollisionElementOverrideDesc>&) const override { return false; }
        bool generateObjectCollisionCoacdFromVisual(
            const QString& objectId,
            std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const override
        {
            generatedObjectId = objectId;
            ++objectCoacdCalls;
            elements.clear();
            for(int i = 0; i < 2; ++i) {
                simulation_project::ObjectCollisionElementOverrideDesc element;
                element.id = "420_tool_attachment_coacd_" + std::to_string(i + 1);
                element.label = element.id;
                element.type = "mesh";
                element.role = simulation_project::kCoacdCollisionModelRole;
                element.source = simulation_project::kCoacdVisualSource;
                element.meshPath = "appGenerated://collision/coacd/test/part_" + std::to_string(i + 1) + ".obj";
                element.enabled = true;
                elements.push_back(std::move(element));
            }
            return true;
        }
        bool generateMissingRobotCollisionProxies(
            const QString&,
            const robot_qt_viewer::CollisionRuntimeProxyRequest&,
            std::vector<simulation_project::CollisionElementOverrideDesc>&) const override { return false; }
        bool evaluateRobotCollisionProxyQuality(
            const QString&,
            const QString&,
            const robot_qt_viewer::CollisionRuntimeProxyRequest&,
            const std::vector<simulation_project::CollisionElementOverrideDesc>&,
            bool,
            CollisionRuntimeProxyQualitySummary&) const override { return false; }
        double robotJointValue(const QString&, const QString&, bool* ok = nullptr) const override
        {
            if(ok != nullptr) {
                *ok = false;
            }
            return 0.0;
        }
        void setRobotJointValue(const QString&, const QString&, double) override {}
        void setRobotAutoMotion(const QString&, bool, double, double) override {}
    };

    struct Checks
    {
        int failures = 0;

        void require(bool condition, const std::string& message)
        {
            std::cout << (condition ? "[PASS] " : "[FAIL] ") << message << "\n";
            if(!condition) {
                ++failures;
            }
        }
    };

    CollisionLinkModelsViewModel buildAttachmentView(
        const simulation_project::ProjectDocument& document,
        const QString& previousVariantId = QString())
    {
        return CollisionLinkModelsController::buildViewModel(
            document,
            QStringLiteral("Red4600"),
            QStringLiteral("Link6"),
            QString(),
            QStringLiteral("420_tool_attachment"),
            QString(),
            previousVariantId,
            QString(),
            QString(),
            std::filesystem::path(PROJECT_SOURCE_PATH),
            nullptr,
            {});
    }

    int countSource(
        const CollisionLinkModelsViewModel& view,
        const QString& source)
    {
        int count = 0;
        for(const CollisionLinkModelVariantItemView& variant : view.variants) {
            if(variant.source == source) {
                ++count;
            }
        }
        return count;
    }

    const CollisionLinkModelVariantItemView* findSource(
        const CollisionLinkModelsViewModel& view,
        const QString& source)
    {
        for(const CollisionLinkModelVariantItemView& variant : view.variants) {
            if(variant.source == source) {
                return &variant;
            }
        }
        return nullptr;
    }
}

int main()
{
    Checks checks;
    const std::filesystem::path root(PROJECT_SOURCE_PATH);
    const std::filesystem::path projectPath =
        root / "config" / "projects" / "420-red4600-tool.sys.json";

    simulation_project::ProjectDocument document;
    std::string error;
    checks.require(
        simulation_project::loadProjectDocument(projectPath, document, &error),
        "420 tool project loads");
    if(!error.empty()) {
        std::cout << "load detail: " << error << "\n";
    }

    CollisionLinkModelsViewModel initial = buildAttachmentView(document);
    checks.require(
        countSource(initial, simulation_project::kConvertFromVisualCollisionSource) == 1,
        "attachment initially exposes Convert from Visual");
    checks.require(
        countSource(initial, simulation_project::kDefinedInProjectCollisionSource) == 1,
        "attachment initially exposes Defined in Project");
    checks.require(
        countSource(initial, simulation_project::kCoacdVisualSource) == 0,
        "attachment initially has no COACD variant");

    FakeViewportServices viewportServices;
    const CollisionLinkModelsCommandResult commandResult =
        CollisionLinkModelsCommandController::generateCoacdForObject(
            document,
            &viewportServices,
            QStringLiteral("420_tool_attachment"));
    checks.require(commandResult.success, "attachment COACD command succeeds through viewport service");
    checks.require(commandResult.projectChanged, "attachment COACD command marks project changed");
    checks.require(commandResult.generatedCount == 2, "attachment COACD command reports generated parts");
    checks.require(
        viewportServices.objectCoacdCalls == 1 &&
            viewportServices.generatedObjectId == QStringLiteral("420_tool_attachment"),
        "attachment COACD command targets the mounted attachment id");

    CollisionLinkModelsViewModel generatedView = buildAttachmentView(document);
    checks.require(
        countSource(generatedView, simulation_project::kCoacdVisualSource) == 1,
        "COACD parts are grouped into one Generated from COACD variant");
    checks.require(
        countSource(generatedView, simulation_project::kDefinedInProjectCollisionSource) == 1,
        "Defined in Project remains a separate variant");

    const CollisionLinkModelVariantItemView* coacd =
        findSource(generatedView, simulation_project::kCoacdVisualSource);
    checks.require(
        coacd != nullptr && coacd->label == QStringLiteral("Generated from COACD"),
        "generated variant has the expected label");
    checks.require(
        coacd != nullptr && !coacd->variantId.isEmpty(),
        "generated variant has a stable id");

    const QString generatedVariantId = coacd != nullptr ? coacd->variantId : QString();
    CollisionLinkModelsViewModel refreshed = buildAttachmentView(document, generatedVariantId);
    const CollisionLinkModelVariantItemView* refreshedCoacd =
        findSource(refreshed, simulation_project::kCoacdVisualSource);
    checks.require(
        refreshedCoacd != nullptr && refreshedCoacd->selected,
        "view-model refresh preserves the generated variant selection");

    simulation_project::ProjectDocumentService service(document);
    bool changed = false;
    checks.require(
        service.setActiveObjectCollisionModel(
            "420_tool_attachment",
            generatedVariantId.toStdString(),
            &changed,
            &error),
        "generated variant can be set current");
    CollisionLinkModelsViewModel currentView = buildAttachmentView(document, generatedVariantId);
    const CollisionLinkModelVariantItemView* currentCoacd =
        findSource(currentView, simulation_project::kCoacdVisualSource);
    checks.require(
        currentCoacd != nullptr && currentCoacd->current,
        "generated variant remains current after document round-trip");

    return checks.failures == 0 ? 0 : 1;
}
