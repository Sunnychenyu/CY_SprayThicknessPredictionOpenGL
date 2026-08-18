#include "CollisionDetectorWorkbenchDocumentFacade.h"

#include "CollisionDetectorsController.h"
#include "CollisionWorkbenchServices.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>

#include <QStringList>

#include <algorithm>
#include <cctype>
#include <utility>

namespace
{
    const simulation_project::CollisionDetectorDesc* findCollisionDetector(
        const simulation_project::ProjectDocument& document,
        const std::string& detectorId)
    {
        const auto it = std::find_if(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& detector) {
                return detector.id == detectorId;
            });
        return it == document.collision.detectors.end() ? nullptr : &(*it);
    }

    const simulation_project::CollisionSelectionSetDesc* findCollisionSelectionSet(
        const simulation_project::ProjectDocument& document,
        const std::string& selectionSetId)
    {
        const auto it = std::find_if(
            document.collision.selectionSets.begin(),
            document.collision.selectionSets.end(),
            [&](const simulation_project::CollisionSelectionSetDesc& selectionSet) {
                return selectionSet.id == selectionSetId;
            });
        return it == document.collision.selectionSets.end() ? nullptr : &(*it);
    }

    bool isMountedAttachmentId(
        const simulation_project::ProjectDocument& document,
        const std::string& attachmentId)
    {
        if(attachmentId.empty()) {
            return false;
        }
        return std::any_of(
            document.mountedAttachments.begin(),
            document.mountedAttachments.end(),
            [&](const simulation_project::MountedAttachmentDesc& attachment) {
                return attachment.id == attachmentId;
            });
    }

    std::string cleanIdentifierBase(const std::string& value, const std::string& fallback)
    {
        std::string result = value.empty() ? fallback : value;
        for(char& ch : result) {
            if(!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_') {
                ch = '_';
            }
        }
        return result.empty() ? fallback : result;
    }

    std::string makeUniqueCollisionDetectorId(
        const simulation_project::ProjectDocument& document,
        const std::string& baseId)
    {
        const std::string cleanBase = cleanIdentifierBase(baseId, "collision_detector");
        int suffix = 1;
        while(true) {
            const std::string candidate = suffix == 1 ? cleanBase : cleanBase + "_" + std::to_string(suffix);
            const bool exists = std::any_of(
                document.collision.detectors.begin(),
                document.collision.detectors.end(),
                [&](const simulation_project::CollisionDetectorDesc& detector) {
                    return detector.id == candidate;
                });
            if(!exists) {
                return candidate;
            }
            ++suffix;
        }
    }

    simulation_project::CollisionDetectorTargetDesc makeRobotTarget(const std::string& robotId)
    {
        simulation_project::CollisionDetectorTargetDesc target;
        target.robotId = robotId;
        return target;
    }

    simulation_project::CollisionDetectorTargetDesc makeObjectTarget(const std::string& objectId)
    {
        simulation_project::CollisionDetectorTargetDesc target;
        target.objectId = objectId;
        return target;
    }

    void applyDocumentDefaults(
        const simulation_project::ProjectDocument& document,
        simulation_project::CollisionDetectorDesc& detector)
    {
        detector.geometryRole = "Exact";
        detector.contacts = document.collision.query.contacts;
        detector.nearestPoints = document.collision.query.nearestPoints;
        detector.distance = document.collision.query.nearestPoints;
        detector.maxContacts = document.collision.query.maxContacts;
        detector.visualization = document.collision.visualization;
        detector.visualization.showNearestPoints = detector.nearestPoints;
    }

    void addSceneAllTargets(
        const simulation_project::ProjectDocument& document,
        simulation_project::CollisionDetectorDesc& detector)
    {
        for(const simulation_project::RobotDesc& robot : document.robots) {
            detector.targets.push_back(makeRobotTarget(robot.id));
        }
        for(const simulation_project::SceneObjectDesc& object : document.objects) {
            detector.targets.push_back(makeObjectTarget(object.id));
        }
        simulation_project::CollisionPairGeneratorDesc generator;
        generator.type = "SceneAll";
        detector.pairGenerators.push_back(std::move(generator));
    }

    simulation_project::CollisionDetectorDesc makeTaskPanelDefaultDetector(
        const simulation_project::ProjectDocument& document)
    {
        simulation_project::CollisionDetectorDesc detector;
        detector.id = makeUniqueCollisionDetectorId(document, "set_detector");
        detector.enabled = document.collision.query.enabled;
        applyDocumentDefaults(document, detector);

        if(document.collision.selectionSets.size() >= 2) {
            detector.name = "Between Sets Detector";
            detector.type = "SelectedObjects";
            detector.queryPolicy = "BetweenSets";
            detector.selectionSetAId = document.collision.selectionSets[0].id;
            detector.selectionSetBId = document.collision.selectionSets[1].id;
        } else if(document.collision.selectionSets.size() == 1) {
            detector.name = "Within Set Detector";
            detector.type = "SelectedObjects";
            detector.queryPolicy = "WithinSet";
            detector.selectionSetAId = document.collision.selectionSets[0].id;
        } else {
            detector.name = "All Enabled Detector";
            detector.type = "SceneAll";
            detector.queryPolicy = "AllEnabled";
            addSceneAllTargets(document, detector);
        }

        return detector;
    }

    QString memberText(const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        if(!member.attachmentId.empty()) {
            return QString("attachment: %1").arg(QString::fromStdString(member.attachmentId));
        }
        if(!member.objectId.empty()) {
            return QString("object: %1").arg(QString::fromStdString(member.objectId));
        }
        if(!member.robotId.empty() && !member.linkName.empty()) {
            return QString("link: %1.%2")
                .arg(QString::fromStdString(member.robotId), QString::fromStdString(member.linkName));
        }
        if(!member.robotId.empty()) {
            return QString("robot: %1").arg(QString::fromStdString(member.robotId));
        }
        return "invalid member";
    }

    QString selectionSetName(
        const simulation_project::ProjectDocument& document,
        const std::string& selectionSetId)
    {
        const simulation_project::CollisionSelectionSetDesc* selectionSet =
            findCollisionSelectionSet(document, selectionSetId);
        if(selectionSet == nullptr) {
            return QString::fromStdString(selectionSetId);
        }
        return QString::fromStdString(selectionSet->name.empty() ? selectionSet->id : selectionSet->name);
    }

    QString generatorSideText(
        const std::string& robotId,
        const std::string& linkName,
        const std::string& objectId,
        const std::string& objectGroupId,
        const std::vector<std::string>& includeLinks)
    {
        if(!robotId.empty()) {
            if(!linkName.empty()) {
                return QString("link: %1.%2")
                    .arg(QString::fromStdString(robotId), QString::fromStdString(linkName));
            }
            if(!includeLinks.empty()) {
                return QString("robot: %1 [%2 link(s)]")
                    .arg(QString::fromStdString(robotId))
                    .arg(static_cast<qulonglong>(includeLinks.size()));
            }
            return QString("robot: %1").arg(QString::fromStdString(robotId));
        }
        if(!objectId.empty()) {
            return QString("object: %1").arg(QString::fromStdString(objectId));
        }
        if(!objectGroupId.empty()) {
            return QString("object group: %1").arg(QString::fromStdString(objectGroupId));
        }
        return "unresolved";
    }

    QString objectLikeGeneratorSideText(
        const simulation_project::ProjectDocument& document,
        const std::string& objectId,
        const std::string& objectGroupId)
    {
        if(isMountedAttachmentId(document, objectId)) {
            return QString("attachment: %1").arg(QString::fromStdString(objectId));
        }
        return generatorSideText({}, {}, objectId, objectGroupId, {});
    }

    void splitObjectLikeGeneratorId(
        const simulation_project::ProjectDocument& document,
        const std::string& id,
        std::string& objectId,
        std::string& attachmentId)
    {
        objectId.clear();
        attachmentId.clear();
        if(id.empty()) {
            return;
        }
        if(isMountedAttachmentId(document, id)) {
            attachmentId = id;
            return;
        }
        objectId = id;
    }

    CollisionDetectorPairItemView makePairItem(
        int index,
        const QString& bodyA,
        const QString& bodyB,
        const QString& source,
        const QString& expansion,
        const QString& filterState,
        int generatorIndex = -1)
    {
        CollisionDetectorPairItemView item;
        item.index = index;
        item.generatorIndex = generatorIndex;
        item.bodyA = bodyA;
        item.bodyB = bodyB;
        item.source = source;
        item.expansion = expansion;
        item.filterState = filterState;
        item.removable = generatorIndex >= 0;
        item.tooltip = QString("A: %1\nB: %2\nSource: %3\nExpansion: %4\nFilter: %5")
            .arg(bodyA, bodyB, source, expansion, filterState);
        return item;
    }

    void setPairSideA(
        CollisionDetectorPairItemView& item,
        const std::string& robotId,
        const std::string& linkName,
        const std::string& objectId,
        const std::string& attachmentId)
    {
        item.robotAId = QString::fromStdString(robotId);
        item.linkAName = QString::fromStdString(linkName);
        item.objectAId = QString::fromStdString(objectId);
        item.attachmentAId = QString::fromStdString(attachmentId);
    }

    void setPairSideB(
        CollisionDetectorPairItemView& item,
        const std::string& robotId,
        const std::string& linkName,
        const std::string& objectId,
        const std::string& attachmentId)
    {
        item.robotBId = QString::fromStdString(robotId);
        item.linkBName = QString::fromStdString(linkName);
        item.objectBId = QString::fromStdString(objectId);
        item.attachmentBId = QString::fromStdString(attachmentId);
    }

    void setPairSideA(
        CollisionDetectorPairItemView& item,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        setPairSideA(item, member.robotId, member.linkName, member.objectId, member.attachmentId);
    }

    void setPairSideB(
        CollisionDetectorPairItemView& item,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        setPairSideB(item, member.robotId, member.linkName, member.objectId, member.attachmentId);
    }

    const robot_qt_viewer::CollisionRuntimeDetectorInfo* findRuntimeInfo(
        const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors,
        const QString& detectorId)
    {
        const std::string id = detectorId.toStdString();
        const auto it = std::find_if(
            runtimeDetectors.begin(),
            runtimeDetectors.end(),
            [&](const robot_qt_viewer::CollisionRuntimeDetectorInfo& info) {
                return info.id == id;
            });
        return it == runtimeDetectors.end() ? nullptr : &(*it);
    }

    robot_qt_viewer::CollisionDetectorAddResult addCollisionDetector(
        simulation_project::ProjectDocument& document,
        simulation_project::CollisionDetectorDesc detector,
        const QString& failurePrefix,
        const QString& successPrefix)
    {
        std::string error;
        simulation_project::ProjectDocumentService service(document);
        if(!service.addCollisionDetector(detector, &error)) {
            robot_qt_viewer::CollisionDetectorAddResult result;
            result.message = QString("%1: %2").arg(failurePrefix, QString::fromStdString(error));
            return result;
        }

        robot_qt_viewer::CollisionDetectorAddResult result;
        result.success = true;
        result.detectorId = QString::fromStdString(detector.id);
        result.message = QString("%1 %2").arg(successPrefix, result.detectorId);
        return result;
    }
}

namespace robot_qt_viewer
{
    CollisionDetectorWorkbenchDocumentFacade::CollisionDetectorWorkbenchDocumentFacade(
        const simulation_project::ProjectDocument& document,
        CollisionWorkbenchServices& appServices)
        : m_document(document)
        , m_appServices(appServices)
    {
    }

    bool CollisionDetectorWorkbenchDocumentFacade::hasSceneEntities() const
    {
        return !m_document.robots.empty() || !m_document.objects.empty();
    }

    bool CollisionDetectorWorkbenchDocumentFacade::hasMultipleDetectors() const
    {
        return m_document.collision.detectors.size() > 1;
    }

    CollisionDetectorPreviewTarget CollisionDetectorWorkbenchDocumentFacade::previewTarget(
        const QString& detectorId) const
    {
        CollisionDetectorPreviewTarget target;
        const simulation_project::CollisionDetectorDesc* detector =
            findCollisionDetector(m_document, detectorId.toStdString());
        if(detector == nullptr || detector->selectionSetAId.empty()) {
            return target;
        }

        const simulation_project::CollisionSelectionSetDesc* selectionSet =
            findCollisionSelectionSet(m_document, detector->selectionSetAId);
        if(selectionSet == nullptr || selectionSet->members.empty()) {
            return target;
        }

        const simulation_project::CollisionSelectionSetMemberDesc& member = selectionSet->members.front();
        if(!member.objectId.empty()) {
            target.type = CollisionDetectorPreviewTarget::Type::SceneObject;
            target.objectId = QString::fromStdString(member.objectId);
        } else if(!member.robotId.empty()) {
            target.type = CollisionDetectorPreviewTarget::Type::RobotLink;
            target.robotId = QString::fromStdString(member.robotId);
            target.linkName = QString::fromStdString(member.linkName);
        } else if(!member.attachmentId.empty()) {
            target.type = CollisionDetectorPreviewTarget::Type::MountedAttachment;
            target.attachmentId = QString::fromStdString(member.attachmentId);
        }
        return target;
    }

    QVector<CollisionDetectorListItemView> CollisionDetectorWorkbenchDocumentFacade::detectorListItems(
        const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const
    {
        return CollisionDetectorsController::buildDetectorListItems(m_document, runtimeDetectors);
    }

    CollisionDetectorPropertiesView CollisionDetectorWorkbenchDocumentFacade::detectorProperties(
        const QString& detectorId) const
    {
        return CollisionDetectorsController::buildProperties(m_document, detectorId);
    }

    CollisionDetectorPairsViewModel CollisionDetectorWorkbenchDocumentFacade::detectorPairs(
        const QString& detectorId,
        const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const
    {
        CollisionDetectorPairsViewModel view;
        const simulation_project::CollisionDetectorDesc* detector =
            findCollisionDetector(m_document, detectorId.toStdString());
        if(detector == nullptr) {
            view.summary = "No detector selected.";
            return view;
        }

        (void)runtimeDetectors;
        view.summary.clear();

        int index = 0;
        if(detector->queryPolicy == "AllEnabled" || detector->type == "SceneAll") {
            CollisionDetectorPairItemView item = makePairItem(
                index++,
                "all enabled collision bodies",
                "all enabled collision bodies",
                "SceneAll",
                "broad phase resolves pairs at runtime",
                "project/global filters apply");
            item.clearsScope = true;
            view.items.push_back(std::move(item));
        } else if(detector->queryPolicy == "WithinSet") {
            const simulation_project::CollisionSelectionSetDesc* setA =
                findCollisionSelectionSet(m_document, detector->selectionSetAId);
            const QString setName = selectionSetName(m_document, detector->selectionSetAId);
            if(setA != nullptr) {
                for(std::size_t i = 0; i < setA->members.size(); ++i) {
                    for(std::size_t j = i + 1; j < setA->members.size(); ++j) {
                        CollisionDetectorPairItemView item = makePairItem(
                            index++,
                            memberText(setA->members[i]),
                            memberText(setA->members[j]),
                            QString("WithinSet: %1").arg(setName),
                            "member combination",
                            "detector filters apply");
                        setPairSideA(item, setA->members[i]);
                        setPairSideB(item, setA->members[j]);
                        view.items.push_back(std::move(item));
                    }
                }
            }
        } else if(detector->queryPolicy == "BetweenSets") {
            const simulation_project::CollisionSelectionSetDesc* setA =
                findCollisionSelectionSet(m_document, detector->selectionSetAId);
            const simulation_project::CollisionSelectionSetDesc* setB =
                findCollisionSelectionSet(m_document, detector->selectionSetBId);
            const QString source = QString("BetweenSets: %1 -> %2")
                .arg(selectionSetName(m_document, detector->selectionSetAId))
                .arg(selectionSetName(m_document, detector->selectionSetBId));
            if(setA != nullptr && setB != nullptr) {
                for(const simulation_project::CollisionSelectionSetMemberDesc& a : setA->members) {
                    for(const simulation_project::CollisionSelectionSetMemberDesc& b : setB->members) {
                        CollisionDetectorPairItemView item = makePairItem(
                            index++,
                            memberText(a),
                            memberText(b),
                            source,
                            "cartesian product",
                            "detector filters apply");
                        setPairSideA(item, a);
                        setPairSideB(item, b);
                        view.items.push_back(std::move(item));
                    }
                }
            }
        }

        int generatorIndex = 0;
        for(const simulation_project::CollisionPairGeneratorDesc& generator : detector->pairGenerators) {
            QString bodyA;
            QString bodyB;
            std::string robotAId;
            std::string linkAName;
            std::string objectAId;
            std::string attachmentAId;
            std::string robotBId;
            std::string linkBName;
            std::string objectBId;
            std::string attachmentBId;
            if(generator.type == "LinkLink" || generator.type == "RobotRobot") {
                bodyA = generatorSideText(
                    generator.robotA,
                    generator.linkA,
                    {},
                    {},
                    generator.includeLinksA);
                bodyB = generatorSideText(
                    generator.robotB,
                    generator.linkB,
                    {},
                    {},
                    generator.includeLinksB);
                robotAId = generator.robotA;
                linkAName = !generator.linkA.empty()
                    ? generator.linkA
                    : (!generator.includeLinksA.empty() ? generator.includeLinksA.front() : std::string());
                robotBId = generator.robotB;
                linkBName = !generator.linkB.empty()
                    ? generator.linkB
                    : (!generator.includeLinksB.empty() ? generator.includeLinksB.front() : std::string());
            } else if(generator.type == "LinkObject" || generator.type == "RobotObject") {
                bodyA = generatorSideText(
                    generator.robotId,
                    generator.linkName,
                    {},
                    {},
                    generator.includeLinksA);
                bodyB = objectLikeGeneratorSideText(m_document, generator.objectId, generator.objectGroupId);
                robotAId = generator.robotId;
                linkAName = !generator.linkName.empty()
                    ? generator.linkName
                    : (!generator.includeLinksA.empty() ? generator.includeLinksA.front() : std::string());
                splitObjectLikeGeneratorId(m_document, generator.objectId, objectBId, attachmentBId);
            } else if(generator.type == "ObjectObject") {
                bodyA = objectLikeGeneratorSideText(m_document, generator.objectA, {});
                bodyB = objectLikeGeneratorSideText(m_document, generator.objectB, {});
                splitObjectLikeGeneratorId(m_document, generator.objectA, objectAId, attachmentAId);
                splitObjectLikeGeneratorId(m_document, generator.objectB, objectBId, attachmentBId);
            } else if(generator.type == "SceneAll") {
                bodyA = "all enabled collision bodies";
                bodyB = "all enabled collision bodies";
            } else {
                bodyA = QString::fromStdString(generator.type);
                bodyB = "generator-defined target";
            }

            CollisionDetectorPairItemView item = makePairItem(
                index++,
                bodyA,
                bodyB,
                QString("generator: %1").arg(QString::fromStdString(generator.type)),
                "expands to collision model pairs at runtime",
                "detector filters apply",
                generatorIndex++);
            setPairSideA(item, robotAId, linkAName, objectAId, attachmentAId);
            setPairSideB(item, robotBId, linkBName, objectBId, attachmentBId);
            view.items.push_back(std::move(item));
        }

        for(const simulation_project::CollisionPairFilterDesc& filter : detector->pairFilters) {
            view.items.push_back(makePairItem(
                index++,
                QString::fromStdString(filter.robotA.empty() ? filter.robotId : filter.robotA),
                QString::fromStdString(filter.robotB),
                QString("filter: %1").arg(QString::fromStdString(filter.type)),
                QString::fromStdString(filter.policy),
                "filter row, not a pair"));
        }

        return view;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::setDetectorEnabled(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        bool enabled)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::setDetectorEnabled(
                    service.document(),
                    viewportServices,
                    detectorId,
                    enabled);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::applyDetectorProperties(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const CollisionDetectorPropertiesView& properties)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::applyDetectorProperties(
                    service.document(),
                    viewportServices,
                    detectorId,
                    properties);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::applyDetectorQueryContract(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const CollisionDetectorQueryContractView& contract)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::applyDetectorQueryContract(
                    service.document(),
                    viewportServices,
                    detectorId,
                    contract);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::showOnlyDetectors(
        RobotQtViewerViewportServices* viewportServices,
        const QVector<QString>& detectorIds)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::showOnlyDetectors(
                    service.document(),
                    viewportServices,
                    detectorIds);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::removeDetector(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::removeDetector(
                    service.document(),
                    viewportServices,
                    detectorId);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::bindDetectorDraftSets(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const QVector<CollisionDetectorDraftMemberView>& setA,
        const QVector<CollisionDetectorDraftMemberView>& setB)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::bindDetectorPairGenerators(
                    service.document(),
                    viewportServices,
                    detectorId,
                    setA,
                    setB);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::removeDetectorPairGenerators(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const QVector<int>& generatorIndexes)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::removeDetectorPairGenerators(
                    service.document(),
                    viewportServices,
                    detectorId,
                    generatorIndexes);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorCommandResult CollisionDetectorWorkbenchDocumentFacade::clearDetectorPairScope(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId)
    {
        CollisionDetectorCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionDetectorCommandController::clearDetectorPairScope(
                    service.document(),
                    viewportServices,
                    detectorId);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionDetectorAddResult CollisionDetectorWorkbenchDocumentFacade::addTaskPanelDefaultDetector()
    {
        CollisionDetectorAddResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = addCollisionDetector(
                    service.document(),
                    makeTaskPanelDefaultDetector(service.document()),
                    "Add detector failed",
                    "Added collision detector");
                changed = result.success;
                return result.success;
            });
        return result;
    }

    CollisionDetectorAddResult CollisionDetectorWorkbenchDocumentFacade::addSceneAllDetector()
    {
        CollisionDetectorAddResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionDetector"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                simulation_project::CollisionDetectorDesc detector;
                detector.id = makeUniqueCollisionDetectorId(service.document(), "scene_all");
                detector.name = "Scene All";
                detector.type = "SceneAll";
                detector.queryPolicy = "AllEnabled";
                applyDocumentDefaults(service.document(), detector);
                addSceneAllTargets(service.document(), detector);

                result = addCollisionDetector(
                    service.document(),
                    detector,
                    "Add SceneAll failed",
                    "Added SceneAll detector");
                changed = result.success;
                return result.success;
            });
        return result;
    }

}
