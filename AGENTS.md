# AGENTS.md

Working contract for AI agents on this C++ industrial robot simulation project. Follow direct user instructions without silently compromising correctness, buildability, data safety, or ownership boundaries.

---

## 0. Lightweight Task Fast Path

For small, explicit, low-risk tasks, act directly without presenting a plan or asking for confirmation.

Examples:
- Edit a few Markdown or text files.
- Fix wording, formatting, comments, or documentation.
- Make a localized code change with no public API, dependency, module, build, serialization, or schema impact.
- Run read-only inspection commands in the workspace.

Apply the change first and report it afterward. Ask only when a material ambiguity prevents a correct implementation. This fast path overrides later planning rules unless correctness, compatibility, architecture, or data safety is genuinely at risk.

---

## 1. Priorities

Use this order when tradeoffs appear:
1. Correctness, buildability, and data safety.
2. Correct ownership layer and stable architecture.
3. Minimal effective change at the owning layer.
4. Clear module boundaries and reversible diffs.
5. Readable code and useful documentation.

“Minimal” means the smallest change that fixes the real invariant, not necessarily the smallest local patch. Prefer one central rule over repeated UI or caller-side checks.

---

## 2. Project Context

- Language: C++17 by default. Do not introduce C++20 unless already required.
- Build: target-based CMake.
- Platform: Windows and Visual Studio; preserve Debug/Release separation and DLL behavior.
- Main packages:
  - `Common`: shared utilities, logging, and runtime support.
  - `SMRobotCore`: robot model, IO, runtime, kinematics, collision, and SDK.
  - `SMRobotPlatform`: assets, rendering, scene, camera, projects, bridges, and platform runtime.
  - `SMRobotApps`: viewers, quick starts, and application integration.

Known modules include `RobotCore`, `RobotIO`, `RobotRuntime`, `RobotInstance`, `Kinematics`, `Collision`, `RobotSDK`, `RobotTrajectoryCore`, `AssetCore`, `RenderCore`, `SceneCore`, `CameraCore`, `RobotRenderBridge`, `SimulationProject`, `SimulationRuntime`, `SensorCore`, `SensorSimulation`, `RobotViewerCore`, `RobotQtViewer`, and `RobotGlfwViewer`.

Do not reorganize repositories, rename modules, move files, or change public APIs unless the task requires it.

---

## 3. Rebuild Current Understanding

Inspect current source whenever code may have changed. Do not rely only on old plans or conversation history.

For large, unclear, or architectural work, create a short Chinese snapshot in `docs/agent_project_snapshot.md` covering:
- branch and task goal;
- affected modules and ownership;
- relevant `git status`, diff, and build state;
- safe extension points, risks, and verification commands.

Read in this order:
1. Root `AGENTS.md`.
2. `docs/agent_project_snapshot.md`.
3. `docs/agent_change_audit.md`.
4. Relevant contracts in `docs/architecture/`.
5. The plan explicitly named by the user.

Source code and this contract override stale documents. Record meaningful discrepancies before coding.

---

## 4. Root Cause and Ownership

Before editing, determine:
- the reported symptom;
- the invariant that must always hold;
- the module that owns it;
- current bypass paths;
- whether it must also work in GLFW, SDK, tests, or headless runtime.

A local fix is appropriate only when behavior belongs to one component and does not affect persistent state, runtime semantics, file IO, selection, collision, save/load, undo/redo, or public SDK behavior.

A structural fix is required when the rule spans views, commands, import/export paths, project state, runtime state, or non-Qt consumers. Add the smallest central API at the owning layer, then migrate callers. Do not duplicate equivalent checks across widgets or callbacks.

Typical owners:
- persistent data: project document/session/service;
- runtime behavior: runtime, trajectory, collision, or domain service;
- visualization mapping: bridge or view-model layer;
- file IO and dirty state: project/session/IO layer;
- selection and transform propagation: scene/project/runtime layer.

---

## 5. Planning

Create a short Chinese plan before coding only when:
- multiple modules are affected;
- a public API, dependency, schema, serialization format, or build configuration may change;
- the likely diff exceeds eight source/config files;
- the user requests a plan;
- inspection alone is insufficient for safe implementation.

Useful plan contents:
- goal and non-goals;
- owning invariant and affected modules;
- expected behavior and verification;
- major risks and stop conditions.

Do not create plan files unless requested. Documentation-only and localized fixes do not need a plan.

For long autonomous work, define scope, completion criteria, verification, change budget, and stop conditions. Stop for user direction only when needed, such as a necessary public API rename, new dependency, uncertain behavior preservation, substantial scope expansion, or repeated unrelated build failures.

Default stop conditions for risky work:
- more than 50 source/config files appear necessary;
- a public API rename or new module dependency becomes necessary;
- a new third-party dependency becomes necessary;
- two consecutive build attempts fail for different reasons;
- behavior or compatibility cannot be preserved confidently.

When stopping, report completed changes, why the condition matters, current verification, and practical next options.

---

## 6. Document, View, and Command Architecture

Persistent project behavior follows this flow:

```text
User intent -> command/mutation -> document/session/runtime state
            -> registered view notification -> view model -> repaint
```

- Document/session owns persistent truth.
- Runtime owns execution state.
- Commands/services perform mutations.
- Views display state and emit user intent.
- Widgets may own transient UI state only.

Do not repair project-visible behavior by directly pushing ad hoc state into multiple widgets, tree items, or viewport flags. Express the rule in the owning data structure, mutate it through the correct service, and refresh registered views from snapshots or view models.

Use existing mechanisms such as `RobotQtViewerDocumentController`, `RobotQtViewerEventHub`, `RobotQtViewerDocumentViewRegistry`, and module controllers. Add the smallest typed registration/notification path when none exists.

`MainWindow` may own menus, docks, dialogs, top-level signal wiring, and temporary UI state. It must not own project mutation semantics, dirty rules, save/load invariants, robot/tool/sensor semantics, collision policy, or runtime execution.

For display or interaction defects, explicitly identify:
- the document/runtime state that should express the result;
- the command or mutation that changes it;
- the registered views that must be notified;
- the view model that projects the state;
- any widget currently compensating for missing ownership.

Before changing GUI document state, workbench modes, selection, binding, view models, task panels, or viewport visibility, read the relevant architecture contracts, especially:
- `docs/architecture/document_view_gui_contract.md`;
- `docs/architecture/simulation_platform_document_workbench_contract.md`.

---

## 7. Module Boundaries

- `AssetCore`: CPU-side asset loading and descriptions; no OpenGL, SceneCore, or robot runtime semantics.
- `RenderCore`: GPU resources; no robot, project document, or Qt semantics.
- `SceneCore`: scene graph, transforms, traversal, and render flow; no robot-specific logic.
- `RobotCore`: robot model and structural data; no rendering, Qt, or OpenGL.
- `RobotIO`: format loading into model data; no rendering or UI ownership.
- `RobotRuntime`: runtime and execution state; no UI ownership.
- `RobotInstance`: runtime robot instances and transforms; no low-level rendering.
- `RobotRenderBridge`: robot/runtime-to-scene visualization; no ownership of core robot semantics.
- `Collision`: geometry, distance, contacts, self-collision, and filtering; independent of UI/rendering.
- `CameraCore` and `Kinematics`: independent modules without UI or rendering dependencies unless explicitly designed.

If a boundary violation is discovered, explain it and choose the smallest correct repair.

---

## 8. Application, Rendering, and Runtime Rules

Viewer applications are UI adapters. Replacing Qt with GLFW should require new controls, not rewritten project, robot, collision, or runtime semantics.

Long-lived project entities and operations belong in Core or Platform. Mounted tools, sensors, fixtures, and attachments should participate in selection, transforms, rendering, collision, save/load, and non-Qt viewers through stable APIs.

Rendering rules:
- robot modules do not own GPU resources or include OpenGL headers;
- `RenderCore` owns GPU resources;
- `SceneCore` owns scene rendering flow;
- bridges map domain data to visualization.

Collision rules:
- collision behavior remains independent of UI and rendering;
- visualization goes through SceneCore or bridges;
- filtering must not be hidden in UI code;
- distance/contact APIs should be usable from tests and headless tools.

---

## 9. File Organization and Style

- Public headers: `include/<Module>/`.
- Keep private headers and sources in the existing module convention.
- Prefer `.h` and `.cpp`; avoid non-trivial header-only implementations.
- Use forward declarations where practical.
- Do not place `.cpp` files under `include/`.
- Check existing extension points before adding files or classes.

Style:
- classes: `PascalCase`;
- functions: `camelCase`;
- data members: `m_xxx`;
- prefer RAII, smart pointers, and simple data structures;
- avoid global mutable state, speculative abstractions, and unnecessary inheritance/templates.

Keep source, CMake, identifiers, and comments ASCII unless the file already uses non-ASCII text or the user requests it. Do not use emoji in project artifacts.

---

## 10. CMake and Dependencies

- Use target-based CMake.
- Do not modify unrelated targets.
- Avoid global include/link directories.
- Keep Debug/Release behavior explicit.
- Do not hardcode local machine paths; use cache variables, imported targets, or user profiles.
- Do not add dependencies unless explicitly requested. If unavoidable, stop and explain why.

On Windows, verify DLL deployment and configuration-specific imported locations. A private runtime dependency must be deployed without unnecessarily becoming a public link dependency.

---

## 11. Change Protocol

For non-trivial changes, briefly state affected files/modules, the owning invariant, and verification before editing. Skip this preamble for small documentation or localized changes.

During work:
- modify only relevant files;
- preserve unrelated user changes;
- avoid unrelated formatting;
- preserve naming, style, and line endings;
- do not claim success without applicable build/test evidence.

For filesystem and Git safety:
- resolve exact targets before recursive delete or move operations;
- keep destructive targets inside the intended workspace/build directory;
- never use broad roots, unresolved variables, or unverified globs;
- do not use `git reset --hard` or discard user changes;
- prefer recoverable operations and report material deletions.

After work, report:
- modified files and purpose;
- build/test results;
- known limitations;
- public API or dependency changes.

If the worktree already contains overlapping changes, inspect `git status` and `git diff --stat`, then audit only affected modules. For genuinely large or risky work, update `docs/agent_change_audit.md` in Chinese when it will help future work.

---

## 12. Verification

Prefer verification in this order:
1. CMake configure.
2. Build the affected target.
3. Build the main application.
4. Run unit tests or examples.
5. Run the viewer or packaged executable when runtime behavior matters.

Useful commands:

```bat
git status
git diff --stat
git diff --name-only
git diff -- <file>
cmake --preset <preset-name>
cmake --build --preset <build-preset-name>
cmake --build <build-dir> --target <target-name> --config Debug
```

Adapt commands to real presets and targets. If verification cannot run, state the exact missing check and reason.

Do not invent successful results. On Windows, verify both the configuration that changed and the configuration likely to differ in runtime naming or deployment.

---

## 13. Code Retirement

Control code growth. Do not keep old functions, wrappers, flags, or adapters merely to avoid deletion.

Classify old code:
- **Active**: used by runtime, targets, tests, examples, public APIs, or supported files.
- **Compatibility**: temporarily required for public API or file migration.
- **Deprecated**: replaced but still referenced; add no new callers.
- **Dead**: unreferenced by builds, tests, APIs, IO, or supported workflows.

Before deletion:
1. Search code, CMake, tests, examples, public headers, project IO, registrations, and string-based factories.
2. Confirm whether compatibility or public API is involved.
3. Migrate callers first.
4. Delete files/functions only after references are gone.
5. Update build files, docs, examples, and tests.
6. Build affected targets.

Use `rg` and `git grep` for reference checks. Keep compatibility paths narrow, route them to the new implementation, document their reason, and define a removal condition.

Also inspect reflection-like string names, factory maps, Qt signal/slot references, plugin registration, and serialization migration paths before declaring code dead.

For broad cleanup, group candidates as: delete now, migrate then delete, keep for compatibility, and unknown/risky.

---

## 14. Generated, Legacy, and Forbidden Changes

Do not modify generated, archived, deprecated, or legacy directories unless required. Preserve supported project-file compatibility.

Unless explicitly required, do not:
- perform unrelated refactors;
- rename public APIs;
- change module dependencies;
- add third-party dependencies;
- move or reorganize files;
- create parallel implementations;
- reformat unrelated files;
- convert unrelated line endings.

If one is necessary for correctness, explain the reason, alternatives, impact, and verification. Stop only when user input is genuinely required.

---

## 15. Documentation and Line Endings

Responses should be concise, engineering-focused, and Chinese by default. Agent-created plans, audits, and architecture notes under `plans/` and `docs/` should be Chinese unless the user requests otherwise. Keep identifiers, paths, commands, and API names unchanged.

This repository primarily uses Visual Studio. Preserve each file’s existing line endings. C/C++/CMake/Qt project files (`.h`, `.hpp`, `.cpp`, `.c`, `.cxx`, `.cmake`, `CMakeLists.txt`, `.ui`, `.qrc`) use CRLF. Never mix LF and CRLF in one file or rewrite unrelated files solely to change line endings.

---

## 16. Packaging and Release Gate

When asked to package, export an SDK, or create a release, do not treat configure success, compilation, or ZIP creation alone as completion.

Recurring failure classes:
- dependency roots were requested even when user profiles could resolve them;
- ABI snapshots failed late because exports changed or install prefixes were stale;
- full `cmake --install` polluted thin SDKs with development dependencies;
- source and prebuilt packages were duplicated;
- private SDKs, platform-specific files, symbols, or sensitive files leaked;
- private runtime DLLs existed in the package but were not deployed to applications;
- archive size grew unexpectedly;
- rebuilt artifacts left stale manifests or hashes;
- version and output paths diverged;
- long Windows paths left stale staging directories.

Before packaging:
1. Inspect worktree changes and confirm version, platform, output, license, user profile, and dependency roots.
2. Use a clean, named SDK install profile; never substitute a full install for a thin release.
3. If public headers, exports, packages, or install logic changed, run ABI snapshot comparison and the consumer matrix first.
4. Confirm private runtime metadata exists without turning private dependencies into public link requirements.
5. Record the previous artifact size or an expected size budget.

After packaging:
1. Build the packaged consumer in Debug and Release from an independent build directory.
2. Compare declared runtime DLLs with application output and run a release smoke test.
3. Audit package allowlists/denylists, sensitive files, duplicate source/prebuilt content, and internal-only targets.
4. Report archive and major-directory sizes; investigate material growth before delivery.
5. Recompute every artifact’s size and SHA256, then verify sidecars, manifest, and checksum index from disk.
6. Confirm version consistency and remove only validated generated staging leftovers.

Prefer the repository’s release orchestrator and profile installer over manual command sequences. Do not update an ABI baseline merely to silence a failure; classify and approve the change first. Final reporting must include artifact paths, sizes, hashes, build/smoke results, ABI/consumer status, content audit, and unresolved warnings. Never report an unchecked item as passed.

---

## END
