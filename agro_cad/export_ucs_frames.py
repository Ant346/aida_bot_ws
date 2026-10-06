# Fusion 360 — Scripts and Add-Ins → Create → Python, вставить и Run.
# Пишет frames.json: пользовательские СК в координатах корня сборки, мм.
import adsk.core
import adsk.fusion
import traceback
import json


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Открой дизайн Fusion, не чертеж.")
            return
        root = design.rootComponent
        if not hasattr(root, "userCoordinateSystems"):
            ui.messageBox("Нужен Fusion от мая 2026 или новее: API пользовательских СК нет.")
            return

        frames = []

        def vec(v):
            return [v.x, v.y, v.z]

        def add(ucs, occ, path):
            native = ucs.nativeObject if ucs.nativeObject else ucs
            origin = native.originConstructionPoint.geometry.copy()
            x = native.xConstructionAxis.geometry.direction.copy()
            y = native.yConstructionAxis.geometry.direction.copy()
            z = native.zConstructionAxis.geometry.direction.copy()
            root_translation_mm = [0.0, 0.0, 0.0]
            if occ is not None:
                world = occ.transform2
                origin.transformBy(world)
                x.transformBy(world)
                y.transformBy(world)
                z.transformBy(world)
                root_translation_mm = [
                    world.getCell(0, 3) * 10.0,
                    world.getCell(1, 3) * 10.0,
                    world.getCell(2, 3) * 10.0,
                ]
            frames.append(
                {
                    "name": native.name,
                    "path": path,
                    "component": native.parentComponent.name,
                    "space": "root",
                    "origin_mm": [origin.x * 10.0, origin.y * 10.0, origin.z * 10.0],
                    "x_axis": vec(x),
                    "y_axis": vec(y),
                    "z_axis": vec(z),
                    "occurrence_translation_mm": root_translation_mm,
                }
            )

        for i in range(root.userCoordinateSystems.count):
            ucs = root.userCoordinateSystems.item(i)
            add(ucs, None, root.name + "/" + ucs.name)

        for occ in root.allOccurrences:
            collection = occ.component.userCoordinateSystems
            for i in range(collection.count):
                ucs = collection.item(i)
                native = ucs.nativeObject if ucs.nativeObject else ucs
                add(native, occ, occ.fullPathName + "/" + native.name)

        if not frames:
            ui.messageBox("Пользовательских СК не найдено.")
            return

        payload = {
            "units": "mm",
            "cad": "fusion360",
            "space": "root",
            "root_component": root.name,
            "frames": frames,
        }
        dlg = ui.createFileDialog()
        dlg.title = "Save frame dump"
        dlg.filter = "JSON (*.json)"
        dlg.initialFilename = "frames.json"
        if dlg.showSave() != adsk.core.DialogResults.DialogOK:
            return
        with open(dlg.filename, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        names = "\n".join(f["path"] for f in frames)
        ui.messageBox("Сохранено СК: {}\n{}\n\n{}".format(len(frames), dlg.filename, names))
    except:
        ui.messageBox(traceback.format_exc())
