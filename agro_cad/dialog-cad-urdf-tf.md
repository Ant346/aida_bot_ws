# Диалог: CAD → URDF / TF (геометрический центр)

Дата: 2026-10-05  
Проект: agrobot (фпкщище)

---

## Цель

Перенести из CAD (Fusion) сведения о роботе, креплениях и камерах в URDF/TF, чтобы получить преобразования из систем координат камер в СК робота — без мокапа и сложных технологий. Дополнительно — докалибровка по ArUco (обе камеры видят доску).

Целевой кадр: **геометрический центр** между осями робота.

### Шесть систем координат

1. Три СК для ZED Mini  
2. Одна СК для RealSense D405  
3. Две СК робота:
   - геометрический центр между осями (`base_link`)
   - выпирающая передняя часть — первая точка контакта при движении прямо (`front_hit`)

Нужны преобразования **из всего в геометрический центр**.

---

## Сообщения

### Ассистент — старт Project

Welcome to your first Project. We can work together here for weeks or months on bigger goals — not just one-off chats.

- **Memory**: I learn from your feedback and carry context forward as we go
- **Local or cloud**: I can work on my own computers or on yours, depending on what fits
- **Pace**: I’ll keep moving in the background and check in when something needs you

What do you want to accomplish first? Drop in a goal, a file, or an existing chat and we’ll take it from there.

---

### Пользователь

Ты сможешь сделать TF из модели fusion по СК?

---

### Ассистент

Да, смогу. Скорее всего речь про TF/трансформы из fusion-модели в нужной системе координат.

Чтобы стартовать без лишних кругов, пришли:

1. Что за fusion-модель (формат, откуда берём позы/оценки)
2. Какая СК нужна на выходе (и относительно чего)
3. Куда это должно встроиться — ROS TF2, свой пайплайн, или просто матрицы/файлы

Можно скинуть файлы, чат или репо — разберём и сделаем.

---

### Пользователь

У меня есть очень достоверная 3Д модель робота, креплений для камер и самих камер. Я хочу перенести эти сведения из CAD в URDF, чтобы решить задачу преобразования из СК камер в СК робота без применения мокапа или других сложных технологий.

Кроме того, у меня есть аруко доска и обе камеры смогут ее увидеть, поэтому можно докалиброваться.

Сейчас напиши в каком формате тебе отдать модель или может просто апи и ты сам все посчитаешь.

Есть 6 ск:

- 3 для zedm
- 1 405 realsense
- 2 робота — геометрический центр между его осями и выпирающая передняя часть, которой он в первую очередь ударится если поедет прямо

Хочу преобразование из всего в геометрический центр.

---

### Ассистент

Считать буду я. Из Fusion нужен JSON со осями шести систем координат — его снимает скрипт ниже. STEP приложи, если нужен ещё и URDF с геометрией; для самих преобразований хватает JSON.

Цель: родитель всех TF — геометрический центр между осями. Туда же приведу переднюю точку и все камеры.

Читаю шесть СК так: три на одном ZED Mini (лево, право, центр или IMU), одна на RealSense D405, две на роботе (геометрический центр и передняя точка, которой он ударится при движении прямо). Если ZED Mini на самом деле три штуки — поправь.

**Как подписать кадры**

- `base_link` — геометрический центр. Оси ROS: X вперёд, Y влево, Z вверх.
- `front_hit` — первая точка контакта. Оси параллельны `base_link`.
- `zedm_left_optical`, `zedm_right_optical`, `zedm_center` — оптика: X вправо, Y вниз, Z из линзы.
- `d405_optical` — то же для D405.

Если системы уже стоят иначе, не переделывай. В JSON подпиши, какая ось куда, я приведу к ROS.

Каждая из шести СК должна быть началом координат своего компонента. Пустой компонент тоже подходит. Скрипт читает origin компонента в координатах корня сборки.

**Скрипт (версия 1 — origin компонентов)**

В Fusion открой дизайн, затем Scripts and Add-Ins → Create → Python, вставь и Run. Он сохранит `frames.json`. Внутренние сантиметры Fusion уже переведены в миллиметры.

```python
import adsk.core, adsk.fusion, traceback, json

def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Открой дизайн Fusion, не чертеж.')
            return
        root = design.rootComponent
        frames = []

        def add(name, path, origin, xdir, ydir, zdir):
            frames.append({
                'name': name,
                'path': path,
                'origin_mm': [origin.x * 10.0, origin.y * 10.0, origin.z * 10.0],
                'x_axis': [xdir.x, xdir.y, xdir.z],
                'y_axis': [ydir.x, ydir.y, ydir.z],
                'z_axis': [zdir.x, zdir.y, zdir.z],
            })

        add(root.name, root.name,
            root.originConstructionPoint.geometry,
            root.xConstructionAxis.geometry.direction,
            root.yConstructionAxis.geometry.direction,
            root.zConstructionAxis.geometry.direction)

        for occ in root.allOccurrences:
            comp = occ.component
            try:
                o = comp.originConstructionPoint.createForAssemblyContext(occ).geometry
                x = comp.xConstructionAxis.createForAssemblyContext(occ).geometry.direction
                y = comp.yConstructionAxis.createForAssemblyContext(occ).geometry.direction
                z = comp.zConstructionAxis.createForAssemblyContext(occ).geometry.direction
            except:
                continue
            add(comp.name, occ.fullPathName, o, x, y, z)

        payload = {
            'units': 'mm',
            'cad': 'fusion360',
            'root_component': root.name,
            'frames': frames,
        }
        dlg = ui.createFileDialog()
        dlg.title = 'Save frame dump'
        dlg.filter = 'JSON (*.json)'
        dlg.initialFilename = 'frames.json'
        if dlg.showSave() != adsk.core.DialogResults.DialogOK:
            return
        with open(dlg.filename, 'w', encoding='utf-8') as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        ui.messageBox('Сохранено кадров: {}\n{}'.format(len(frames), dlg.filename))
    except:
        ui.messageBox(traceback.format_exc())
```

Вместе с файлом одной строкой напиши, какой `path` какому кадру соответствует. Проверка: расстояние между левым и правым оптическим центром ZED Mini около 63 мм — столько же даст Measure в Fusion.

ArUco потом поправит камеру относительно камеры: ошибку установки и сдвиг оптического центра. Геометрический центр и переднюю точку доска не видит, их берём из CAD.

---

### Пользователь

Каждая из шести СК должна быть началом координат своего компонента. Пустой компонент тоже подходит. Скрипт читает origin компонента в координатах корня сборки.

У меня не так, я отдельные ск создавал.

---

### Ассистент

Компоненты трогать не нужно. Скрипт читает отдельные пользовательские системы координат и переводит их в координаты корня сборки, даже если они лежат внутри подсборок.

В Fusion: Scripts and Add-Ins → Create → Python, вставь и Run. Сохранится `frames.json`. Имена СК из браузера попадут в поле `name`.

**Скрипт (версия 2 — пользовательские СК / UCS)**

```python
import adsk.core, adsk.fusion, traceback, json

def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Открой дизайн Fusion, не чертеж.')
            return
        root = design.rootComponent
        if not hasattr(root, 'userCoordinateSystems'):
            ui.messageBox('Нужен Fusion от мая 2026 или новее: API пользовательских СК в этой сборке нет.')
            return

        frames = []

        def pose(ucs, occ):
            if occ:
                origin = ucs.originConstructionPoint.createForAssemblyContext(occ).geometry
                x = ucs.xConstructionAxis.createForAssemblyContext(occ).geometry.direction
                y = ucs.yConstructionAxis.createForAssemblyContext(occ).geometry.direction
                z = ucs.zConstructionAxis.createForAssemblyContext(occ).geometry.direction
            else:
                origin = ucs.originConstructionPoint.geometry
                x = ucs.xConstructionAxis.geometry.direction
                y = ucs.yConstructionAxis.geometry.direction
                z = ucs.zConstructionAxis.geometry.direction
            return origin, x, y, z

        def add(ucs, occ, path):
            native = ucs.nativeObject if ucs.nativeObject else ucs
            origin, x, y, z = pose(native, occ)
            frames.append({
                'name': native.name,
                'path': path,
                'component': native.parentComponent.name,
                'origin_mm': [origin.x * 10.0, origin.y * 10.0, origin.z * 10.0],
                'x_axis': [x.x, x.y, x.z],
                'y_axis': [y.x, y.y, y.z],
                'z_axis': [z.x, z.y, z.z],
            })

        for i in range(root.userCoordinateSystems.count):
            ucs = root.userCoordinateSystems.item(i)
            add(ucs, None, root.name + '/' + ucs.name)

        for occ in root.allOccurrences:
            comp = occ.component
            collection = comp.userCoordinateSystems
            for i in range(collection.count):
                ucs = collection.item(i)
                native = ucs.nativeObject if ucs.nativeObject else ucs
                add(native, occ, occ.fullPathName + '/' + native.name)

        if not frames:
            ui.messageBox('Пользовательских СК не найдено. Проверь, что они есть в браузере дизайна.')
            return

        payload = {
            'units': 'mm',
            'cad': 'fusion360',
            'root_component': root.name,
            'frames': frames,
        }
        dlg = ui.createFileDialog()
        dlg.title = 'Save frame dump'
        dlg.filter = 'JSON (*.json)'
        dlg.initialFilename = 'frames.json'
        if dlg.showSave() != adsk.core.DialogResults.DialogOK:
            return
        with open(dlg.filename, 'w', encoding='utf-8') as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        names = '\n'.join(f['name'] for f in frames)
        ui.messageBox('Сохранено СК: {}\n{}\n\n{}'.format(len(frames), dlg.filename, names))
    except:
        ui.messageBox(traceback.format_exc())
```

Пришли `frames.json` и одной строкой, какое `name` какой кадр: геометрический центр, передняя точка, три СК ZED Mini, D405. Расстояние между левой и правой оптикой ZED Mini в файле должно быть около 63 мм.

---

### Пользователь

выгрузи в md весь диалог в D:\PhD\папир

---

### Ассистент

Файл сохранён по запрошенному пути (или в Context, если локальный доступ был недоступен). См. актуальный статус в Project Notes.
