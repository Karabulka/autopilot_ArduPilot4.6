# Размещение файлов проекта

Корень — существующий репозиторий PX4. Посторонние исходники и документы
PX4 не переносились и не удалялись.

- src/glider_visual_nav/include, src: математическое ядро и узлы C++.
- src/glider_visual_nav/tools: исполняемые средства запуска, мост и оценщики.
- src/glider_visual_nav/test: тесты; config: параметры; launch: ROS launch.
- src/glider_visual_nav/simulation: модели, миры и исходники систем Gazebo.
- docs/ALIGNMENT_DIAGNOSIS_2026-11-12.md: текущая диагностика точности.
- docs/results: компактные машиночитаемые результаты проверок.
- log_visual_nav: сырая телеметрия и журналы, не коммитятся.
- build_visual_nav и install_visual_nav: сборка, зависимости и установка.
- build_visual_nav/publication.CSNoTY: отдельный репозиторий публикации.

Для новых отчетов использовать тематическое имя с датой, данные хранить
в docs/results, большие записи только в log_visual_nav. Не создавать
дубликаты исходников в корне. Desktop HTML/TXT — экспорт для чтения.

Текущая приемка — ACCEPTANCE_2026-11-12.md.
Экспорт desktop: tools/export_documents.py DESTINATION.
Объединитель критериев: tools/build_acceptance_report.py.

Выбор носителя и закупка — HARDWARE_BOM_AND_ARCHITECTURE.md;
сопоставление с SDF — AIRFRAME_MODEL_MAPPING_RU.md.
Неактивные паспортные данные — simulation/models/glider_visual/physical_reference.json.
Desktop-артефакты и контрольные суммы — DELIVERY_AUDIT_2026-11-12.md.
