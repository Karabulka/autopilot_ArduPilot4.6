# Спринт 3 — PX4 Offboard и замкнутые испытания Gazebo Harmonic

Дата: 21 октября 2026, Europe/Moscow. PX4 checkout: `4a5eabb61e` с рабочими изменениями
Спринтов 1–3. Все полётные результаты относятся к SITL, без ветра и с плоской поверхностью.

## Реализованный контур

```text
Gazebo camera + CameraInfo ── ros_gz_bridge ── optical_flow_node ── visual velocity ─┐
Gazebo physical magnetometer ─ ros_gz_bridge ──────────────────── navigation_node ├─ LOS
PX4 IMU/mag/baro EKF attitude + angular velocity + barometric altitude ─ interface ┘
LOS target ENU/FLU ── px4_interface_node ── uXRCE Agent ── PX4 attitude/rate loops
                                                   └─ control allocator ── Gazebo servos/motor

Gazebo physics pose/body velocity ── /evaluation/ground_truth ── flight_evaluator.py
Gazebo motor command ────────────── /evaluation/motor_speed_cmd ────────────────┘
```

Навигационные узлы не подписываются на GPS, VehicleLocalPosition, VehicleOdometry PX4,
позу или скорость симулятора. Ground truth подписан только независимым оценщиком.
Оценщик не публикует навигационные данные или команды управления. Для проверки полярности
отдельный процесс `polarity_stimulus.py` выдаёт заранее заданные ступени, без подписки на truth.

Испытательный `FlightFixtureSystem` удерживает динамическую модель через fixed joint до готовности
Offboard и арминга. Сервис `/evaluation/launch` однократно освобождает joint и задаёт начальную
скорость 16 м/с всем звеньям. После этого никаких принудительных перемещений, коррекций скорости
или дополнительных сил от стенда нет: полёт определяется аэродинамикой, гравитацией, рулями и
двигателем. Это испытание с воздушным стартом, а не проверка разбега или взлёта с полосы.

## Airframe и GPS-denied настройки

`ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_rc_cessna_visual` наследует 4003 и
зарегистрирован в airframes/CMakeLists.txt. При каждом старте жёстко задаются:

```text
SYS_HAS_GPS=0, SIM_GPS_USED=0, EKF2_GPS_CTRL=0
EKF2_EV_CTRL=0, EKF2_OF_CTRL=0
UXRCE_DDS_SYNCT=0, UXRCE_DDS_PTCFG=2
EKF2_MAG_DECL=11.459156 deg, EKF2_DECL_TYPE=0
SENS_EN_ARSPDSIM=0, SYS_HAS_NUM_ASPD=0, FW_USE_AIRSPD=0
```

Три GPS-параметра дополнительно считываются из реально запущенного PX4 перед освобождением
модели; ненулевое значение останавливает тест. В модели отсутствует NavSat. GPS DDS writers
остаются в общей таблице PX4 для других airframe, но их наличие не означает поступление GPS
или его использование этим контуром.

Отключён унаследованный симулятор airspeed, использующий EKF groundspeed: без GPS он не является
независимым датчиком скорости. Тяга и скорость планирования регулируются по оптической скорости
в условиях нулевого ветра. Контуры PX4 без pitot используют параметрическую trim airspeed для
масштабирования регуляторов. Арминг производится обычным VehicleCommand без force-arm.
`COM_RC_IN_MODE=4` соответствует отсутствующему RC; при потере Offboard задан Stabilized
(`COM_OBL_RC_ACT=2`), без GPS-зависимого RTL/позиционного удержания.

## DDS и согласование сообщений

Сообщения `px4_msgs` генерируются из `msg/` и `msg/versioned/` текущего PX4, а не из движущейся
ветки GitHub. Манифест исходных путей и SHA256 сохраняется в
`build_visual_nav/deps/px4_msgs/message_manifest.json`. Сборка этой зависимости выполняется CMake
в изолированном каталоге, затем colcon обнаруживает только `src/glider_visual_nav`.

| ROS topic | Схема из PX4 | MESSAGE_VERSION |
| --- | --- | --- |
| /fmu/in/vehicle_attitude_setpoint_v1 | VehicleAttitudeSetpoint | 1 |
| /fmu/in/offboard_control_mode | OffboardControlMode | 0 / без константы |
| /fmu/in/vehicle_command | VehicleCommand | 0 |
| /fmu/out/vehicle_command_ack | VehicleCommandAck | 0 |
| /fmu/out/vehicle_status_v1 | VehicleStatus | 1 |
| /fmu/out/vehicle_attitude | VehicleAttitude | 0 |
| /fmu/out/vehicle_angular_velocity | VehicleAngularVelocity | 0 |
| /fmu/out/vehicle_air_data | VehicleAirData | 0 / без константы |
| /fmu/out/actuator_motors | ActuatorMotors | 0 / без константы |
| /fmu/out/vehicle_attitude_setpoint_v1 | VehicleAttitudeSetpoint | 1 |

Добавлены publications angular velocity (100 Hz), air data (50 Hz), motor output (20 Hz) и
принятая PX4 attitude setpoint (25 Hz). Суффиксы версий добавляются существующим генератором
PX4; ручное `_v1` в YAML привело бы к удвоению суффикса.

Micro XRCE-DDS Agent v2.4.3, commit `73622810d984349b80bbac0ef55fc0b694d62222`, собран локально
с установленными ROS FastDDS/FastCDR. Стандартное создание participant через XML от PX4 заменяет
транспортные QoS Agent, поэтому одного default-профиля Спринта 2 недостаточно. Airframe выбирает
именованный `px4_participant` через `UXRCE_DDS_PTCFG=2`; профиль дублирует UDPv4 loopback и
RTPS maxMessageSize=1200. Он загружается до запуска Agent и узлов ROS.

При регрессии DDS обнаружено, что graph discovery иногда опережает фактическую готовность
приёмника. Тест теперь начинает четырёхсекундное измерение после ACK реально полученных
изображения и магнитного сообщения, с отдельным ограничением времени прогрева. Пороговые
требования >=70 полных 640×480 изображений и >=200 магнитных сообщений не снижались.

## Математика сопряжения и динамическая компенсация

Используются матрицы из AGENTS.md:

```text
T = R_NED,ENU = [[0,1,0],[1,0,0],[0,0,-1]]
S = R_FLU,FRD = diag(1,-1,-1)
R_NED,FRD = T * R_WB * S
```

Обе матрицы имеют det=+1. Quaternion сериализуется как PX4 `[w,x,y,z]`. Для обычных углов
это соответствует roll_NED=roll_FLU, pitch_NED=-pitch_FLU, yaw_NED=pi/2-yaw_ENU.
Подъём носа задаётся отрицательным FLU pitch и положительным FRD/NED pitch.

VehicleAttitude переводится обратно по `R_WB=T^T*R_NED,FRD*S`. Gyro FRD переводится в FLU
матрицей S. Ориентация интерполируется к acquisition stamp angular velocity; публикуется
`sensor_msgs/Imu` в base_link. Газебовский quaternion ориентации IMU как truth не используется.

Из VehicleAirData берётся измеренная барометрическая высота. Нулевой barometric ground reference
задан для этого плоского мира с нулевой отметкой и стандартным давлением. Для реальной площадки
этот reference требует отдельной калибровки. Высота оптического центра:

```text
h_C(t) = h_B,baro(t) + [R_WB(t) * r_BC]_z
```

Измеренная h_C интерполируется к середине пары изображений и заменяет постоянный масштаб в
математическом ядре. Для луча q=[x,y,1] и нормали земли n_C глубина Z=h_C/(n_C^T q).
В imu_aided решается `flow_i - B_i*omega_C = A_i*v_C` методом LS/SVD после LK FB/RANSAC;
`omega_C=R_BC^T*omega_B`, `v_B=R_BC*v_C-omega_B×r_BC`. Крен/тангаж также компенсируют магнитометр.
Положение XY по-прежнему получается только интегрированием визуальной скорости и магнитного курса.

## Автомат Offboard и режимы полёта

`WAIT_SENSORS -> PRESTREAM (>=1.2 s) -> REQUEST_OFFBOARD -> REQUEST_ARM -> ACTIVE`.
Переходы подтверждаются VehicleStatus; VehicleCommandAck записывается в лог. Heartbeat
OffboardControlMode и VehicleAttitudeSetpoint публикуются wall timer 50 Hz. В прогонах полученная
частота по имитационному времени выше 20 Hz; она отличается от wall rate при RTF<1.

При отсутствии свежей визуальной одометрии более 0.7 s в полёте включается защёлкнутый
`DEGRADED_WINGS_LEVEL`: горизонтальные крылья, ограниченный trim pitch, тяга 0.35 в cruise и
строго 0 в glide. Старая LOS-команда не удерживается. Восстановление маршрута требует перезапуска
контроллера/явного вмешательства; само возвращение данных не снимает защёлку. При полном исчезновении
IMU/baro/status узел прекращает heartbeat, передавая управление встроенному Offboard-loss watchdog PX4.

`cruise_const_alt`: LOS по квадратному маршруту (0,0)->(100,0)->(100,100)->(0,100),
lookahead 40 m, максимальный крен 0.45 rad. PID высоты использует барометрическую высоту и
сглаженную её производную. Задание 40 m относится к оптическому центру; высота base_link
корректируется на повёрнутый рычаг камеры. Номинальная тяга 0.55 дополняется PI скорости
(12 m/s, Kp=0.08, Ki=0.02), с ограничением [0,1] и anti-windup.

`unpowered_glide`: тяга обнуляется последним присваиванием, включая bootstrap и fallback.
PI скорости планирования управляет тангажом (8.5 m/s, Kp=0.08, Ki=0.01), без команды набора
высоты. Высота для оптического масштаба измеряется непрерывно. В PX4 ActuatorMotors значение NaN
означает остановленный двигатель; отдельно проверяется численная команда Gazebo motor speed = 0.

## Методика и результаты

Сопоставление выполняется по acquisition stamp с интерполяцией truth. Первую секунду после
воздушного старта исключаем из RMSE навигации; для установившейся glide speed исключаем первые
5 s. Координатные начала совмещаются только в оценщике. Процент дрейфа — конечная ошибка XY,
делённая на фактически пройденный горизонтальный путь. CTE считается к активному прямому участку;
отдельно выделяются первые 4 s после смены цели waypoint. Banked-метрики используют |roll|>0.12 rad.

| Метрика | Cruise: cruise_final.json | Glide: glide_02.json |
| --- | ---: | ---: |
| Длительность после освобождения | 45 s | 30 s |
| Пройденный путь | 529.93 m | 243.68 m |
| RMSE Vx / Vy | 0.071 / 0.121 m/s | 0.156 / 0.168 m/s |
| RMSE курса | 1.07 deg | 1.59 deg |
| RMSE курса на виражах | 1.16 deg | 1.76 deg |
| RMSE Vx / Vy на виражах | 0.057 / 0.128 m/s | 0.093 / 0.177 m/s |
| Конечный дрейф XY | 6.50 m / 1.23% | 5.04 m / 2.07% |
| CTE RMSE, все участки | 11.94 m | 6.86 m |
| CTE RMSE, вне окна смены ППМ | 11.83 m | 6.01 m |
| CTE RMSE, окно смены ППМ | 12.11 m | 8.71 m |
| Максимальная абсолютная CTE | 22.84 m | 19.24 m |
| RMSE base_link относительно номинальных 40.25 m | 0.38 m | неприменимо |
| Высота base_link min / max | 39.24 / 40.75 m | 47.74 / 102.40 m |
| RMSE glide speed после первых 5 s | неприменимо | 0.33 m/s |
| Максимальная уставка тяги | 0.641 | 0.000 |
| Максимальная команда двигателя Gazebo | 641 rad/s | 0 rad/s |
| Полученная частота attitude setpoint, sim time | 63.61 Hz | 60.64 Hz |

Cruise прошёл все четыре участка и начал следующий круг; Glide переключил цели на три участка.
Оба прошли в Offboard/armed без визуального fallback. В cruise максимальный промежуток между
сопоставленными навигационными сообщениями составил 0.396 s; счисление удерживает координату на
неизвестном интервале и увеличивает covariance. В glide максимальный промежуток 0.034 s.

`polarity_final.json`: отдельный процесс подал ступень +0.15 rad крена и затем ступень
подъёма носа с 0.04 до 0.12 rad. Изменение физического крена составило +0.1534 rad (+8.79 deg),
подъёма носа +0.0890 rad (+5.10 deg). Оба знака верны. Этот тест не является проверкой маршрута
или высоты; route/CTE для него не интерпретируются.

`fallback_final.json`: на 8-й секунде полёта остановлен только созданный данным launch процесс
optical_flow_node. Через 0.612 s после отказа зафиксирован DEGRADED_WINGS_LEVEL. К концу 13-секундного
испытания физический крен -1.46 deg, PX4 nav_state=14 (Offboard), поток уставок продолжался
с полученной частотой 59.89 Hz по sim time. Высота уменьшилась до 36.73 m: fallback стабилизирует
крылья и ограничивает тягу, но не обеспечивает удержание высоты/посадку без визуальной навигации.
Отказ и отсутствие возобновления старой LOS-команды подтверждены телеметрией.

Последняя сборка PX4 и изолированная сборка пакета успешны. `colcon test-result`:
**25 агрегированных записей, 0 ошибок, 0 отказов**. Это 20 математических GTests, один тест ROS-узлов
и обёртки CTest/ament, включая три передачи DDS между разными процессами. Новые математические
проверки охватывают точный T*R*S и изменение масштаба по измеренной высоте.

![Маршруты и высота](/home/nikol/autopilot/log_visual_nav/sprint3_trajectories.png)
![Ошибки скорости, курса и координат](/home/nikol/autopilot/log_visual_nav/sprint3_errors.png)
![Скорость и команды тяги](/home/nikol/autopilot/log_visual_nav/sprint3_height_speed_actuation.png)

## Настройка и неуспешные прогоны

- Ранние старты выявили неверный rootfs аргумент PX4. Исправлено на build/px4_sitl_default/etc;
  launch готовит отдельный workdir и корректный etc symlink.
- `cruise_01`: постоянная тяга 0.55 и начальные настройки PID не удерживали высоту: снижение
  примерно до 28 m за 55 s, несмотря на прохождение маршрута. Это не квалификация cruise.
- `cruise_02`: простое увеличение тяги до 0.9 привело к разгону, выходу за текстурированную
  площадку и штатному визуальному fallback. Прогон не принят. Нужен замкнутый PI скорости,
  а не постоянная завышенная тяга.
- Первые glide настройки требовали 16 m/s и насыщали pitch. Для данной модели выбран и проверен
  режим 8.5 m/s с PI. Старый файл glide_final.json относится к промежуточной настройке;
  канонический результат итоговой настройки — glide_02.json.
- Промежуточные DDS тесты под нагрузкой выявили проблему стадии готовности; исправлен handshake,
  без снижения порогов приёма. Последний colcon test-result: 25 записей, 0 ошибок, 0 отказов.

## Воспроизведение

```bash
cd /home/nikol/autopilot
bash src/glider_visual_nav/tools/prepare_px4_dependencies.sh
source /opt/ros/jazzy/setup.bash
source install_visual_nav/px4_msgs/local_setup.bash
python3 src/glider_visual_nav/tools/generate_sim_assets.py
ninja -C build/px4_sitl_default -j4 px4
colcon --log-base /home/nikol/autopilot/log_visual_nav build \
  --base-paths /home/nikol/autopilot/src/glider_visual_nav \
  --build-base /home/nikol/autopilot/build_visual_nav \
  --install-base /home/nikol/autopilot/install_visual_nav \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install_visual_nav/setup.bash
python3 src/glider_visual_nav/tools/flight_evaluator.py --duration 45 \
  --output /home/nikol/autopilot/log_visual_nav/cruise_repeat.json
python3 src/glider_visual_nav/tools/flight_evaluator.py --regime unpowered_glide \
  --initial-height 100 --duration 30 --output /home/nikol/autopilot/log_visual_nav/glide_repeat.json
python3 src/glider_visual_nav/tools/flight_evaluator.py --polarity --duration 10 \
  --output /home/nikol/autopilot/log_visual_nav/polarity_repeat.json
python3 src/glider_visual_nav/tools/flight_evaluator.py --duration 13 --drop-visual-at 8 \
  --output /home/nikol/autopilot/log_visual_nav/fallback_repeat.json
colcon --log-base /home/nikol/autopilot/log_visual_nav test \
  --base-paths /home/nikol/autopilot/src/glider_visual_nav \
  --build-base /home/nikol/autopilot/build_visual_nav \
  --install-base /home/nikol/autopilot/install_visual_nav --packages-select glider_visual_nav
colcon test-result --test-result-base /home/nikol/autopilot/build_visual_nav --verbose
python3 src/glider_visual_nav/tools/export_flight_results.py
```

Прогоны выполняются последовательно: зарезервированы ROS_DOMAIN_ID=66, порт Agent 8888 и
PX4 instance 0; GZ_PARTITION уникален на прогон. Нельзя одновременно запускать другой PX4/Agent
на этих ресурсах. Оценщик останавливает только созданные им процессы и сохраняет JSON, JSONL,
process.log и PX4 ULog в log_visual_nav. Порог прекращения испытания по truth height 3 m относится
только к стенду и не является входом бортового управления.

Для ручного старта: `ros2 launch glider_visual_nav flight.launch.py flight_regime:=cruise_const_alt`.
После ACTIVE модель остаётся в удерживающем стенде до вызова `/evaluation/launch`; автоматическое
освобождение выполняет evaluator. Смена flight_regime требует совместного перезапуска узлов.

## Изменённые файлы

- AGENTS.md: разрешённая область Спринта 3.
- ROMFS/.../airframes/4022_gz_rc_cessna_visual и airframes/CMakeLists.txt.
- src/modules/uxrce_dds_client/dds_topics.yaml.
- src/glider_visual_nav/src/px4_interface_node.cpp и flight_fixture_system.cpp.
- visual_odometry.hpp/.cpp: measured optical height и точное PX4 преобразование.
- optical_flow_node.cpp, navigation_node.cpp, los_controller_node.cpp: динамические измерения,
  glide PI, altitude PID и корректировка высоты оптического центра.
- config/flight.yaml, flight_bridge.yaml, fastdds_wsl.xml, navigation.yaml; launch/flight.launch.py.
- tools/prepare_px4_messages.py, prepare_px4_dependencies.sh, flight_evaluator.py,
  polarity_stimulus.py, generate_sim_assets.py, inspect_flight.py, export_flight_results.py.
- test/test_visual_odometry.cpp, test/dds_peer.cpp, test/test_dds_processes.py.
- CMakeLists.txt, package.xml, README.md и этот отчёт.
- Tools/simulation/gz/worlds/visual_nav_flight.sdf внутри уже изменённого сабмодуля.

Сабмодуль и основной репозиторий пока не коммитились. Предшествующие изменения пользователя
в board, .vscode, дополнительных модулях и изображениях сохранены.

## Границы результата

Это замкнутый GPS-denied SITL, включая реальные PX4 attitude/rate loops и actuator allocation.
Не проверялись аппаратная платформа, ветер, рельеф, GPS-denied посадка, разбег, бесконечное
удержание высоты без тяги и потеря всех измерений одновременно. Нативный PX4 Offboard-loss
watchdog настроен, но отдельный отказ всего интерфейса не входит в эти лётные прогоны.
Барометрический ground reference и pinhole calibration считаются известными. Визуальная оценка
остаётся локальным счислением: дрейф не устраняется абсолютным внешним позиционированием.
CTE достигает десятков метров на 100-метровом маршруте; это проверка функционирования контура,
а не заявка на прецизионное прохождение ППМ. RMSE и covariance не являются гарантией безопасности
реального воздушного судна.

Основные первичные источники: локальные схемы msg/versioned и реализация PX4 fw_att_control,
fw_rate_control, uxrce_dds_client; описание Offboard — https://docs.px4.io/main/en/flight_modes/offboard;
версионирование ROS/PX4 — https://docs.px4.io/main/en/ros2/user_guide.
