const apiBase = (() => {
  const host = window.location.hostname || "localhost";
  const protocol = window.location.protocol === "file:" ? "http:" : window.location.protocol;
  const port = window.location.port || "8080";
  return `${protocol}//${host}:${port}/api`;
})();

const state = {
  status: {},
  resources: {},
  system: { steps: [] },
  maps: { maps: [] },
  waypoints: { waypoints: [] },
  inspection: { running: false, status: "idle" },
  robotProfile: { active: "", profiles: [] },
  nav2Presets: { active: "legged", presets: [] },
};
let inspectionSequence = (() => {
  try {
    const saved = JSON.parse(localStorage.getItem("inspectionSequence") || "[]");
    return Array.isArray(saved) ? saved : [];
  }
  catch (_) { return []; }
})();
let guideTexts = (() => {
  try {
    const saved = JSON.parse(localStorage.getItem("guideTexts") || "[]");
    return Array.isArray(saved) ? saved : [];
  } catch (_) { return []; }
})();
let selectedGuideWaypointId = null;
let selectedGuideTextId = null;
let editingGuideTextId = null;
let pointcloudWidgets = (() => {
  try {
    const saved = JSON.parse(localStorage.getItem("pointcloudWidgets") || "[]");
    if (Array.isArray(saved) && saved.length) return saved.map((widget) => ({
      ...widget,
      topic: widget.topic === "/livox/lidar" ? "/cloud_registered" : (widget.topic === "/laser_scan" ? "/front_laser_scan" : (widget.topic === "/scan" ? "/rear_laser_scan" : widget.topic)),
      collapsed: true,
    }));
  } catch (_) {}
  return [{ id: `cloud-${Date.now()}`, type: "pointcloud", name: "FAST-LIO 点云", topic: "/cloud_registered", color: "#ff0000", size: 3, visible: true, collapsed: true }];
})();
let pointcloudWidgetRenderKey = "";

const mapView = {
  tool: "pan", scale: 1, offsetX: 0, offsetY: 0, fittedMap: null,
  dragging: false, start: null, last: null, preview: null, goal: null,
  image: null, imageUrl: null,
};

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
let stopRemoteControl = () => {};
let mapUploadTargetName = null;
const escapeHtml = (value) => String(value ?? "").replace(/[&<>"']/g, (character) => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
})[character]);

function savePointcloudWidgets() {
  localStorage.setItem("pointcloudWidgets", JSON.stringify(pointcloudWidgets));
}

function pointcloudTopics() {
  return state.status.pointcloud_topics?.topics || [
    "/lidar_points", "/lidar_points_2", "/a2/navigation/lidar_points",
    "/cloud_registered", "/cloud_registered_body", "/Laser_map",
    "/ground_segmentation/ground_points", "/ground_segmentation/obstacle_points",
    "/cloud_effected", "/cloud_pcd", "/submap",
    "/combined_scan", "/front_laser_scan", "/rear_laser_scan",
  ];
}

async function syncPointcloudSubscriptions() {
  const topics = [...new Set(pointcloudWidgets.filter((widget) => widget.visible && (widget.type || "pointcloud") === "pointcloud").map((widget) => widget.topic))];
  await api("/status/pointcloud-topics", { method: "POST", body: JSON.stringify({ topics }) });
}

function renderPointcloudWidgets(force = false) {
  const container = $("#pointcloudWidgets");
  if (!container) return;
  const topics = pointcloudTopics();
  const key = JSON.stringify([pointcloudWidgets, topics]);
  if (!force && key === pointcloudWidgetRenderKey) return;
  pointcloudWidgetRenderKey = key;
  container.innerHTML = pointcloudWidgets.map((widget, index) => {
    const type = widget.type || "pointcloud";
    const pathTopics = ["/plan", "/plan_smoothed", "/transformed_global_plan", "/path"];
    const selectableTopics = type === "path" ? pathTopics : topics;
    const labels = { pointcloud: "点云", path: "路径", trajectory: "运动路径", waypoints: "点位", tf: "TF 坐标系" };
    const typeLabel = labels[type] || "图层";
    const topicSetting = ["pointcloud", "path"].includes(type)
      ? `<label>${typeLabel}话题<select data-cloud-topic="${index}">${selectableTopics.map((topic) => `<option value="${escapeHtml(topic)}" ${topic === widget.topic ? "selected" : ""}>${escapeHtml(topic)}</option>`).join("")}</select></label>`
      : "";
    return `
    <section class="pointcloud-widget ${widget.collapsed ? "collapsed" : ""}" data-cloud-widget="${escapeHtml(widget.id)}">
      <div class="pointcloud-widget-head">
        <input class="component-visible" type="checkbox" data-cloud-visible="${index}" ${widget.visible ? "checked" : ""} title="显示或隐藏">
        <span class="component-title"><strong>${escapeHtml(widget.name || `${typeLabel} ${index + 1}`)}</strong><small class="component-frequency" data-component-frequency="${index}">--</small></span>
        <button type="button" class="cloud-widget-toggle" data-cloud-toggle="${index}" title="展开或收起">${widget.collapsed ? "▸" : "▾"}</button>
        <button type="button" class="cloud-widget-remove" data-cloud-remove="${index}" title="删除组件">×</button>
      </div>
      <div class="pointcloud-widget-body" ${widget.collapsed ? "hidden" : ""}>
        <label>组件名称<input type="text" data-cloud-name="${index}" maxlength="40" value="${escapeHtml(widget.name || `${typeLabel} ${index + 1}`)}"></label>
        ${topicSetting}
        ${type === "tf" ? `<div class="cloud-appearance">
          <label>统一大小<input type="number" data-tf-length="${index}" min="0.05" max="5" step="0.05" value="${Number(widget.length) || 0.35}"></label>
          <label>统一粗细<input type="number" data-cloud-size="${index}" min="1" max="12" step="1" value="${Number(widget.size) || 3}"></label>
        </div><div class="tf-relations" data-tf-relations="${index}"></div>` : `<div class="cloud-appearance">
          <label>颜色<input type="color" data-cloud-color="${index}" value="${escapeHtml(widget.color)}"></label>
          <label>${["path", "trajectory"].includes(type) ? "线宽" : "大小"}<input type="number" data-cloud-size="${index}" min="1" max="12" step="1" value="${widget.size}"></label>
        </div>`}
      </div>
    </section>
  `; }).join("");
  renderComponentFrequencies();
}

function formatFrequency(value, fallback = "0 Hz") {
  const frequency = Number(value);
  return Number.isFinite(frequency) && frequency > 0 ? `${frequency.toFixed(frequency >= 10 ? 0 : 1)} Hz` : fallback;
}

function preferredFrequency(source) {
  const refreshFrequency = Number(source?.refresh_frequency_hz);
  return Number.isFinite(refreshFrequency) && refreshFrequency > 0
    ? refreshFrequency
    : source?.frequency_hz;
}

function renderComponentFrequencies() {
  const scanNames = { "/combined_scan": "combined", "/front_laser_scan": "front", "/rear_laser_scan": "rear" };
  pointcloudWidgets.forEach((widget, index) => {
    let label = "0 Hz";
    if ((widget.type || "pointcloud") === "pointcloud") {
      const scanName = scanNames[widget.topic];
      const source = scanName ? state.status.scans?.[scanName] : (state.status.pointclouds?.[widget.topic] || state.status.pointcloud_health?.[widget.topic]);
      label = formatFrequency(preferredFrequency(source));
    } else if (widget.type === "path") {
      label = formatFrequency(state.status.paths?.[widget.topic]?.frequency_hz);
    } else if (widget.type === "trajectory") {
      label = formatFrequency(state.status.mapping?.trajectory_frequency_hz);
    } else if (widget.type === "waypoints") {
      label = "按需";
    } else if (widget.type === "tf") {
      const frames = state.status.tf_tree?.frames || [];
      const availableNames = frames.map((frame) => frame.name);
      const selectedNames = Array.isArray(widget.frames)
        ? widget.frames.filter((name) => availableNames.includes(name))
        : availableNames;
      const selectedSet = new Set(selectedNames);
      const dynamicRates = frames
        .filter((frame) => selectedSet.has(frame.name) && frame.parent && !frame.static)
        .map((frame) => Number(frame.rate_hz))
        .filter((rate) => Number.isFinite(rate) && rate > 0);
      const selectedDynamicCount = frames.filter(
        (frame) => selectedSet.has(frame.name) && frame.parent && !frame.static,
      ).length;
      label = dynamicRates.length
        ? formatFrequency(Math.min(...dynamicRates))
        : (selectedNames.length && selectedDynamicCount === 0 ? "静态" : "0 Hz");
      const relations = document.querySelector(`[data-tf-relations="${index}"]`);
      if (relations) {
        const optionsKey = JSON.stringify(availableNames);
        if (relations.dataset.optionsKey !== optionsKey) {
          relations.dataset.optionsKey = optionsKey;
          relations.innerHTML = frames.length
            ? frames.map((frame) => `<label class="tf-frame-option"><input type="checkbox" data-tf-frame="${escapeHtml(frame.name)}" data-tf-widget="${index}"><span>${escapeHtml(frame.name)}</span></label>`).join("")
            : "<small>等待 TF 数据</small>";
        }
        relations.querySelectorAll("[data-tf-frame]").forEach((checkbox) => {
          checkbox.checked = selectedNames.includes(checkbox.dataset.tfFrame);
        });
      }
    }
    const element = document.querySelector(`[data-component-frequency="${index}"]`);
    if (element) element.textContent = label;
  });
  const builtin = {
    mapFrameFrequency: "静态",
    robotFrameFrequency: formatFrequency(state.status.tf?.frequency_hz),
    localCostmapFrequency: formatFrequency(state.status.costmaps?.local?.frequency_hz),
    globalCostmapFrequency: formatFrequency(state.status.costmaps?.global?.frequency_hz),
  };
  Object.entries(builtin).forEach(([id, label]) => { const element = document.getElementById(id); if (element) element.textContent = label; });
}

function renderCanvasTelemetry(pose) {
  const poseValue = $("#canvasPoseValue");
  if (poseValue) poseValue.textContent = pose
    ? `X=${Number(pose.x || 0).toFixed(2)}  Y=${Number(pose.y || 0).toFixed(2)}  W=${Number(pose.yaw || 0).toFixed(2)}`
    : "等待 TF";
  const motion = state.status.motion;
  const speedValue = $("#canvasSpeedValue");
  if (speedValue) speedValue.textContent = motion?.recent
    ? `X=${Number(motion.linear_x || 0).toFixed(2)}  Y=${Number(motion.linear_y || 0).toFixed(2)}  W=${Number(motion.angular_z || 0).toFixed(2)}`
    : "X=0.00  Y=0.00  W=0.00";
  const goal = state.status.goal_metrics;
  const distanceValue = $("#canvasGoalDistanceValue");
  if (distanceValue) {
    const target = Array.isArray(goal?.target)
      ? goal.target
      : (mapView.goal ? [mapView.goal.x, mapView.goal.y, mapView.goal.yaw || 0] : null);
    if (pose && target) {
      const dx = Math.abs(Number(target[0]) - Number(pose.x || 0));
      const dy = Math.abs(Number(target[1]) - Number(pose.y || 0));
      const yawDelta = Math.atan2(
        Math.sin(Number(target[2] || 0) - Number(pose.yaw || 0)),
        Math.cos(Number(target[2] || 0) - Number(pose.yaw || 0)),
      );
      const directDistance = Math.hypot(dx, dy);
      const pathDistance = Number.isFinite(Number(goal.distance_remaining))
        ? `${Number(goal.distance_remaining).toFixed(2)}m`
        : "--";
      distanceValue.textContent = `X=${dx.toFixed(2)}  Y=${dy.toFixed(2)}  W=${Math.abs(yawDelta).toFixed(2)}  直线=${directDistance.toFixed(2)}m  路径=${pathDistance}`;
    } else if (!target && !state.system?.navigation_running && !state.system?.startup_in_progress) {
      // Goal/result messages and TF updates arrive independently. While the
      // navigation session is alive, an empty sample must not erase the last
      // valid telemetry. Stopping the session is the explicit reset boundary.
      distanceValue.textContent = "X=--  Y=--  W=--  直线=--  路径=--";
    }
  }
}

const viewMeta = {
  overview: ["控制面板", "机器人状态、导航控制和当前地图"],
  task: ["任务", "导览路线、到点等待和外部语音播报"],
  maps: ["地图管理", "建图、存图、地图选择和文件完整性"],
  waypoints: ["存点管理", "保存、编辑、排序点位及往返测试"],
  texts: ["文本管理", "新增、编辑和删除导览播报文本"],
  advanced: ["参数页面", "选择机器人类型与已有导航参数"],
};

$("#apiBaseText").textContent = apiBase;

async function api(path, options = {}) {
  const response = await fetch(`${apiBase}${path}`, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  if (!response.ok) {
    if (response.status === 401) {
      window.location.replace("/login");
      throw new Error("登录已失效");
    }
    const text = await response.text();
    throw new Error(text || response.statusText);
  }
  return response.json();
}

function notify(text) {
  const box = $("#message");
  box.textContent = `${new Date().toLocaleTimeString()}  ${text}`;
}

function statusClass(ok, neutral = false) {
  if (neutral) return "idle";
  return ok ? "ok" : "bad";
}

function navigationDisplay(status, localized) {
  if (!stepRunning("nav2")) return { label: "导航未启动", ok: false, neutral: true };
  if (!localized) return { label: "等待定位", ok: false, neutral: true };
  if (!status.navigation?.status_recent) return { label: "导航空闲", ok: true, neutral: true };
  const statuses = status.navigation?.goal_statuses || [];
  const code = Number(status.navigation?.current_status ?? (statuses.length ? statuses[statuses.length - 1] : 0));
  const states = {
    1: { label: "目标已接受", ok: true, neutral: true },
    2: { label: "正在导航", ok: true, neutral: false },
    3: { label: "正在取消", ok: true, neutral: true },
    4: { label: "已到达", ok: true, neutral: false },
    5: { label: "已取消", ok: true, neutral: true },
    6: { label: "导航失败", ok: false, neutral: false },
  };
  return states[code] || { label: "导航空闲", ok: true, neutral: true };
}

function currentRobotPose() {
  // The robot-frame plugin must always represent the planar robot center,
  // never the FAST-LIO front-lidar origin used by /Odometry.
  const poseSourceActive = stepRunning("mapping") || Boolean(state.status.tf?.ok);
  if (!poseSourceActive) return null;
  const lightweightPose = state.status.robot_pose;
  if (lightweightPose?.ok
      && lightweightPose.child_frame_id === "base_link"
      && Date.now() - Number(lightweightPose._received_at || 0) <= 1000) {
    return lightweightPose;
  }
  const baseFrame = (state.status.tf_tree?.frames || []).find(
    (frame) => frame.name === "base_link"
      && Array.isArray(frame.translation)
      && Array.isArray(frame.rotation),
  );
  if (baseFrame) {
    const [qx, qy, qz, qw] = baseFrame.rotation.map(Number);
    return {
      x: Number(baseFrame.translation[0]) || 0,
      y: Number(baseFrame.translation[1]) || 0,
      z: 0,
      yaw: Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)),
      ok: true,
      frame_id: state.status.tf_tree?.fixed_frame || "map",
      child_frame_id: "base_link",
    };
  }
  return null;
}

function stepRunning(key) {
  return Boolean((state.system.steps || []).find((step) => step.key === key)?.running);
}

function updateControlInterlocks() {
  const status = state.status || {};
  const connected = Boolean(status.ros_data_connected ?? status.connected);
  const localized = connected && Boolean(status.tf?.ok) && Boolean(status.map?.received);
  const navReady = localized && stepRunning("nav2");
  const actionBusy = (status.navigation?.goal_statuses || []).some(
    (value) => [1, 2, 3].includes(Number(value)),
  );
  const navigationBusy = Boolean(
    status.waypoint_navigation?.busy || status.goal_metrics?.active || actionBusy,
  );
  const testButton = $("#startWaypointTestBtn");
  if (testButton) {
    testButton.disabled = !navReady || navigationBusy;
    testButton.title = navigationBusy ? "已有导航命令正在执行" : (navReady ? "开始点位往返测试" : (stepRunning("nav2") ? "请先完成定位" : "请先在主界面启动导航"));
  }
  const inspectionButton = $("#startInspectionBtn");
  if (inspectionButton) {
    inspectionButton.disabled = !navReady || navigationBusy || Boolean(state.inspection.running);
    inspectionButton.title = navigationBusy ? "已有导航命令正在执行" : (navReady ? "开始导览任务" : (stepRunning("nav2") ? "请先完成定位" : "请先启动导航"));
  }
  const goalButton = $("#goalTool");
  if (goalButton) {
    goalButton.disabled = !navReady || navigationBusy;
    goalButton.title = navigationBusy ? "已有导航命令正在执行" : (navReady ? "下发导航目标" : "请先完成定位");
  }
  const driverReady = stepRunning("driver");
  const velocityReady = Boolean(status.velocity?.ready);
  const remoteReady = driverReady && velocityReady && !navigationBusy;
  $$('[data-velocity]').forEach((button) => { button.disabled = !remoteReady; });
  if (!remoteReady) stopRemoteControl();
  $$('[data-remote-hint]').forEach((hint) => {
    hint.textContent = !driverReady
      ? "等待驱动启动"
      : (navigationBusy ? "自动导航执行中，遥控已锁定" : (velocityReady ? "底盘速度接口已连接，可以遥控" : "等待底盘速度接口连接"));
  });
}

function updateSystemControlButtons() {
  const starting = Boolean(state.system?.startup_in_progress);
  const stopping = Boolean(state.system?.shutdown_in_progress);
  const mapping = stepRunning("mapping");
  const mapSaving = stepRunning("map_save");
  // The LiDAR driver is shared by navigation and mapping. It blocks mapping
  // startup while left over from navigation, but must not disable Save/Stop
  // after the mapping session itself has started it.
  const navigationSessionRunning = [
    "localization", "nav2", "waypoint", "mission_service",
  ].some(stepRunning) || (!mapping && stepRunning("driver"));
  const coreRunning = navigationSessionRunning;
  const startButton = $("#startBtn");
  const stopButton = $("#stopBtn");
  const restartButton = $("#restartBtn");
  if (startButton) {
    startButton.disabled = coreRunning || starting || stopping || mapping;
    startButton.title = stopping ? "正在等待导航系统完全停止" : (startButton.disabled ? "导航已启动；请使用停止或重启" : "启动导航系统");
  }
  if (stopButton) stopButton.disabled = stopping || (!coreRunning && !starting);
  if (restartButton) restartButton.disabled = stopping || (!coreRunning && !starting);
  const mapSwitchLocked = coreRunning || starting || stopping || mapping;
  $$('button[data-map-selectable]').forEach((button) => {
    const selectable = button.dataset.mapSelectable === "true";
    button.disabled = mapSwitchLocked || !selectable;
    button.title = mapSwitchLocked ? "请先停止导航系统或建图" : "";
  });
  const mappingInterlocked = navigationSessionRunning || starting || stopping;
  const mappingStartButton = $("#mappingStartBtn");
  const mappingSaveButton = $("#mappingSaveBtn");
  const mappingStopButton = $("#mappingStopBtn");
  if (mappingStartButton) {
    mappingStartButton.disabled = mappingInterlocked || mapping || mapSaving;
    mappingStartButton.title = mappingInterlocked
      ? "导航系统尚未完全关闭"
      : (mapping || mapSaving ? "建图或存图正在运行" : "开始建图");
  }
  for (const button of [mappingSaveButton, mappingStopButton]) {
    if (!button) continue;
    button.disabled = mappingInterlocked || !mapping || mapSaving;
    button.title = mappingInterlocked
      ? "导航系统尚未完全关闭"
      : (!mapping ? "当前没有运行中的建图任务" : "");
  }
}

function renderStatus() {
  const status = state.status || {};
  const monitorConnected = Boolean(status.connected);
  const tfOk = monitorConnected && Boolean(status.tf?.ok);
  const navProcessRunning = stepRunning("nav2");
  // localization.launch.py owns map_server, so /map is available as soon as
  // localization starts; it must not be gated by the later Nav2 step.
  const mapOk = monitorConnected && Boolean(status.map?.received);
  const combinedOk = Boolean(status.sensors?.combined_scan || status.scans?.combined?.recent);
  const frontOk = Boolean(status.sensors?.front_scan || status.scans?.front?.recent);
  const rearOk = Boolean(status.sensors?.rear_scan || status.scans?.rear?.recent);
  // Sensor health must not depend on whether a visualization component is
  // enabled. Track the raw Livox stream specifically so a static PCD topic
  // cannot hide a stopped LiDAR driver.
  const cloudOk = [
    "/lidar_points", "/lidar_points_2", "/a2/navigation/lidar_points",
    "/cloud_registered", "/Laser_map",
  ].some((topic) => Boolean(status.pointcloud_health?.[topic]?.recent));
  const localCostmapReceived = navProcessRunning && Boolean(status.costmaps?.local?.recent);
  const globalCostmapReceived = navProcessRunning && Boolean(status.costmaps?.global?.recent);
  const localCostmapOk = localCostmapReceived && Boolean(status.costmaps?.local?.transformed_to_map);
  const globalCostmapOk = globalCostmapReceived && Boolean(status.costmaps?.global?.transformed_to_map);
  const connected = Boolean(status.ros_data_connected ?? status.connected);
  const localizationReady = connected && tfOk && mapOk;
  const navigationState = navigationDisplay(status, localizationReady);
  const localizationStable = Boolean(status.mapping?.localization_status?.stable);

  renderPointcloudWidgets();

  const cards = [
    ["连接", connected ? "在线" : "离线", connected],
    ["TF", tfOk ? "完整" : "不完整", tfOk],
    ["地图", mapOk ? "已加载" : "未加载", mapOk],
    ["融合雷达", combinedOk ? "有心跳" : "无心跳", combinedOk],
    ["点云", cloudOk ? "有数据" : "无数据", cloudOk],
    ["定位状态", localizationStable ? "已定位" : "未定位", localizationStable],
    ["局部代价地图", localCostmapOk ? "已对齐" : (localCostmapReceived ? "等待TF" : "无数据"), localCostmapOk],
    ["全局代价地图", globalCostmapOk ? "已对齐" : (globalCostmapReceived ? "等待TF" : "无数据"), globalCostmapOk],
    ["前雷达", frontOk ? "有心跳" : "无心跳", frontOk],
    ["后雷达", rearOk ? "有心跳" : "无心跳", rearOk],
    ["导航状态", navigationState.label, navigationState.ok, navigationState.neutral],
    ["机器人类型", state.robotProfile?.profiles?.find((item) => item.id === state.robotProfile?.active)?.name || "未知", true, true],
  ];

  $("#statusCards").innerHTML = cards.map(([label, value, ok, neutral]) => `
    <div class="metric">
      <strong>${label}</strong>
      <span class="${statusClass(ok, neutral)}">${value}</span>
    </div>
  `).join("");

  const pose = currentRobotPose();
  renderCanvasTelemetry(pose);
  if ($("#navState")) {
    $("#navState").textContent = status.navigation?.goal_statuses?.length
      ? `状态码 ${status.navigation.goal_statuses.join(",")}`
      : "无任务";
  }
  const recordButton = $("#saveCurrentWaypointBtn");
  if (recordButton) {
    recordButton.disabled = !localizationReady;
    recordButton.title = localizationReady ? "记录 map 坐标系中的当前位姿" : "请先完成地图加载和定位";
    $("#recordWaypointHint").textContent = localizationReady ? "定位完成，可以记录" : "需先完成定位";
  }
  updateControlInterlocks();
  renderPoseCanvas(pose);
  if ($("#currentSummary")) renderSummary();
}

function resourceLevel(percent, warning, danger) {
  if (!Number.isFinite(percent)) return "idle";
  if (percent >= danger) return "bad";
  if (percent >= warning) return "warn";
  return "ok";
}

function formatBytes(value, decimals = 1) {
  if (!Number.isFinite(value) || value < 0) return "--";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let amount = value;
  let index = 0;
  while (amount >= 1024 && index < units.length - 1) {
    amount /= 1024;
    index += 1;
  }
  return `${amount.toFixed(index === 0 ? 0 : decimals)} ${units[index]}`;
}

function formatUptime(seconds) {
  if (!Number.isFinite(seconds)) return "--";
  const days = Math.floor(seconds / 86400);
  const hours = Math.floor((seconds % 86400) / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  return days ? `${days}天 ${hours}小时` : `${hours}小时 ${minutes}分`;
}

function renderHostResources() {
  const container = $("#hostStatusCards");
  if (!container) return;
  const resources = state.resources || {};
  const cpu = resources.cpu || {};
  const memory = resources.memory || {};
  const disk = resources.disk || {};
  const load = resources.load || {};
  const swap = resources.swap || {};
  const network = resources.network || {};
  const battery = resources.battery || {};
  const loadRatio = Number.isFinite(load.one) && load.cores ? load.one / load.cores : null;
  const batteryLevel = !battery.recent || !Number.isFinite(battery.percent)
    ? "idle"
    : battery.percent <= 10 ? "bad" : battery.percent <= 20 ? "warn" : "ok";
  const batteryDetail = battery.recent
    ? `SOC${battery.power_state === "charging" ? " · 充电中" : ""}`
    : "等待数据";
  const cards = [
    ["CPU", Number.isFinite(cpu.percent) ? `${cpu.percent.toFixed(1)}%` : "采样中", `${cpu.cores || "--"} 核`, resourceLevel(cpu.percent, 70, 90)],
    ["内存", Number.isFinite(memory.percent) ? `${memory.percent.toFixed(1)}%` : "--", `${formatBytes(memory.used_bytes)} / ${formatBytes(memory.total_bytes)}`, resourceLevel(memory.percent, 75, 90)],
    ["磁盘", Number.isFinite(disk.percent) ? `${disk.percent.toFixed(1)}%` : "--", `${formatBytes(disk.used_bytes)} / ${formatBytes(disk.total_bytes)}`, resourceLevel(disk.percent, 80, 90)],
    ["系统负载", Number.isFinite(load.one) && Number.isFinite(load.five) ? `${load.one.toFixed(2)} / ${load.five.toFixed(2)}` : "-- / --", "1分 / 5分", resourceLevel(loadRatio, 0.7, 1)],
    ["Swap", swap.total_bytes ? `${swap.percent.toFixed(1)}%` : "未配置", swap.total_bytes ? `${formatBytes(swap.used_bytes)} / ${formatBytes(swap.total_bytes)}` : "无需告警", swap.total_bytes ? resourceLevel(swap.percent, 10, 40) : "idle"],
    ["网络", network.interface ? `↓ ${formatBytes(network.receive_bytes_per_second)}/s · ↑ ${formatBytes(network.transmit_bytes_per_second)}/s` : "无网卡", network.interface || "未检测到接口", network.online ? "ok" : "bad"],
    ["运行时间", formatUptime(resources.uptime_seconds), "", "ok"],
    ["电量", battery.recent && Number.isFinite(battery.percent) ? `${battery.percent.toFixed(1)}%` : "无数据", batteryDetail, batteryLevel],
  ];
  container.innerHTML = cards.map(([label, value, detail, level]) => `
    <div class="host-metric">
      <strong>${label}</strong>
      <small>${detail}</small>
      <span class="${level}">${value}</span>
    </div>
  `).join("");
}

async function refreshHostResources() {
  state.resources = await api("/status/resources");
  renderHostResources();
}

function renderSystemSteps() {
  const steps = (state.system.steps || []).filter(
    (step) => ![
      "status_monitor", "pose_monitor", "mapping", "map_save",
      "waypoint", "tcp_listener", "ultrasonic_stop",
    ].includes(step.key),
  );
  const tcpStep = (state.system.steps || []).find((step) => step.key === "tcp_listener") || {};
  const tcpRunning = Boolean(tcpStep.running);
  const tcpConfig = state.system.tcp_config || { server_host: "127.0.0.1", server_port: 8050 };
  $("#startupSteps").innerHTML = steps.map((step) => `
    <div class="step-tile">
      <div class="step-title-row">
        <strong>${step.label}</strong>
        <span class="${step.running ? "ok" : "idle"}">${step.running ? (step.managed ? "运行中" : "外部运行") : "未启动"}</span>
      </div>
      <div class="step-actions">
        <button data-step-start="${step.key}" ${step.running ? "disabled" : ""}>启动</button>
        <button data-step-stop="${step.key}" ${step.controllable ? "" : "disabled"} title="${step.running && !step.controllable ? "未找到可安全接管的启动进程" : ""}">停止</button>
        <button data-step-restart="${step.key}" ${step.controllable ? "" : "disabled"}>重启</button>
      </div>
    </div>
  `).join("") + `
    <div class="step-tile tcp-step-tile">
      <div class="tcp-step-top">
        <div class="step-title-row"><strong>语音TCP</strong><span class="${tcpRunning ? "ok" : "idle"}">${tcpRunning ? "运行中" : "未启动"}</span></div>
        <div class="tcp-config-fields">
          <label><span>IP</span><input id="tcpServerHost" inputmode="decimal" value="${escapeHtml(tcpConfig.server_host)}" aria-label="语音 TCP 服务器 IP"></label>
          <label><span>端口</span><input id="tcpServerPort" type="number" min="1" max="65535" value="${Number(tcpConfig.server_port) || 8050}" aria-label="语音 TCP 服务器端口"></label>
        </div>
      </div>
      <div class="step-actions">
        <button data-tcp-action="start" ${tcpRunning ? "disabled" : ""}>启动</button>
        <button data-tcp-action="stop" ${tcpRunning ? "" : "disabled"}>停止</button>
        <button id="saveTcpConfigBtn" type="button">保存</button>
      </div>
    </div>`;
  updateControlInterlocks();

  $$("button[data-step-start]").forEach((button) => {
    button.addEventListener("click", async () => {
      await api("/system/step/start", {
        method: "POST",
        body: JSON.stringify({ step: button.dataset.stepStart }),
      });
      notify("步骤启动命令已发送");
      await refreshSystem();
    });
  });

  $$("button[data-step-stop]").forEach((button) => {
    button.addEventListener("click", async () => {
      await api("/system/step/stop", {
        method: "POST",
        body: JSON.stringify({ step: button.dataset.stepStop }),
      });
      notify("步骤停止命令已发送");
      await refreshSystem();
    });
  });
  $$("button[data-step-restart]").forEach((button) => {
    button.addEventListener("click", async () => {
      await api("/system/step/restart", {
        method: "POST",
        body: JSON.stringify({ step: button.dataset.stepRestart }),
      });
      notify("步骤已重启");
      await refreshSystem();
    });
  });
  $$("button[data-tcp-action]").forEach((button) => {
    button.addEventListener("click", async () => {
      const action = button.dataset.tcpAction;
      state.system = await api(`/system/tcp-control/${action}`, { method: "POST" });
      notify(`语音TCP${{ start: "启动", stop: "停止", restart: "重启" }[action]}命令已发送`);
      renderSystemSteps();
    });
  });
  $("#saveTcpConfigBtn")?.addEventListener("click", async () => {
    const serverHost = $("#tcpServerHost").value.trim();
    const serverPort = Number($("#tcpServerPort").value);
    try {
      const result = await api("/system/tcp-config", {
        method: "POST",
        body: JSON.stringify({ server_host: serverHost, server_port: serverPort }),
      });
      state.system.tcp_config = result;
      notify(tcpRunning ? "语音TCP配置已保存，请停止后重新启动以生效" : "语音TCP配置已保存");
      renderSystemSteps();
    } catch (error) { notify(error.message); }
  });
}

function selectedMap() {
  return (state.maps.maps || []).find((map) => map.selected);
}

function ensureMapImage(map) {
  if (!map?.pgm?.exists) return;
  const url = apiBase + "/maps/" + encodeURIComponent(map.name) + "/thumbnail.bmp?v=" + (map.pgm.modified_ns || 0);
  if (mapView.imageUrl === url) return;
  mapView.imageUrl = url;
  mapView.image = new Image();
  mapView.image.onload = () => { fitMap(); renderPoseCanvas(currentRobotPose()); };
  mapView.image.src = url;
}

function fitMap() {
  if (mapView.scene) { mapView.scene.fit(); return; }
  const canvas = $("#poseCanvas");
  const metadata = selectedMap()?.metadata;
  if (!metadata) return;
  const padding = 28;
  mapView.scale = Math.min((canvas.width - padding * 2) / metadata.width, (canvas.height - padding * 2) / metadata.height);
  mapView.offsetX = (canvas.width - metadata.width * mapView.scale) / 2;
  mapView.offsetY = (canvas.height - metadata.height * mapView.scale) / 2;
  mapView.fittedMap = selectedMap()?.name;
}

function worldToCanvas(x, y) {
  const metadata = selectedMap()?.metadata;
  if (!metadata) return null;
  const [originX, originY, originYaw] = metadata.origin;
  const dx = x - originX, dy = y - originY;
  const gridX = (Math.cos(originYaw) * dx + Math.sin(originYaw) * dy) / metadata.resolution;
  const gridY = (-Math.sin(originYaw) * dx + Math.cos(originYaw) * dy) / metadata.resolution;
  return { x: mapView.offsetX + gridX * mapView.scale, y: mapView.offsetY + (metadata.height - gridY) * mapView.scale };
}

function canvasToWorld(x, y) {
  const metadata = selectedMap()?.metadata;
  if (!metadata) return null;
  const [originX, originY, originYaw] = metadata.origin;
  const gridX = ((x - mapView.offsetX) / mapView.scale) * metadata.resolution;
  const gridY = (metadata.height - (y - mapView.offsetY) / mapView.scale) * metadata.resolution;
  return { x: originX + Math.cos(originYaw) * gridX - Math.sin(originYaw) * gridY, y: originY + Math.sin(originYaw) * gridX + Math.cos(originYaw) * gridY };
}

function drawArrow(ctx, start, yaw, color, size = 24) {
  if (!start) return;
  ctx.save(); ctx.translate(start.x, start.y); ctx.rotate(-yaw);
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(-size * .45, 0); ctx.lineTo(size, 0); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(size, 0); ctx.lineTo(size * .55, -size * .28); ctx.lineTo(size * .55, size * .28); ctx.closePath(); ctx.fill();
  ctx.restore();
}

function recentScans() {
  const scans = state.status.scans || {};
  if (scans.combined?.recent) return [["combined", scans.combined]];
  return [["front", scans.front], ["rear", scans.rear]].filter(([, scan]) => scan?.recent);
}

function scanPointInRobot(_name, angle, range) {
  return { x: Math.cos(angle) * range, y: Math.sin(angle) * range };
}

function drawLocalRadar(ctx, scans) {
  const center = { x: ctx.canvas.width / 2, y: ctx.canvas.height / 2 };
  const pixelsPerMeter = 45;
  ctx.fillStyle = "#202830"; ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
  ctx.strokeStyle = "#46515c"; ctx.lineWidth = 1;
  for (let radius = pixelsPerMeter; radius < Math.min(ctx.canvas.width, ctx.canvas.height) / 2; radius += pixelsPerMeter) {
    ctx.beginPath(); ctx.arc(center.x, center.y, radius, 0, Math.PI * 2); ctx.stroke();
  }
  ctx.fillStyle = "#ee3e32";
  scans.forEach(([name, scan]) => scan.points.forEach(([angle, range]) => {
    const point = scanPointInRobot(name, angle, range);
    ctx.fillRect(center.x + point.x * pixelsPerMeter - 1.5, center.y - point.y * pixelsPerMeter - 1.5, 3, 3);
  }));
  ctx.fillStyle = "#1677c8"; ctx.beginPath(); ctx.arc(center.x, center.y, 9, 0, Math.PI * 2); ctx.fill();
  drawArrow(ctx, center, 0, "#0b9ddd", 22);
  ctx.fillStyle = "#d8e0e7"; ctx.font = "14px Arial";
  ctx.fillText("雷达局部视图（等待定位 TF）", 18, 26);
}

function renderPoseCanvas(pose) {
  const canvas = $("#poseCanvas");
  if (!mapView.scene) {
    try { mapView.scene = new NavigationScene3D(canvas); }
    catch (error) { notify(error.message); return; }
  }
  const scanNames = { "/combined_scan": "combined", "/front_laser_scan": "front", "/rear_laser_scan": "rear" };
  const clouds = pointcloudWidgets.filter((widget) => widget.visible && (widget.type || "pointcloud") === "pointcloud").map((widget) => {
    const scanName = scanNames[widget.topic];
    if (!scanName) return {
      data: state.status.pointclouds?.[widget.topic] || null,
      color: widget.color, size: widget.size,
    };
    const scan = state.status.scans?.[scanName];
    if (!scan?.recent) return { data: null, color: widget.color, size: widget.size };
    if (scan.transformed_to_fixed && scan.fixed_points?.length) {
      return {
        data: { recent: true, transformed_to_fixed: true, points: scan.fixed_points },
        color: widget.color, size: widget.size,
      };
    }
    // Sensor frames no longer share the robot-center origin. Never guess an
    // installation offset in the browser; wait for the backend TF transform.
    return { data: null, color: widget.color, size: widget.size };
  });
  const mappingActive = stepRunning("mapping");
  const navigationActive = !mappingActive
    && (stepRunning("nav2") || Boolean(state.system?.startup_in_progress));
  if (!navigationActive) {
    mapView.goal = null;
    mapView.preview = null;
  }
  pointcloudWidgets.filter((widget) => navigationActive && widget.visible && widget.type === "waypoints").forEach((widget) => {
    clouds.push({
      data: { recent: true, transformed_to_fixed: true, points: state.waypoints.waypoints.map((item) => [item.x, item.y, 0.12]) },
      color: widget.color, size: widget.size,
    });
  });
  const paths = pointcloudWidgets
    .filter((widget) => navigationActive && widget.visible && widget.type === "path")
    .map((widget) => ({
      data: state.status.paths?.[widget.topic] || null,
      color: widget.color,
      size: widget.size,
      kind: "path",
    }));
  pointcloudWidgets.filter((widget) => widget.visible && widget.type === "trajectory").forEach((widget) => {
    paths.push({
      data: { transformed_to_fixed: true, points: (state.status.mapping?.trajectory || []).map((item) => [item[0], item[1], 0]) },
      color: widget.color, size: widget.size, kind: "trajectory",
    });
  });
  const tfWidget = pointcloudWidgets.find((widget) => widget.visible && widget.type === "tf");
  mapView.scene.update({
    map: selectedMap(), pose, pointclouds: clouds, paths,
    waypoints: [],
    goal: navigationActive ? mapView.goal : null,
    preview: mapView.preview,
    showMapFrame: $("#mapFrameVisible")?.checked,
    showRobotFrame: $("#robotFrameVisible")?.checked,
    costmaps: navigationActive ? (state.status.costmaps || {}) : {},
    showLocalCostmap: $("#localCostmapVisible")?.checked,
    showGlobalCostmap: $("#globalCostmapVisible")?.checked,
    liveMap: state.status.mapping?.live_map,
    mapFrameLength: Number($("#mapFrameLength")?.value) || 0.5,
    mapFrameThickness: Number($("#mapFrameThickness")?.value) || 4,
    robotFrameLength: Number($("#robotFrameLength")?.value) || 0.5,
    robotFrameThickness: Number($("#robotFrameThickness")?.value) || 4,
    tfTree: tfWidget ? state.status.tf_tree : null,
    tfSelectedFrames: tfWidget && Array.isArray(tfWidget.frames) ? tfWidget.frames : null,
    tfFrameLength: Number(tfWidget?.length) || 0.35,
    tfFrameThickness: Number(tfWidget?.size) || 3,
  });
}

function renderSummary() {
  const currentMap = state.maps.maps.find((map) => map.selected);
  const waypointCount = state.waypoints.waypoints.length;
  $("#currentSummary").innerHTML = [
    ["当前地图", currentMap ? currentMap.name : "未选择"],
    ["点位数量", `${waypointCount}`],
    ["地图根目录", state.maps.map_directory || "-"],
    ["Z 轴状态", state.status.safety?.z_axis_warning || "-"],
  ].map(([label, value]) => `
    <div class="summary-row"><strong>${label}</strong><span>${value}</span></div>
  `).join("");
}

function renderMaps() {
  $("#mapDirectory").textContent = state.maps.map_directory || "";
  const mapSwitchLocked = ["driver", "localization", "nav2", "waypoint", "mapping", "map_save"].some(stepRunning)
    || Boolean(state.system?.startup_in_progress);
  $("#mapList").innerHTML = (state.maps.maps || []).map((map) => {
    const complete = map.yaml.exists && map.pgm.exists && map.pcd.exists;
    const waypointOptions = (map.waypoints || []).map((point) =>
      `<option value="${point.index}">${escapeHtml(point.note || "未命名点位")}</option>`
    ).join("");
    return `
      <div class="tile map-tile">
        <div class="map-thumb">
          ${map.pgm.exists ? `<img src="${apiBase}/maps/${encodeURIComponent(map.name)}/thumbnail.bmp" alt="${map.name}">` : "<span>无缩略图</span>"}
        </div>
        <div class="tile-main">
          <div class="map-title-line">
            <strong>${map.name}${map.selected ? "（当前）" : ""}</strong>
            <span class="meta">YAML ${map.yaml.exists ? "存在" : "缺失"}</span>
            <span class="meta">PGM ${map.pgm.exists ? "存在" : "缺失"}</span>
            <span class="meta">PCD ${map.pcd.exists ? "存在" : "缺失"}</span>
          </div>
          <div class="map-pose-fields">
            <label><span>初始 X</span><input type="number" step="0.01" data-map-pose-x="${map.name}" value="${Number(map.initial_pose?.x) || 0}"></label>
            <label><span>初始 Y</span><input type="number" step="0.01" data-map-pose-y="${map.name}" value="${Number(map.initial_pose?.y) || 0}"></label>
            <label><span>初始 Yaw</span><input type="number" step="0.01" data-map-pose-yaw="${map.name}" value="${Number(map.initial_pose?.yaw) || 0}"></label>
            <select data-map-pose-waypoint="${map.name}" ${waypointOptions ? "" : "disabled"}>
              ${waypointOptions || '<option value="">无已存点位</option>'}
            </select>
            <button type="button" data-map-pose-import="${map.name}" ${waypointOptions ? "" : "disabled"}>导入</button>
          </div>
        </div>
        <div class="step-actions map-actions">
          <button data-map-download="${map.name}">下载 PGM</button>
          <button data-map-upload="${map.name}">回传 PGM</button>
          <button data-map-pose-save="${map.name}">保存初始位姿</button>
          <button data-map="${map.name}" data-map-selectable="${!map.selected && complete}" ${map.selected || !complete || mapSwitchLocked ? "disabled" : ""}>选择</button>
          <button data-map-delete="${map.name}" class="danger" ${map.selected ? "disabled" : ""}>删除</button>
        </div>
      </div>
    `;
  }).join("");

  $$("button[data-map]").forEach((button) => {
    button.addEventListener("click", async () => {
      try {
        const result = await api("/maps/select", {
          method: "POST",
          body: JSON.stringify({ name: button.dataset.map }),
        });
        notify(result.requires_navigation_restart ? "地图已选择，请重启导航生效" : "地图已选择");
        await refreshAll();
      } catch (error) { notify(error.message); }
    });
  });
  $$("button[data-map-pose-save]").forEach((button) => {
    button.addEventListener("click", async () => {
      const name = button.dataset.mapPoseSave;
      const pose = {
        x: Number($(`[data-map-pose-x="${name}"]`).value),
        y: Number($(`[data-map-pose-y="${name}"]`).value),
        yaw: Number($(`[data-map-pose-yaw="${name}"]`).value),
      };
      await api(`/maps/${encodeURIComponent(name)}/initial-pose`, {
        method: "PUT", body: JSON.stringify(pose),
      });
      notify(`地图“${name}”的初始位姿已保存`);
      await refreshMaps();
    });
  });
  $$("button[data-map-pose-import]").forEach((button) => {
    button.addEventListener("click", () => {
      const name = button.dataset.mapPoseImport;
      const map = (state.maps.maps || []).find((item) => item.name === name);
      const index = Number($(`[data-map-pose-waypoint="${name}"]`).value);
      const point = (map?.waypoints || []).find((item) => Number(item.index) === index);
      if (!point) return;
      $(`[data-map-pose-x="${name}"]`).value = Number(point.x);
      $(`[data-map-pose-y="${name}"]`).value = Number(point.y);
      $(`[data-map-pose-yaw="${name}"]`).value = Number(point.yaw || 0);
      notify(`已导入点位“${point.note || "未命名点位"}”，点击保存后生效`);
    });
  });
  $$("button[data-map-delete]").forEach((button) => {
    button.addEventListener("click", async () => {
      const name = button.dataset.mapDelete;
      if (!window.confirm(`确定删除地图“${name}”吗？地图和对应点位会分别移入可恢复目录。`)) return;
      const result = await api(`/maps/${encodeURIComponent(name)}`, { method: "DELETE" });
      const pointHint = result.waypoint_recovery_path
        ? `；点位备份：${result.waypoint_recovery_path}`
        : "；该地图没有点位数据";
      notify(`地图已删除，可从 ${result.recovery_path} 恢复${pointHint}`);
      await refreshMaps();
    });
  });
  $$("button[data-map-download]").forEach((button) => {
    button.addEventListener("click", () => {
      window.location.href = `${apiBase}/maps/${encodeURIComponent(button.dataset.mapDownload)}/download`;
    });
  });
  $$("button[data-map-upload]").forEach((button) => {
    button.addEventListener("click", () => {
      mapUploadTargetName = button.dataset.mapUpload;
      $("#mapUploadFile").value = "";
      $("#mapUploadFile").click();
    });
  });
}

function renderWaypoints() {
  $("#waypointTable").innerHTML = state.waypoints.waypoints.map((point) => `
    <tr>
      <td>${point.index}</td>
      <td>${point.x}</td>
      <td>${point.y}</td>
      <td>${point.yaw}</td>
      <td>${escapeHtml(point.note || "-")}</td>
      <td><div class="waypoint-actions">
        <button data-waypoint-edit="${point.index}">编辑</button>
        <button data-waypoint-up="${point.index}" ${point.index === 1 ? "disabled" : ""}>上移</button>
        <button data-waypoint-down="${point.index}" ${point.index === state.waypoints.waypoints.length ? "disabled" : ""}>下移</button>
        <button data-waypoint-delete="${point.index}" class="danger">删除</button>
      </div></td>
    </tr>
  `).join("");

  $$("button[data-waypoint-delete]").forEach((button) => {
    button.addEventListener("click", async () => {
      await api(`/waypoints/${button.dataset.waypointDelete}`, { method: "DELETE" });
      notify("点位已删除");
      await refreshWaypointViews();
    });
  });

  $$("button[data-waypoint-edit]").forEach((button) => {
    button.addEventListener("click", async () => {
      const point = state.waypoints.waypoints.find(
        (item) => item.index === Number(button.dataset.waypointEdit),
      );
      const x = window.prompt("X 坐标", point.x); if (x === null) return;
      const y = window.prompt("Y 坐标", point.y); if (y === null) return;
      const yaw = window.prompt("Yaw（弧度）", point.yaw); if (yaw === null) return;
      const note = window.prompt("备注 / 语义点位", point.note || ""); if (note === null) return;
      await api(`/waypoints/${point.index}`, {
        method: "PUT",
        body: JSON.stringify({ x: Number(x), y: Number(y), yaw: Number(yaw), note }),
      });
      notify("点位已更新"); await refreshWaypointViews();
    });
  });

  const moveWaypoint = async (index, delta) => {
    const order = state.waypoints.waypoints.map((point) => point.index);
    const position = order.indexOf(index), target = position + delta;
    [order[position], order[target]] = [order[target], order[position]];
    await api("/waypoints/reorder", {
      method: "POST", body: JSON.stringify({ order }),
    });
    notify("点位顺序已更新"); await refreshWaypointViews();
  };
  $$("button[data-waypoint-up]").forEach((button) =>
    button.addEventListener("click", () => moveWaypoint(Number(button.dataset.waypointUp), -1)));
  $$("button[data-waypoint-down]").forEach((button) =>
    button.addEventListener("click", () => moveWaypoint(Number(button.dataset.waypointDown), 1)));

  renderInspectionBuilder();
}

function saveInspectionSequence() {
  localStorage.setItem("inspectionSequence", JSON.stringify(inspectionSequence));
}

function saveGuideTexts() {
  localStorage.setItem("guideTexts", JSON.stringify(guideTexts));
}

function renderInspectionBuilder() {
  if (!$("#inspectionWaypointSource")) return;
  $("#inspectionWaypointSource").innerHTML = state.waypoints.waypoints.map((point) => `
    <button class="inspection-item selectable ${selectedGuideWaypointId === point.id ? "selected" : ""}" data-guide-waypoint-select="${point.id}">
      <span class="inspection-item-main"><strong>#${point.index} ${escapeHtml(point.note || "未备注")}</strong><span>x=${point.x} y=${point.y} yaw=${point.yaw}</span></span>
    </button>
  `).join("");
  $("#guideTextList").innerHTML = guideTexts.map((textBlock) => `
    <button class="inspection-item selectable ${selectedGuideTextId === textBlock.id ? "selected" : ""}" data-guide-text-select="${textBlock.id}">
      <div class="inspection-item-main"><strong>${escapeHtml(textBlock.note || textBlock.title || "未备注文本")}</strong><span>${escapeHtml(textBlock.content)}</span></div>
    </button>
  `).join("");
  renderGuideTextManagement();
  $("#inspectionSequence").innerHTML = inspectionSequence.map((step, index) => {
    const point = step.type === "waypoint" ? state.waypoints.waypoints.find((item) => item.id === step.waypoint_id || (!step.waypoint_id && item.index === step.waypoint_index)) : null;
    const title = step.type === "wait" ? `等待 ${step.seconds} 秒` : step.type === "text" ? "播报文本" : `前往点位 #${point?.index || step.waypoint_index} ${escapeHtml(point?.note || "")}`;
    const detail = step.type === "wait" ? "计时结束后继续" : step.type === "text" ? escapeHtml(step.message) : (point ? `x=${point.x} y=${point.y}` : "点位已不存在");
    return `<div class="inspection-item sequence-${step.type}"><div class="inspection-item-main"><strong>${index + 1}. ${title}</strong><span>${detail}</span></div><div class="inspection-item-actions"><button data-inspection-up="${index}" ${index === 0 ? "disabled" : ""}>↑</button><button data-inspection-down="${index}" ${index === inspectionSequence.length - 1 ? "disabled" : ""}>↓</button><button data-inspection-delete="${index}" class="danger">删除</button></div></div>`;
  }).join("");
  $$('[data-guide-waypoint-select]').forEach((button) => button.addEventListener("click", () => { selectedGuideWaypointId = button.dataset.guideWaypointSelect; renderInspectionBuilder(); }));
  $$('[data-guide-text-select]').forEach((item) => item.addEventListener("click", () => { selectedGuideTextId = item.dataset.guideTextSelect; renderInspectionBuilder(); }));
  bindInspectionSequenceActions();
}

function renderGuideTextManagement() {
  if (!$("#guideTextManageList")) return;
  $("#guideTextManageList").innerHTML = guideTexts.map((textBlock, index) => `
    <div class="inspection-item">
      <div class="inspection-item-main"><strong>${index + 1}. ${escapeHtml(textBlock.note || textBlock.title || "未备注文本")}</strong><span>${escapeHtml(textBlock.content)}</span></div>
      <div class="inspection-item-actions"><button data-guide-text-edit="${textBlock.id}">编辑</button><button data-guide-text-delete="${textBlock.id}" class="danger">删除</button></div>
    </div>
  `).join("");
  $$('[data-guide-text-edit]').forEach((button) => button.addEventListener("click", () => {
    const block = guideTexts.find((item) => item.id === button.dataset.guideTextEdit);
    editingGuideTextId = block.id; $("#guideTextContent").value = block.content;
    $("#guideTextNote").value = block.note || "";
    $("#saveGuideTextBtn").textContent = "保存文本块"; $("#cancelGuideTextEditBtn").hidden = false;
  }));
  $$('[data-guide-text-delete]').forEach((button) => button.addEventListener("click", () => {
    guideTexts = guideTexts.filter((item) => item.id !== button.dataset.guideTextDelete);
    if (selectedGuideTextId === button.dataset.guideTextDelete) selectedGuideTextId = null;
    saveGuideTexts(); renderInspectionBuilder();
  }));
  if (!guideTexts.length) $("#guideTextManageList").innerHTML = '<div class="empty-state">暂无文本块，请在上方输入内容后新增。</div>';
}

function bindInspectionSequenceActions() {
  const move = (index, delta) => {
    const target = index + delta;
    [inspectionSequence[index], inspectionSequence[target]] = [inspectionSequence[target], inspectionSequence[index]];
    saveInspectionSequence(); renderInspectionBuilder();
  };
  $$('[data-inspection-up]').forEach((button) => button.addEventListener("click", () => move(Number(button.dataset.inspectionUp), -1)));
  $$('[data-inspection-down]').forEach((button) => button.addEventListener("click", () => move(Number(button.dataset.inspectionDown), 1)));
  $$('[data-inspection-delete]').forEach((button) => button.addEventListener("click", () => {
    inspectionSequence.splice(Number(button.dataset.inspectionDelete), 1);
    saveInspectionSequence(); renderInspectionBuilder();
  }));
}

function renderInspectionStatus() {
  const task = state.inspection || {};
  const labels = { idle: "导览未运行", running: "导览准备中", navigating: "前往导览点位", speaking: "发送播报文本", waiting: "等待中", completed: "导览已完成", cancelled: "导览已停止", failed: "导览失败" };
  $("#inspectionStatus").textContent = `${labels[task.status] || task.status}${task.total_steps ? ` · ${task.completed_steps}/${task.total_steps}` : ""}${task.error ? ` · ${task.error}` : ""}`;
  $("#cancelInspectionBtn").disabled = !task.running;
  updateControlInterlocks();
}

let statusRefreshRunning = false;
async function refreshStatus() {
  if (statusRefreshRunning) return;
  statusRefreshRunning = true;
  try {
    const fresh = await api("/status");
    const currentScans = state.status.scans || {};
    const scanHealth = fresh.scans || {};
    state.status = {
      ...state.status, ...fresh,
      tf: state.status.tf?._visual ? state.status.tf : fresh.tf,
      scans: {
        ...currentScans,
        ...Object.fromEntries(Object.entries(scanHealth).map(([name, health]) => [
          name, { ...(currentScans[name] || {}), ...health },
        ])),
      },
      pointclouds: state.status.pointclouds || {},
      paths: state.status.paths || {},
    };
    const goalStatuses = fresh.navigation?.goal_statuses || [];
    if ([4, 5, 6].includes(Number(goalStatuses[goalStatuses.length - 1]))) {
      mapView.goal = null;
      mapView.preview = null;
    }
    renderStatus();
  } finally { statusRefreshRunning = false; }
}

let visualizationRefreshRunning = false;
async function refreshVisualization() {
  if (visualizationRefreshRunning || document.hidden) return;
  visualizationRefreshRunning = true;
  try {
    const localVersion = Number(state.status.costmaps?.local?.version ?? -1);
    const globalVersion = Number(state.status.costmaps?.global?.version ?? -1);
    const liveMapVersion = Number(state.status.mapping?.live_map?.version ?? -1);
    const trajectoryVersion = Number(state.status.mapping?.trajectory_version ?? -1);
    const visual = await api(`/status/visualization?v=${localVersion},${globalVersion},${liveMapVersion},${trajectoryVersion}`);
    // A TF buffer can retain the last pose after all publishers have stopped.
    // Only use the high-frequency pose when the independent TF health check
    // says the complete chain is currently valid.
    state.status.tf = visual.tf?.ok && visual.pose
      ? { ...visual.pose, _visual: true }
      : (visual.tf || null);
    if (Date.now() - Number(state.status.tf_tree_received_at || 0) > 1000) {
      state.status.tf_tree = visual.tf_tree || { fixed_frame: null, frames: [] };
    }
    state.status.scans = visual.scans || {};
    state.status.pointclouds = visual.pointclouds || {};
    state.status.pointcloud_topics = visual.pointcloud_topics || state.status.pointcloud_topics;
    state.status.pointcloud_health = visual.pointcloud_health || state.status.pointcloud_health;
    state.status.paths = visual.paths || {};
    const previousCostmaps = state.status.costmaps || {};
    state.status.costmaps = Object.fromEntries(Object.entries(visual.costmaps || {}).map(([name, grid]) => [
        name,
        grid.unchanged ? { ...(previousCostmaps[name] || {}), ...grid } : grid,
      ]));
    const previousMapping = state.status.mapping || {};
    const nextMapping = visual.mapping || {};
    const liveMap = nextMapping.live_map?.unchanged
      ? { ...(previousMapping.live_map || {}), ...nextMapping.live_map }
      : nextMapping.live_map;
    state.status.mapping = {
      ...previousMapping,
      ...nextMapping,
      live_map: liveMap,
      trajectory: nextMapping.trajectory ?? previousMapping.trajectory ?? [],
    };
    state.status.motion = visual.motion || null;
    state.status.goal_metrics = visual.goal_metrics || { active: false, distance_remaining: null };
    if (!state.status.goal_metrics.active) {
      mapView.goal = null;
    }
    const pose = currentRobotPose();
    renderCanvasTelemetry(pose);
    renderPoseCanvas(pose);
    renderComponentFrequencies();
  } finally {
    visualizationRefreshRunning = false;
  }
}

let poseRefreshRunning = false;
async function refreshPose() {
  if (poseRefreshRunning || document.hidden) return;
  poseRefreshRunning = true;
  try {
    const result = await api("/status/pose");
    state.status.velocity = result.velocity || { ready: false, external_subscribers: 0 };
    if (result.tf_tree) {
      state.status.tf_tree = result.tf_tree;
      state.status.tf_tree_received_at = Date.now();
    }
    updateControlInterlocks();
    const mappingActive = stepRunning("mapping");
    if (!mappingActive && !result.tf_ok) {
      state.status.tf = result.tf || { ok: false };
      return;
    }
    if (!result.pose || result.pose.child_frame_id !== "base_link") return;
    state.status.robot_pose = { ...result.pose, _received_at: Date.now() };
    if (!mappingActive) state.status.tf = { ...result.pose, _visual: true };
    if (mappingActive) {
      const mapping = state.status.mapping || (state.status.mapping = {});
      const trajectory = mapping.trajectory || (mapping.trajectory = []);
      const last = trajectory[trajectory.length - 1];
      if (!last || Math.hypot(result.pose.x - last[0], result.pose.y - last[1]) >= 0.05) {
        trajectory.push([result.pose.x, result.pose.y, result.pose.yaw]);
        if (trajectory.length > 5000) trajectory.splice(0, trajectory.length - 5000);
      }
    }
    // This lightweight endpoint remains responsive even when the mapping
    // visualization response contains a large live occupancy grid.
    const tfWidget = pointcloudWidgets.find((widget) => widget.visible && widget.type === "tf");
    renderCanvasTelemetry(result.pose);
    mapView.scene?.updateRealtime(
      result.pose,
      tfWidget ? result.tf_tree : null,
      state.status.mapping?.trajectory || [],
      tfWidget && Array.isArray(tfWidget.frames) ? tfWidget.frames : null,
    );
  } finally {
    poseRefreshRunning = false;
  }
}

async function refreshSystem() {
  state.system = await api("/system");
  state.robotProfile = state.system.robot_profile || state.robotProfile;
  // Avoid replacing a focused field during the twice-per-second system poll.
  if (!document.activeElement?.closest(".tcp-config-fields")) renderSystemSteps();
  updateSystemControlButtons();
  renderRobotProfiles();
}

function renderRobotProfiles() {
  const select = $("#robotProfileSelect");
  if (!select) return;
  const profiles = state.robotProfile?.profiles || [];
  const selected = select.value || state.robotProfile?.active;
  select.innerHTML = profiles.map((profile) =>
    `<option value="${escapeHtml(profile.id)}">${escapeHtml(profile.name)}</option>`
  ).join("");
  select.value = profiles.some((profile) => profile.id === selected) ? selected : state.robotProfile?.active;
  const profile = profiles.find((item) => item.id === select.value);
  renderNav2PresetOptions(select.value);
  renderBehaviorTreeOptions(select.value);
  $("#robotProfileDescription").textContent = profile?.description || "暂无机器人档案";
  const running = Boolean(
    state.system?.navigation_running
    || state.system?.startup_in_progress
    || state.system?.shutdown_in_progress
  );
  const applyButton = $("#applyConfigurationBtn");
  applyButton.disabled = !profile || running;
  applyButton.title = running ? "请先停止导航系统和建图" : "下发机器人类型和参数配置";
}

function renderBehaviorTreeOptions(profileId) {
  const select = $("#behaviorTreeSelect");
  if (!select) return;
  const previous = select.value;
  const profile = (state.robotProfile?.profiles || []).find((item) => item.id === profileId);
  const presets = (state.robotProfile?.behavior_trees || []).filter(
    (preset) => preset.profile === profileId,
  );
  select.innerHTML = presets.map((preset) =>
    `<option value="${escapeHtml(preset.id)}">${escapeHtml(preset.name)}（${escapeHtml(preset.filename)}）</option>`
  ).join("");
  const preferred = presets.some((preset) => preset.id === previous)
    ? previous
    : (presets.find((preset) => preset.id === profile?.behavior_tree)?.id || presets[0]?.id || "");
  select.value = preferred;
  select.disabled = !presets.length;
}

function renderNav2PresetOptions(profileId) {
  const select = $("#nav2PresetSelect");
  if (!select) return;
  const previous = select.value;
  const presets = (state.nav2Presets.presets || []).filter(
    (preset) => preset.profile === profileId,
  );
  select.innerHTML = presets.map((preset) =>
    `<option value="${escapeHtml(preset.id)}">${escapeHtml(preset.name)}（${escapeHtml(preset.filename)}）</option>`
  ).join("");
  const preferred = presets.some((preset) => preset.id === previous)
    ? previous
    : (presets.find((preset) => preset.current)?.id || presets[0]?.id || "");
  select.value = preferred;
  select.disabled = !presets.length;
}

async function refreshInspection() {
  state.inspection = await api("/inspection-task");
  renderInspectionStatus();
}

async function refreshMaps() {
  state.maps = await api("/maps");
  renderMaps();
  renderStatus();
}

async function refreshWaypoints() {
  state.waypoints = await api("/waypoints");
  renderWaypoints();
  renderStatus();
}

async function refreshWaypointViews() {
  const [waypoints, maps] = await Promise.all([api("/waypoints"), api("/maps")]);
  state.waypoints = waypoints;
  state.maps = maps;
  renderWaypoints();
  renderMaps();
  renderStatus();
}

async function refreshAll() {
  const [status, system, maps, waypoints, inspection, resources] = await Promise.all([
    api("/status"),
    api("/system"),
    api("/maps"),
    api("/waypoints"),
    api("/inspection-task"),
    api("/status/resources"),
  ]);
  state.status = status;
  state.system = system;
  state.robotProfile = system.robot_profile || state.robotProfile;
  state.maps = maps;
  state.waypoints = waypoints;
  state.inspection = inspection;
  state.resources = resources;
  renderStatus();
  renderSystemSteps();
  renderRobotProfiles();
  renderMaps();
  renderWaypoints();
  renderInspectionStatus();
  renderHostResources();
}

function bindNavigation() {
  $$(".nav-item").forEach((button) => {
    button.addEventListener("click", () => {
      $$(".nav-item").forEach((item) => item.classList.remove("active"));
      $$(".view").forEach((view) => view.classList.remove("active"));
      button.classList.add("active");
      $(`#${button.dataset.view}`).classList.add("active");
      const [title, subtitle] = viewMeta[button.dataset.view];
      $("#viewTitle").textContent = title;
      $("#viewSubtitle").textContent = subtitle;
      if (button.dataset.view === "overview") {
        requestAnimationFrame(() => requestAnimationFrame(() => {
          mapView.scene?.render();
          renderStatus();
        }));
      }
      if (button.dataset.view === "advanced") loadNav2Presets().catch((error) => notify(error.message));
    });
  });
}

async function loadNav2Presets() {
  state.nav2Presets = await api("/advanced/nav2-presets");
  renderNav2PresetOptions($("#robotProfileSelect")?.value || state.robotProfile?.active);
}

function canvasPoint(event) {
  const canvas = $("#poseCanvas");
  const rect = canvas.getBoundingClientRect();
  return { x: (event.clientX - rect.left) * canvas.width / rect.width, y: (event.clientY - rect.top) * canvas.height / rect.height };
}

function setMapTool(tool) {
  mapView.tool = tool;
  $("#panTool").classList.toggle("active", tool === "pan");
  $("#goalTool").classList.toggle("active", tool === "goal");
  $("#initialPoseTool").classList.toggle("active", tool === "initial");
  $("#poseCanvas").classList.toggle("aiming", tool !== "pan");
  $("#mapHint").textContent = tool === "pan" ? "左键旋转，右键或 Shift+左键平移，滚轮缩放" : "在地图上按下并拖动以指定位置和朝向";
}

function clearCanvasTarget({ resetTool = false } = {}) {
  mapView.dragging = false;
  mapView.start = null;
  mapView.last = null;
  mapView.preview = null;
  mapView.goal = null;
  const distanceValue = $("#canvasGoalDistanceValue");
  if (distanceValue) {
    distanceValue.textContent = "X=--  Y=--  W=--  直线=--  路径=--";
  }
  if (resetTool) setMapTool("pan");
  if (mapView.scene) renderPoseCanvas(currentRobotPose());
}

function bindMapInteraction() {
  const canvas = $("#poseCanvas");
  try {
    const style = JSON.parse(localStorage.getItem("coordinateStylesV2") || "{}");
    for (const [id, value] of Object.entries(style)) if ($(`#${id}`) && Number.isFinite(Number(value))) $(`#${id}`).value = value;
  } catch (_) {}
  $("#panTool").addEventListener("click", () => setMapTool("pan"));
  $("#goalTool").addEventListener("click", () => setMapTool("goal"));
  $("#initialPoseTool").addEventListener("click", () => setMapTool("initial"));
  $("#fitMapBtn").addEventListener("click", () => { fitMap(); renderStatus(); });
  $("#topViewBtn").addEventListener("click", () => mapView.scene?.topView());
  $("#pointcloudWidgets").addEventListener("change", async (event) => {
    if (event.target.dataset.tfFrame !== undefined) {
      const frameIndex = Number(event.target.dataset.tfWidget);
      const tfWidget = pointcloudWidgets[frameIndex];
      if (!tfWidget || tfWidget.type !== "tf") return;
      const availableNames = (state.status.tf_tree?.frames || []).map((frame) => frame.name);
      const selected = new Set(Array.isArray(tfWidget.frames) ? tfWidget.frames : availableNames);
      if (event.target.checked) selected.add(event.target.dataset.tfFrame);
      else selected.delete(event.target.dataset.tfFrame);
      tfWidget.frames = availableNames.filter((name) => selected.has(name));
      savePointcloudWidgets();
      // Keep the widget DOM in place so selecting a lower frame does not
      // reset the scroll container to the top on the next status refresh.
      pointcloudWidgetRenderKey = JSON.stringify([pointcloudWidgets, pointcloudTopics()]);
      renderComponentFrequencies();
      renderPoseCanvas(currentRobotPose());
      return;
    }
    const index = Number(event.target.dataset.cloudName ?? event.target.dataset.cloudVisible ?? event.target.dataset.cloudTopic ?? event.target.dataset.cloudColor ?? event.target.dataset.cloudSize ?? event.target.dataset.tfLength);
    if (!Number.isInteger(index) || !pointcloudWidgets[index]) return;
    const widget = pointcloudWidgets[index];
    if (event.target.dataset.cloudName !== undefined) widget.name = event.target.value.trim() || widget.name || `组件 ${index + 1}`;
    if (event.target.dataset.cloudVisible !== undefined) widget.visible = event.target.checked;
    if (event.target.dataset.cloudTopic !== undefined) widget.topic = event.target.value;
    if (event.target.dataset.cloudColor !== undefined) widget.color = event.target.value;
    if (event.target.dataset.cloudSize !== undefined) widget.size = Math.max(1, Math.min(12, Number(event.target.value) || 3));
    if (event.target.dataset.tfLength !== undefined) widget.length = Math.max(0.05, Math.min(5, Number(event.target.value) || 0.35));
    savePointcloudWidgets(); pointcloudWidgetRenderKey = ""; renderPointcloudWidgets(true); renderStatus();
    try { await syncPointcloudSubscriptions(); }
    catch (error) { notify(error.message); }
  });
  $("#pointcloudWidgets").addEventListener("input", (event) => {
    const index = Number(event.target.dataset.cloudColor ?? event.target.dataset.cloudSize ?? event.target.dataset.tfLength);
    if (!Number.isInteger(index) || !pointcloudWidgets[index]) return;
    if (event.target.dataset.cloudColor !== undefined) pointcloudWidgets[index].color = event.target.value;
    if (event.target.dataset.cloudSize !== undefined) pointcloudWidgets[index].size = Math.max(1, Math.min(12, Number(event.target.value) || 3));
    if (event.target.dataset.tfLength !== undefined) pointcloudWidgets[index].length = Math.max(0.05, Math.min(5, Number(event.target.value) || 0.35));
    savePointcloudWidgets();
    pointcloudWidgetRenderKey = JSON.stringify([pointcloudWidgets, pointcloudTopics()]);
    renderPoseCanvas(currentRobotPose());
  });
  $("#pointcloudWidgets").addEventListener("click", async (event) => {
    const toggle = event.target.closest("[data-cloud-toggle]");
    const remove = event.target.closest("[data-cloud-remove]");
    if (toggle) {
      const index = Number(toggle.dataset.cloudToggle);
      pointcloudWidgets[index].collapsed = !pointcloudWidgets[index].collapsed;
      savePointcloudWidgets(); pointcloudWidgetRenderKey = ""; renderPointcloudWidgets(true);
    }
    if (remove) {
      pointcloudWidgets.splice(Number(remove.dataset.cloudRemove), 1);
      savePointcloudWidgets(); pointcloudWidgetRenderKey = ""; renderPointcloudWidgets(true); renderStatus();
      try { await syncPointcloudSubscriptions(); } catch (error) { notify(error.message); }
    }
  });
  $("#addPointcloudWidgetBtn").addEventListener("click", () => {
    const menu = $("#addLayerMenu");
    menu.hidden = !menu.hidden;
  });
  $("#addLayerMenu").addEventListener("click", async (event) => {
    const choice = event.target.closest("[data-add-layer]");
    if (!choice) return;
    const type = choice.dataset.addLayer;
    if (type === "pointcloud") {
      const count = pointcloudWidgets.filter((widget) => (widget.type || "pointcloud") === "pointcloud").length + 1;
      pointcloudWidgets.push({ id: `cloud-${Date.now()}-${Math.random().toString(16).slice(2)}`, type, name: `点云 ${count}`, topic: "/cloud_registered", color: "#ff0000", size: 3, visible: true, collapsed: true });
    } else if (type === "path") {
      const count = pointcloudWidgets.filter((widget) => widget.type === "path").length + 1;
      pointcloudWidgets.push({ id: `path-${Date.now()}-${Math.random().toString(16).slice(2)}`, type, name: `目标路径 ${count}`, topic: "/plan", color: "#7b35d1", size: 3, visible: true, collapsed: true });
    } else if (type === "trajectory") {
      const count = pointcloudWidgets.filter((widget) => widget.type === "trajectory").length + 1;
      pointcloudWidgets.push({ id: `trajectory-${Date.now()}-${Math.random().toString(16).slice(2)}`, type, name: `运动路径 ${count}`, color: "#ffd90d", size: 3, visible: true, collapsed: true });
    } else if (type === "waypoints") {
      const count = pointcloudWidgets.filter((widget) => widget.type === "waypoints").length + 1;
      pointcloudWidgets.push({ id: `waypoints-${Date.now()}-${Math.random().toString(16).slice(2)}`, type, name: `点位 ${count}`, color: "#ff8c00", size: 6, visible: true, collapsed: true });
    } else if (type === "tf") {
      const existing = pointcloudWidgets.find((widget) => widget.type === "tf");
      if (existing) {
        existing.visible = true;
        existing.collapsed = false;
        notify("TF 坐标系组件已存在");
      } else {
        pointcloudWidgets.push({ id: `tf-${Date.now()}`, type, name: "TF 坐标系", length: 0.35, size: 3, visible: true, collapsed: false });
      }
    } else return;
    $("#addLayerMenu").hidden = true;
    savePointcloudWidgets(); pointcloudWidgetRenderKey = ""; renderPointcloudWidgets(true); renderStatus();
    try { await syncPointcloudSubscriptions(); } catch (error) { notify(error.message); }
  });
  $("#mapFrameVisible").addEventListener("change", renderStatus);
  $("#robotFrameVisible").addEventListener("change", renderStatus);
  $("#localCostmapVisible").addEventListener("change", renderStatus);
  $("#globalCostmapVisible").addEventListener("change", renderStatus);
  const updateCoordinateStyles = () => {
    const style = {};
    for (const id of ["mapFrameLength", "mapFrameThickness", "robotFrameLength", "robotFrameThickness"]) style[id] = Number($(`#${id}`).value);
    localStorage.setItem("coordinateStylesV2", JSON.stringify(style));
    renderPoseCanvas(currentRobotPose());
  };
  for (const id of ["mapFrameLength", "mapFrameThickness", "robotFrameLength", "robotFrameThickness"]) $(`#${id}`).addEventListener("input", updateCoordinateStyles);
  $$("[data-builtin-toggle]").forEach((button) => button.addEventListener("click", () => {
    const body = $(`#${button.dataset.builtinToggle}`);
    body.hidden = !body.hidden;
    button.textContent = body.hidden ? "▸" : "▾";
  }));
  canvas.addEventListener("pointerdown", (event) => {
    if (mapView.tool === "pan" && event.button !== 0 && event.button !== 2) return;
    canvas.setPointerCapture(event.pointerId);
    mapView.dragging = true;
    mapView.dragMode = mapView.tool === "pan" && (event.button === 2 || event.shiftKey) ? "translate" : "orbit";
    mapView.start = { clientX: event.clientX, clientY: event.clientY };
    mapView.last = mapView.start;
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!mapView.dragging) return;
    const point = { clientX: event.clientX, clientY: event.clientY };
    if (mapView.tool === "pan" && mapView.scene) {
      const dx = point.clientX - mapView.last.clientX;
      const dy = point.clientY - mapView.last.clientY;
      if (mapView.dragMode === "translate") mapView.scene.pan(dx, dy);
      else mapView.scene.orbit(dx, dy);
    } else if (mapView.scene) {
      const start = mapView.scene.screenToGround(mapView.start.clientX, mapView.start.clientY);
      const end = mapView.scene.screenToGround(point.clientX, point.clientY);
      if (start && end) mapView.preview = {
        x: start.x, y: start.y,
        yaw: Math.atan2(end.y - start.y, end.x - start.x),
        kind: mapView.tool,
      };
    }
    mapView.last = point; renderPoseCanvas(currentRobotPose());
  });
  canvas.addEventListener("pointerup", async (event) => {
    if (!mapView.dragging) return;
    mapView.dragging = false;
    if (mapView.tool !== "pan") {
      const start = mapView.scene?.screenToGround(mapView.start.clientX, mapView.start.clientY);
      const end = mapView.scene?.screenToGround(event.clientX, event.clientY);
      mapView.preview = null;
      if (!start || !end) return;
      const yaw = Math.atan2(end.y - start.y, end.x - start.x);
      const endpoint = mapView.tool === "goal" ? "/navigation/goal" : "/navigation/initial-pose";
      try {
        const result = await api(endpoint, { method: "POST", body: JSON.stringify({ x: start.x, y: start.y, yaw }) });
        if (mapView.tool === "goal") mapView.goal = { ...start, yaw };
        notify(mapView.tool === "goal"
          ? "导航目标已发送"
          : (result.subscriber_count ? "初始位姿已发送" : "初始位姿已缓存，等待全局定位完成地图加载"));
        setMapTool("pan");
      } catch (error) { notify(error.message); }
    }
    renderPoseCanvas(currentRobotPose());
  });
  const cancelCanvasDrag = () => {
    if (!mapView.dragging && !mapView.preview) return;
    mapView.dragging = false;
    mapView.start = null;
    mapView.last = null;
    mapView.preview = null;
    renderPoseCanvas(currentRobotPose());
  };
  canvas.addEventListener("pointercancel", cancelCanvasDrag);
  canvas.addEventListener("lostpointercapture", cancelCanvasDrag);
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    mapView.scene?.zoom(event.deltaY);
  }, { passive: false });
  canvas.addEventListener("contextmenu", (event) => event.preventDefault());
}

function bindActions() {
  $("#refreshBtn").addEventListener("click", () => refreshAll().catch((error) => notify(error.message)));
  $("#logoutBtn").addEventListener("click", async () => {
    try { await api("/auth/logout", { method: "POST" }); }
    finally { window.location.replace("/login"); }
  });
  $("#robotProfileSelect").addEventListener("change", renderRobotProfiles);
  $("#applyConfigurationBtn").addEventListener("click", async () => {
    const profile = $("#robotProfileSelect").value;
    const selected = state.robotProfile.profiles.find((item) => item.id === profile);
    const nav2Preset = $("#nav2PresetSelect").value;
    const behaviorTree = $("#behaviorTreeSelect").value;
    if (!selected) return;
    if (!nav2Preset) { notify("当前机器人类型没有可用的 Nav2 参数预设"); return; }
    if (!behaviorTree) { notify("当前机器人类型没有可用的行为树"); return; }
    if (state.system?.navigation_running || state.system?.startup_in_progress || state.system?.shutdown_in_progress) { notify("请先停止导航系统和建图"); return; }
    if (!window.confirm(`确定应用“${selected.name}”和所选参数吗？应用后需要手动启动测试。`)) return;
    try {
      const result = await api("/configuration/apply", {
        method: "POST", body: JSON.stringify({ profile, nav2_preset: nav2Preset, behavior_tree: behaviorTree }),
      });
      state.robotProfile = result.profile;
      notify(`已应用${selected.name}和参数配置，可以手动启动测试`);
      await refreshAll();
    } catch (error) { notify(error.message); }
  });
  $("#startBtn").addEventListener("click", async () => {
    $("#startBtn").disabled = true;
    clearCanvasTarget({ resetTool: true });
    await api("/system/start", { method: "POST" });
    notify("启动命令已发送");
    await refreshSystem();
  });
  $("#stopBtn").addEventListener("click", async () => {
    $("#startBtn").disabled = true;
    $("#stopBtn").disabled = true;
    $("#restartBtn").disabled = true;
    clearCanvasTarget({ resetTool: true });
    await api("/system/stop", { method: "POST" });
    notify("停止命令已发送");
    await refreshSystem();
  });
  $("#restartBtn").addEventListener("click", async () => {
    $("#startBtn").disabled = true;
    $("#stopBtn").disabled = true;
    $("#restartBtn").disabled = true;
    clearCanvasTarget({ resetTool: true });
    await api("/system/restart", { method: "POST" });
    notify("重启命令已发送");
    await refreshSystem();
  });
  $("#stopMotionBtn").addEventListener("click", async () => {
    try { await api("/navigation/emergency-stop", { method: "POST" }); notify("停止运动命令已发送"); }
    catch (error) { notify(error.message); }
  });
  $("#waypointForm").addEventListener("submit", async (event) => {
    event.preventDefault();
    const form = new FormData(event.currentTarget);
    try {
      const result = await api("/waypoints", {
        method: "POST",
        body: JSON.stringify({
          x: Number(form.get("x")),
          y: Number(form.get("y")),
          yaw: Number(form.get("yaw")),
          note: form.get("note") || "",
        }),
      });
      state.waypoints.waypoints = [...(state.waypoints.waypoints || []), result.waypoint];
      renderWaypoints();
      event.currentTarget.reset();
      notify("点位已添加");
      await refreshWaypointViews();
    } catch (error) { notify(error.message); }
  });
  $("#saveCurrentWaypointBtn").addEventListener("click", async () => {
    try {
      await api("/waypoints/current", {
        method: "POST",
        body: JSON.stringify({ note: $("#currentWaypointNote").value }),
      });
      $("#currentWaypointNote").value = "";
      notify("当前位置已记录为点位"); await refreshWaypointViews();
    } catch (error) { notify(error.message); }
  });
  $("#startWaypointTestBtn").addEventListener("click", async () => {
    const waypoint_indices = $("#testWaypointIndices").value.split(",")
      .map((value) => Number(value.trim()))
      .filter((value) => Number.isInteger(value) && value > 0);
    const cycles = Number($("#testCycles").value);
    if (!waypoint_indices.length) { notify("请输入测试点位顺序"); return; }
    try {
      await api("/navigation/test/start", {
        method: "POST", body: JSON.stringify({ waypoint_indices, cycles }),
      });
      notify("往返测试已开始");
    } catch (error) { notify(error.message); }
  });
  $("#cancelWaypointTestBtn").addEventListener("click", async () => {
    try { await api("/navigation/cancel", { method: "POST" }); notify("测试已停止"); }
    catch (error) { notify(error.message); }
  });
  $("#saveGuideTextBtn").addEventListener("click", () => {
    const content = $("#guideTextContent").value.trim();
    const note = $("#guideTextNote").value.trim();
    if (!content) { notify("请输入文本块内容"); return; }
    if (editingGuideTextId) {
      const block = guideTexts.find((item) => item.id === editingGuideTextId);
      if (block) { block.content = content; block.note = note; }
    } else {
      guideTexts.push({ id: `text-${Date.now()}-${Math.random().toString(16).slice(2)}`, title: `文本 ${guideTexts.length + 1}`, content, note });
    }
    editingGuideTextId = null; $("#guideTextContent").value = ""; $("#guideTextNote").value = "";
    $("#saveGuideTextBtn").textContent = "新增文本块"; $("#cancelGuideTextEditBtn").hidden = true;
    saveGuideTexts(); renderInspectionBuilder();
  });
  $("#cancelGuideTextEditBtn").addEventListener("click", () => {
    editingGuideTextId = null; $("#guideTextContent").value = ""; $("#guideTextNote").value = "";
    $("#saveGuideTextBtn").textContent = "新增文本块"; $("#cancelGuideTextEditBtn").hidden = true;
  });
  $("#insertSelectedWaypointBtn").addEventListener("click", () => {
    const point = state.waypoints.waypoints.find((item) => item.id === selectedGuideWaypointId);
    if (!point) { notify("请先选择一个已存点位"); return; }
    inspectionSequence.push({ type: "waypoint", waypoint_index: point.index, waypoint_id: point.id });
    saveInspectionSequence(); renderInspectionBuilder();
  });
  $("#insertSelectedTextBtn").addEventListener("click", () => {
    const block = guideTexts.find((item) => item.id === selectedGuideTextId);
    if (!block) { notify("请先选择一个文本块"); return; }
    inspectionSequence.push({ type: "text", text_id: block.id, message: block.content });
    saveInspectionSequence(); renderInspectionBuilder();
  });
  $("#insertInspectionWaitBtn").addEventListener("click", () => {
    const seconds = Number($("#inspectionWaitSeconds").value);
    if (!Number.isFinite(seconds) || seconds < 0 || seconds > 86400) { notify("等待时间必须在 0 到 86400 秒之间"); return; }
    inspectionSequence.push({ type: "wait", seconds });
    saveInspectionSequence(); renderInspectionBuilder();
  });
  $("#clearInspectionBtn").addEventListener("click", () => {
    if (state.inspection.running) { notify("请先停止当前导览任务"); return; }
    inspectionSequence = []; saveInspectionSequence(); renderInspectionBuilder();
  });
  $("#startInspectionBtn").addEventListener("click", async () => {
    if (!inspectionSequence.length) { notify("请先编排导览步骤"); return; }
    try {
      state.inspection = await api("/inspection-task/start", {
        method: "POST", body: JSON.stringify({ steps: inspectionSequence, cycles: Number($("#inspectionCycles").value) }),
      });
      renderInspectionStatus(); notify("导览任务已开始");
    } catch (error) { notify(error.message); }
  });
  $("#cancelInspectionBtn").addEventListener("click", async () => {
    state.inspection = await api("/inspection-task/cancel", { method: "POST" });
    renderInspectionStatus(); notify("导览停止命令已发送");
  });
  $("#mapUploadFile").addEventListener("change", async () => {
    const file = $("#mapUploadFile").files[0];
    const name = mapUploadTargetName;
    if (!name || !file) { notify("请选择目标地图和 PGM 文件"); return; }
    if (!window.confirm(`确定用“${file.name}”替换地图“${name}”的 PGM 吗？`)) return;
    try {
      const result = await api(`/maps/upload?name=${encodeURIComponent(name)}`, {
        method: "POST", headers: { "Content-Type": "application/octet-stream" }, body: file,
      });
      $("#mapUploadFile").value = "";
      notify(result.requires_navigation_restart ? "PGM 已回传，请重启定位和导航生效" : "PGM 已回传");
      await refreshMaps();
    } catch (error) { notify(error.message); }
    finally { mapUploadTargetName = null; }
  });
  $("#mappingStartBtn").addEventListener("click", async () => {
    const name = $("#mappingName").value.trim();
    if (!name) { notify("请输入新地图名称"); return; }
    try {
      clearCanvasTarget({ resetTool: true });
      await api("/mapping/start", { method: "POST", body: JSON.stringify({ name }) });
      notify("已进入建图模式；请移动机器人采集环境"); await refreshAll();
    } catch (error) { notify(error.message); }
  });
  $("#mappingSaveBtn").addEventListener("click", async () => {
    try { await api("/mapping/finish", { method: "POST" }); notify("二维地图与 PCD 已保存，建图已停止"); await refreshAll(); }
    catch (error) { notify(error.message); }
  });
  $("#mappingStopBtn").addEventListener("click", async () => {
    try { await api("/mapping/stop", { method: "POST" }); notify("建图已停止"); await refreshAll(); }
    catch (error) { notify(error.message); }
  });
}

function bindRemoteControl() {
  const commands = {
    forward: { linear_x: 0.25 }, backward: { linear_x: -0.2 },
    left: { linear_y: 0.18 }, right: { linear_y: -0.18 },
    "rotate-left": { angular_z: 0.4 }, "rotate-right": { angular_z: -0.4 },
  };
  let speedScale = 1;
  let timer = null;
  let activeButton = null;
  let activeCommand = null;
  const scaledCommand = (command) => Object.fromEntries(
    Object.entries(command || {}).map(([key, value]) => [key, value * speedScale]),
  );
  const publish = async (command) => {
    try {
      await api("/navigation/velocity", {
        method: "POST",
        body: JSON.stringify({ linear_x: 0, linear_y: 0, angular_z: 0, ...command }),
      });
    } catch (error) { notify(error.message); }
  };
  const stop = () => {
    const wasActive = timer !== null || activeButton !== null || activeCommand !== null;
    if (timer !== null) clearInterval(timer);
    timer = null;
    if (activeButton) activeButton.classList.remove("active");
    activeButton = null;
    activeCommand = null;
    // Send one stop command when an actual manual command ends. Status polls,
    // blur events and repeated pointer events must not spam zero velocity.
    if (wasActive) publish({});
  };
  stopRemoteControl = stop;
  $$('[data-speed-scale]').forEach((slider) => {
    slider.addEventListener("input", () => {
      speedScale = Math.min(1.5, Math.max(0.75, Number(slider.value) || 1));
      $$('[data-speed-scale]').forEach((item) => { item.value = String(speedScale); });
      $$('[data-speed-scale-value]').forEach((output) => { output.textContent = `${speedScale.toFixed(2)}×`; });
      if (activeCommand) publish(scaledCommand(activeCommand));
    });
  });
  $$('[data-velocity]').forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      if (button.disabled) return;
      event.preventDefault();
      stop();
      activeButton = button;
      button.classList.add("active");
      activeCommand = commands[button.dataset.velocity];
      publish(scaledCommand(activeCommand));
      timer = setInterval(() => publish(scaledCommand(activeCommand)), 180);
      button.setPointerCapture?.(event.pointerId);
    });
    button.addEventListener("pointerup", stop);
    button.addEventListener("pointercancel", stop);
    button.addEventListener("lostpointercapture", stop);
  });
  window.addEventListener("blur", stop);
  document.addEventListener("visibilitychange", () => { if (document.hidden) stop(); });
}

async function startRoute(rawIndices, loop) {
  const waypoint_indices = String(rawIndices)
    .split(",")
    .map((value) => Number(value.trim()))
    .filter((value) => Number.isInteger(value) && value > 0);
  if (!waypoint_indices.length) {
    notify("请输入点位编号");
    return;
  }
  await api("/navigation/route/start", {
    method: "POST",
    body: JSON.stringify({ waypoint_indices, loop, one_based_index: true }),
  });
  notify("路线已发送");
}

bindNavigation();
bindActions();
bindMapInteraction();
bindRemoteControl();
refreshAll()
  .then(() => syncPointcloudSubscriptions())
  .catch((error) => notify(error.message));
setInterval(() => refreshStatus().catch(() => {}), 500);
setInterval(() => refreshVisualization().catch(() => {}), 200);
setInterval(() => refreshPose().catch(() => {}), 100);
setInterval(() => refreshSystem().catch(() => {}), 500);
setInterval(() => refreshInspection().catch(() => {}), 1000);
setInterval(() => refreshHostResources().catch(() => {}), 2000);
