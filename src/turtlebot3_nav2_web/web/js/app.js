/**
 * TurtleBot3 Nav2 Mission Control Frontend Application
 */

(function () {
  'use strict';

  // --- State Configuration ---
  const state = {
    ws: null,
    connected: false,
    map: {
      loaded: false,
      image: null,
      metadata: null
    },
    robot: {
      x: -2.0,
      y: -0.5,
      yaw: 0.0,
      linearSpeed: 0.0,
      angularSpeed: 0.0,
      trail: []
    },
    target: {
      x: 0.0,
      y: 0.0,
      yaw: 0.0,
      active: false
    },
    plan: [],
    scan: [],
    navStatus: 'IDLE',
    navFeedback: {
      distanceRemaining: null,
      navigationTime: null,
      estimatedTimeRemaining: null
    },
    viewport: {
      scale: 1.0,
      panX: 0.0,
      panY: 0.0
    },
    interaction: {
      mode: 'goal', // 'goal' or 'pose'
      isDraggingMap: false,
      isSettingPose: false,
      startScreen: { x: 0, y: 0 },
      startPan: { x: 0, y: 0 },
      targetStartWorld: { x: 0, y: 0 }
    },
    rooms: {},
    options: {
      showRooms: true,
      showScan: true,
      showPath: true,
      showTrail: true
    },
    teleop: {
      activeKeys: new Set(),
      timer: null,
      speed: 0.18,
      enabled: false,       // Requires explicit enable click before keys work
      deadManTimer: null    // Stops robot if no input for 400ms
    }
  };

  // --- DOM Elements ---
  const canvas = document.getElementById('nav-canvas');
  const ctx = canvas.getContext('2d');
  const container = document.getElementById('canvas-container');

  const connDot = document.getElementById('conn-dot');
  const connText = document.getElementById('conn-text');
  const navStateVal = document.getElementById('nav-state-val');
  const navStatePill = document.getElementById('nav-state-pill');
  const mapBadge = document.getElementById('map-info-badge');

  const valRobotX = document.getElementById('val-robot-x');
  const valRobotY = document.getElementById('val-robot-y');
  const valRobotYaw = document.getElementById('val-robot-yaw');
  const valSpeedLin = document.getElementById('val-speed-lin');
  const valSpeedAng = document.getElementById('val-speed-ang');
  const valDistRemain = document.getElementById('val-dist-remain');

  const targetXInput = document.getElementById('target-x');
  const targetYInput = document.getElementById('target-y');
  const targetYawInput = document.getElementById('target-yaw');
  const targetIndicator = document.getElementById('target-indicator');

  const btnSendGoal = document.getElementById('btn-send-goal');
  const btnCancelGoal = document.getElementById('btn-cancel-goal');
  const btnModeGoal = document.getElementById('btn-mode-goal');
  const btnModePose = document.getElementById('btn-mode-pose');
  const btnCenterRobot = document.getElementById('btn-center-robot');
  const btnFitMap = document.getElementById('btn-fit-map');
  const btnZoomIn = document.getElementById('btn-zoom-in');
  const btnZoomOut = document.getElementById('btn-zoom-out');

  const mouseCoords = document.getElementById('mouse-coords');
  const logConsole = document.getElementById('log-console');
  const btnClearLog = document.getElementById('btn-clear-log');
  const speedSlider = document.getElementById('speed-slider');
  const speedLabel = document.getElementById('speed-label');

  // AI LLM & Rooms Elements
  const toggleRooms = document.getElementById('toggle-rooms');
  const chatForm = document.getElementById('chat-form');
  const chatInput = document.getElementById('chat-input');
  const chatMessages = document.getElementById('chat-messages');
  const promptChips = document.querySelectorAll('.prompt-chip');
  const aiModelBadge = document.getElementById('ai-model-badge');
  const roomChips = document.querySelectorAll('.room-chip');

  // --- Logger Helper ---
  function addLog(text, level = 'info') {
    const entry = document.createElement('div');
    entry.className = `log-entry ${level}`;
    const timestamp = new Date().toLocaleTimeString();
    entry.textContent = `[${timestamp}] ${text}`;
    logConsole.appendChild(entry);
    logConsole.scrollTop = logConsole.scrollHeight;

    // Prune old logs if exceeding 80 entries
    if (logConsole.childElementCount > 80) {
      logConsole.removeChild(logConsole.firstChild);
    }
  }

  btnClearLog.addEventListener('click', () => {
    logConsole.innerHTML = '';
  });

  // --- Resize Canvas ---
  function resizeCanvas() {
    const rect = container.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
    canvas.style.width = `${rect.width}px`;
    canvas.style.height = `${rect.height}px`;
  }
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  // --- Coordinate Transformations ---
  // World (m) -> Map Grid Coordinates
  function worldToGrid(wx, wy) {
    if (!state.map.metadata) return { gx: 0, gy: 0 };
    const meta = state.map.metadata;
    const gx = (wx - meta.origin_x) / meta.resolution;
    // ROS row 0 is bottom. In flipped image, row 0 is top
    const gy = meta.height - (wy - meta.origin_y) / meta.resolution;
    return { gx, gy };
  }

  // Map Grid Coordinates -> World (m)
  function gridToWorld(gx, gy) {
    if (!state.map.metadata) return { wx: 0, wy: 0 };
    const meta = state.map.metadata;
    const wx = meta.origin_x + gx * meta.resolution;
    const wy = meta.origin_y + (meta.height - gy) * meta.resolution;
    return { wx, wy };
  }

  // World (m) -> Screen Pixels (CSS px)
  function worldToScreen(wx, wy) {
    const { gx, gy } = worldToGrid(wx, wy);
    const sx = state.viewport.panX + gx * state.viewport.scale;
    const sy = state.viewport.panY + gy * state.viewport.scale;
    return { sx, sy };
  }

  // Screen Pixels (CSS px) -> World (m)
  function screenToWorld(sx, sy) {
    const gx = (sx - state.viewport.panX) / state.viewport.scale;
    const gy = (sy - state.viewport.panY) / state.viewport.scale;
    return gridToWorld(gx, gy);
  }

  // --- WebSocket Connection ---
  function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    connText.textContent = 'Connecting...';
    connDot.className = 'status-dot disconnected';

    state.ws = new WebSocket(wsUrl);

    state.ws.onopen = () => {
      state.connected = true;
      connText.textContent = 'Connected (WS)';
      connDot.className = 'status-dot connected';
      addLog('WebSocket connection established with Nav2 server.', 'success');
      // Request initial map if not already sent
      state.ws.send(JSON.stringify({ action: 'request_map' }));
    };

    state.ws.onclose = () => {
      state.connected = false;
      connText.textContent = 'Disconnected';
      connDot.className = 'status-dot disconnected';
      addLog('WebSocket disconnected. Retrying in 2s...', 'warning');
      setTimeout(connectWebSocket, 2000);
    };

    state.ws.onerror = (err) => {
      console.error('WebSocket Error:', err);
    };

    state.ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        handleServerMessage(msg);
      } catch (e) {
        console.error('Error parsing incoming message:', e);
      }
    };
  }

  function handleServerMessage(msg) {
    switch (msg.type) {
      case 'map':
        handleMapMessage(msg);
        break;
      case 'robot_state':
        handleRobotStateMessage(msg);
        break;
      case 'plan':
        state.plan = msg.points || [];
        break;
      case 'scan':
        state.scan = msg.points || [];
        break;
      case 'rooms':
        handleRoomsMessage(msg.rooms);
        break;
      case 'llm_response':
        handleLLMResponse(msg);
        break;
      default:
        break;
    }
  }

  function handleMapMessage(msg) {
    state.map.metadata = msg.metadata;
    const img = new Image();
    img.onload = () => {
      state.map.image = img;
      state.map.loaded = true;
      mapBadge.textContent = `${msg.metadata.width}x${msg.metadata.height} (${(msg.metadata.resolution * 100).toFixed(0)}cm/cell)`;
      addLog(`Map received: ${msg.metadata.width}x${msg.metadata.height} @ ${msg.metadata.resolution}m/px`, 'info');
      fitMapToView();
    };
    img.src = `data:image/png;base64,${msg.image}`;
  }

  function handleRobotStateMessage(msg) {
    const p = msg.pose;
    const prevX = state.robot.x;
    const prevY = state.robot.y;

    state.robot.x = p.x;
    state.robot.y = p.y;
    state.robot.yaw = p.yaw;
    state.robot.linearSpeed = msg.speed.linear;
    state.robot.angularSpeed = msg.speed.angular;

    // Append to robot breadcrumb trail if moved significantly
    const distSq = (p.x - prevX) ** 2 + (p.y - prevY) ** 2;
    if (distSq > 0.0025) {
      state.robot.trail.push({ x: p.x, y: p.y });
      if (state.robot.trail.length > 250) {
        state.robot.trail.shift();
      }
    }

    // Update HUD telemetry
    valRobotX.textContent = `${p.x.toFixed(2)} m`;
    valRobotY.textContent = `${p.y.toFixed(2)} m`;
    valRobotYaw.textContent = `${(p.yaw * 180 / Math.PI).toFixed(1)}°`;
    valSpeedLin.textContent = `${msg.speed.linear.toFixed(2)} m/s`;
    valSpeedAng.textContent = `${msg.speed.angular.toFixed(2)} rad/s`;

    // Nav2 Status update
    if (msg.nav_status !== state.navStatus) {
      state.navStatus = msg.nav_status;
      navStateVal.textContent = state.navStatus;
      addLog(`Nav2 Status changed to: ${state.navStatus}`, state.navStatus === 'SUCCEEDED' ? 'success' : (state.navStatus === 'FAILED' ? 'error' : 'info'));
      
      if (state.navStatus === 'SUCCEEDED' || state.navStatus === 'CANCELED' || state.navStatus === 'IDLE') {
        targetIndicator.className = 'status-indicator ready';
        targetIndicator.textContent = 'Ready';
        if (state.navStatus === 'SUCCEEDED') state.target.active = false;
      } else {
        targetIndicator.className = 'status-indicator busy';
        targetIndicator.textContent = state.navStatus;
      }
    }

    // Navigation Feedback
    if (msg.nav_feedback && msg.nav_feedback.distance_remaining !== undefined) {
      state.navFeedback = msg.nav_feedback;
      if (state.navStatus === 'NAVIGATING') {
        valDistRemain.textContent = `${msg.nav_feedback.distance_remaining.toFixed(2)} m`;
      } else {
        valDistRemain.textContent = '--';
      }
    }

    // Update dynamic distances to house rooms
    updateRoomDistances();
  }

  function handleRoomsMessage(rooms) {
    state.rooms = rooms || {};
    updateRoomDistances();
  }

  function updateRoomDistances() {
    if (!state.rooms) return;
    for (const [roomId, r] of Object.entries(state.rooms)) {
      const dist = Math.hypot(state.robot.x - r.x, state.robot.y - r.y);
      const el = document.getElementById(`dist-${roomId}`);
      if (el) {
        el.textContent = `${dist.toFixed(1)}m`;
      }
    }
  }

  // --- Viewport Pan & Zoom Controls ---
  function fitMapToView() {
    if (!state.map.metadata) return;
    const rect = container.getBoundingClientRect();
    const meta = state.map.metadata;
    const scaleX = (rect.width - 60) / meta.width;
    const scaleY = (rect.height - 60) / meta.height;
    state.viewport.scale = Math.min(scaleX, scaleY);
    state.viewport.panX = (rect.width - meta.width * state.viewport.scale) / 2;
    state.viewport.panY = (rect.height - meta.height * state.viewport.scale) / 2;
  }

  function centerOnRobot() {
    if (!state.map.metadata) return;
    const rect = container.getBoundingClientRect();
    const { gx, gy } = worldToGrid(state.robot.x, state.robot.y);
    state.viewport.panX = rect.width / 2 - gx * state.viewport.scale;
    state.viewport.panY = rect.height / 2 - gy * state.viewport.scale;
  }

  function zoomAt(factor, clientX, clientY) {
    const rect = container.getBoundingClientRect();
    const mouseX = clientX - rect.left;
    const mouseY = clientY - rect.top;

    const newScale = Math.max(0.2, Math.min(10.0, state.viewport.scale * factor));
    state.viewport.panX = mouseX - (mouseX - state.viewport.panX) * (newScale / state.viewport.scale);
    state.viewport.panY = mouseY - (mouseY - state.viewport.panY) * (newScale / state.viewport.scale);
    state.viewport.scale = newScale;
  }

  btnCenterRobot.addEventListener('click', centerOnRobot);
  btnFitMap.addEventListener('click', fitMapToView);
  btnZoomIn.addEventListener('click', () => {
    const rect = container.getBoundingClientRect();
    zoomAt(1.25, rect.left + rect.width / 2, rect.top + rect.height / 2);
  });
  btnZoomOut.addEventListener('click', () => {
    const rect = container.getBoundingClientRect();
    zoomAt(0.8, rect.left + rect.width / 2, rect.top + rect.height / 2);
  });

  // Mouse wheel zoom
  container.addEventListener('wheel', (e) => {
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.12 : 0.89;
    zoomAt(factor, e.clientX, e.clientY);
  }, { passive: false });

  // Mode Selection
  btnModeGoal.addEventListener('click', () => {
    state.interaction.mode = 'goal';
    btnModeGoal.classList.add('active');
    btnModePose.classList.remove('active');
    addLog('Interactive Mode: Set Navigation Goal', 'info');
  });

  btnModePose.addEventListener('click', () => {
    state.interaction.mode = 'pose';
    btnModePose.classList.add('active');
    btnModeGoal.classList.remove('active');
    addLog('Interactive Mode: Set 2D Initial Pose (AMCL)', 'info');
  });

  // --- Mouse & Canvas Interaction ---
  container.addEventListener('mousedown', (e) => {
    const rect = container.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // Right-click or Middle-click drags/pans the map
    if (e.button === 1 || e.button === 2) {
      state.interaction.isDraggingMap = true;
      state.interaction.startScreen = { x: sx, y: sy };
      state.interaction.startPan = { x: state.viewport.panX, y: state.viewport.panY };
      e.preventDefault();
      return;
    }

    // Left-click sets target goal or initial pose
    if (e.button === 0) {
      const worldPt = screenToWorld(sx, sy);
      state.interaction.isSettingPose = true;
      state.interaction.targetStartWorld = { x: worldPt.wx, y: worldPt.wy };

      if (state.interaction.mode === 'goal') {
        state.target.x = worldPt.wx;
        state.target.y = worldPt.wy;
        state.target.yaw = 0.0;
        state.target.active = true;
        updateTargetInputs(worldPt.wx, worldPt.wy, 0.0);
      }
    }
  });

  window.addEventListener('mousemove', (e) => {
    const rect = container.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // Update cursor coordinate HUD
    if (sx >= 0 && sx <= rect.width && sy >= 0 && sy <= rect.height) {
      const { wx, wy } = screenToWorld(sx, sy);
      mouseCoords.textContent = `Cursor: X: ${wx.toFixed(2)} m, Y: ${wy.toFixed(2)} m`;
    }

    // Pan map
    if (state.interaction.isDraggingMap) {
      const dx = sx - state.interaction.startScreen.x;
      const dy = sy - state.interaction.startScreen.y;
      state.viewport.panX = state.interaction.startPan.x + dx;
      state.viewport.panY = state.interaction.startPan.y + dy;
      return;
    }

    // Drag to specify orientation
    if (state.interaction.isSettingPose) {
      const currentWorld = screenToWorld(sx, sy);
      const dx = currentWorld.wx - state.interaction.targetStartWorld.x;
      const dy = currentWorld.wy - state.interaction.targetStartWorld.y;

      let yaw = Math.atan2(dy, dx);
      if (Math.hypot(dx, dy) < 0.1) {
        yaw = 0.0; // Click without drag keeps 0 rad
      }

      if (state.interaction.mode === 'goal') {
        state.target.yaw = yaw;
        updateTargetInputs(state.target.x, state.target.y, yaw);
      }
    }
  });

  window.addEventListener('mouseup', (e) => {
    if (state.interaction.isDraggingMap) {
      state.interaction.isDraggingMap = false;
    }

    if (state.interaction.isSettingPose) {
      state.interaction.isSettingPose = false;

      const rect = container.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const currentWorld = screenToWorld(sx, sy);
      const dx = currentWorld.wx - state.interaction.targetStartWorld.x;
      const dy = currentWorld.wy - state.interaction.targetStartWorld.y;
      const yaw = Math.hypot(dx, dy) >= 0.1 ? Math.atan2(dy, dx) : 0.0;

      if (state.interaction.mode === 'pose') {
        // Dispatch AMCL Initial Pose
        sendInitialPose(state.interaction.targetStartWorld.x, state.interaction.targetStartWorld.y, yaw);
      } else {
        addLog(`Target selected at (${state.target.x.toFixed(2)}, ${state.target.y.toFixed(2)}) heading ${(state.target.yaw * 180 / Math.PI).toFixed(0)}°`, 'info');
      }
    }
  });

  container.addEventListener('contextmenu', (e) => e.preventDefault());

  function updateTargetInputs(x, y, yaw) {
    targetXInput.value = x.toFixed(2);
    targetYInput.value = y.toFixed(2);
    targetYawInput.value = (yaw * 180 / Math.PI).toFixed(0);
  }

  // Target input change listener
  [targetXInput, targetYInput, targetYawInput].forEach(inp => {
    inp.addEventListener('input', () => {
      state.target.x = parseFloat(targetXInput.value) || 0.0;
      state.target.y = parseFloat(targetYInput.value) || 0.0;
      state.target.yaw = (parseFloat(targetYawInput.value) || 0.0) * Math.PI / 180;
      state.target.active = true;
    });
  });

  // Waypoint Presets
  document.querySelectorAll('.preset-chip').forEach(btn => {
    btn.addEventListener('click', () => {
      const x = parseFloat(btn.dataset.x);
      const y = parseFloat(btn.dataset.y);
      const yawDeg = parseFloat(btn.dataset.yaw);
      const yaw = yawDeg * Math.PI / 180;

      state.target.x = x;
      state.target.y = y;
      state.target.yaw = yaw;
      state.target.active = true;
      updateTargetInputs(x, y, yaw);
      addLog(`Selected preset waypoint: (${x}, ${y})`, 'info');
    });
  });

  // House Room Quick Chips
  roomChips.forEach(chip => {
    chip.addEventListener('click', () => {
      const x = parseFloat(chip.dataset.x);
      const y = parseFloat(chip.dataset.y);
      const yawDeg = parseFloat(chip.dataset.yaw);
      const yaw = yawDeg * Math.PI / 180;
      const roomName = chip.querySelector('.room-name').textContent;

      state.target.x = x;
      state.target.y = y;
      state.target.yaw = yaw;
      state.target.active = true;
      updateTargetInputs(x, y, yaw);

      // Highlight active room chip
      roomChips.forEach(c => c.classList.remove('active-room'));
      chip.classList.add('active-room');

      // Dispatch navigation goal directly
      if (state.connected) {
        state.ws.send(JSON.stringify({
          action: 'navigate',
          x: x,
          y: y,
          yaw: yaw
        }));
        addLog(`Direct Nav to Room: ${roomName} -> (${x.toFixed(2)}, ${y.toFixed(2)})`, 'info');
      }
    });
  });

  // --- AI LLM Voice & Chat Navigator Handlers ---
  function sendLLMPrompt(promptText) {
    if (!promptText || !promptText.trim()) return;
    promptText = promptText.trim();

    if (!state.connected) {
      alert('Local Nav2 server is not connected!');
      return;
    }

    // Append user message bubble
    appendChatMessage('user', promptText);

    // Append thinking placeholder
    const thinkingId = 'thinking-' + Date.now();
    const thinkingEl = document.createElement('div');
    thinkingEl.className = 'chat-msg assistant thinking-placeholder';
    thinkingEl.id = thinkingId;
    thinkingEl.innerHTML = `
      <div class="msg-avatar">🤖</div>
      <div class="msg-content">
        <div class="msg-bubble chat-thinking">
          <span class="ai-sparkle">✨</span>
          <span>Reasoning with local LLM...</span>
        </div>
      </div>
    `;
    chatMessages.appendChild(thinkingEl);
    chatMessages.scrollTop = chatMessages.scrollHeight;

    // Send action over WebSocket
    state.ws.send(JSON.stringify({
      action: 'llm_chat',
      prompt: promptText
    }));

    addLog(`Sent prompt to LLM: "${promptText}"`, 'info');
  }

  function handleLLMResponse(msg) {
    // Remove thinking placeholder
    document.querySelectorAll('.thinking-placeholder').forEach(el => el.remove());

    const result = msg.result || {};
    const replyText = result.reply || "Command received.";
    const thought = result.thought;
    const model = result.model;

    if (model && aiModelBadge) {
      aiModelBadge.textContent = model.split(' ')[0];
    }

    // Build assistant response bubble
    const msgEl = document.createElement('div');
    msgEl.className = 'chat-msg assistant';

    let contentHtml = `
      <div class="msg-avatar">🤖</div>
      <div class="msg-content">
        <div class="msg-bubble">
          ${escapeHtml(replyText).replace(/\n/g, '<br>')}
    `;

    if (result.action === 'navigate' && result.room_name) {
      contentHtml += `
          <div class="chat-target-tag">
            <span>🎯</span>
            <span>Target: <strong>${escapeHtml(result.room_name)}</strong> (${result.x.toFixed(2)}, ${result.y.toFixed(2)})</span>
          </div>
      `;

      // Update active target coordinates on map
      state.target.x = result.x;
      state.target.y = result.y;
      state.target.yaw = result.yaw || 0.0;
      state.target.active = true;
      updateTargetInputs(result.x, result.y, result.yaw || 0.0);

      // Highlight the target room chip
      if (result.target_room) {
        roomChips.forEach(c => {
          if (c.dataset.room === result.target_room) c.classList.add('active-room');
          else c.classList.remove('active-room');
        });
      }

      addLog(`AI dispatched goal -> ${result.room_name} (${result.x.toFixed(2)}, ${result.y.toFixed(2)})`, 'success');
    }

    contentHtml += `</div>`;

    if (thought) {
      contentHtml += `<div class="chat-thought">💭 ${escapeHtml(thought)}</div>`;
    }

    contentHtml += `</div>`;
    msgEl.innerHTML = contentHtml;
    chatMessages.appendChild(msgEl);
    chatMessages.scrollTop = chatMessages.scrollHeight;
  }

  function appendChatMessage(role, text) {
    const msgEl = document.createElement('div');
    msgEl.className = `chat-msg ${role}`;
    if (role === 'user') {
      msgEl.innerHTML = `
        <div class="msg-content">
          <div class="msg-bubble">${escapeHtml(text)}</div>
        </div>
      `;
    }
    chatMessages.appendChild(msgEl);
    chatMessages.scrollTop = chatMessages.scrollHeight;
  }

  function escapeHtml(str) {
    if (!str) return '';
    const div = document.createElement('div');
    div.textContent = str;
    return div.innerHTML;
  }

  // LLM Chat form submit
  if (chatForm) {
    chatForm.addEventListener('submit', (e) => {
      e.preventDefault();
      const text = chatInput.value;
      if (text) {
        sendLLMPrompt(text);
        chatInput.value = '';
      }
    });
  }

  // LLM Prompt Chips
  promptChips.forEach(chip => {
    chip.addEventListener('click', () => {
      sendLLMPrompt(chip.dataset.prompt);
    });
  });

  // --- Goal Dispatch and Control ---
  btnSendGoal.addEventListener('click', () => {
    if (!state.connected) {
      alert('Local Nav2 server is not connected!');
      return;
    }
    const x = parseFloat(targetXInput.value);
    const y = parseFloat(targetYInput.value);
    const yawDeg = parseFloat(targetYawInput.value);
    const yaw = yawDeg * Math.PI / 180;

    state.target.x = x;
    state.target.y = y;
    state.target.yaw = yaw;
    state.target.active = true;

    state.ws.send(JSON.stringify({
      action: 'navigate',
      x: x,
      y: y,
      yaw: yaw
    }));

    addLog(`Sent Navigation Goal -> (${x.toFixed(2)}, ${y.toFixed(2)}, yaw: ${yawDeg.toFixed(0)}°)`, 'info');
  });

  btnCancelGoal.addEventListener('click', () => {
    if (!state.connected) return;
    state.ws.send(JSON.stringify({ action: 'cancel' }));
    addLog('Emergency Cancel / Stop command sent.', 'warning');
  });

  function sendInitialPose(x, y, yaw) {
    if (!state.connected) return;
    state.ws.send(JSON.stringify({
      action: 'initial_pose',
      x: x,
      y: y,
      yaw: yaw
    }));
    addLog(`Sent Initial Pose -> (${x.toFixed(2)}, ${y.toFixed(2)}, yaw: ${(yaw * 180 / Math.PI).toFixed(0)}°)`, 'success');
  }

  // --- Teleop Controls ---
  speedSlider.addEventListener('input', () => {
    state.teleop.speed = parseFloat(speedSlider.value);
    speedLabel.textContent = state.teleop.speed.toFixed(2);
  });

  function sendTeleop(lin, ang) {
    if (!state.connected) return;
    // Don't send movement if teleop not explicitly enabled by user
    if (!state.teleop.enabled && (lin !== 0 || ang !== 0)) return;
    state.ws.send(JSON.stringify({
      action: 'teleop',
      linear: lin,
      angular: ang
    }));
    // Dead-man's switch: auto-stop after 400ms of no new input
    if (state.teleop.deadManTimer) clearTimeout(state.teleop.deadManTimer);
    if (lin !== 0 || ang !== 0) {
      state.teleop.deadManTimer = setTimeout(() => {
        state.ws.send(JSON.stringify({ action: 'teleop', linear: 0, angular: 0 }));
      }, 400);
    }
  }

  function handleTeleopDirection(action) {
    state.teleop.enabled = true; // Actively allow movement from button click
    let lin = 0.0;
    let ang = 0.0;
    const maxLin = state.teleop.speed;
    const maxAng = 1.0;

    if (action === 'forward') lin = maxLin;
    else if (action === 'backward') lin = -maxLin;
    else if (action === 'left') ang = maxAng;
    else if (action === 'right') ang = -maxAng;
    else if (action === 'stop') {
      sendTeleop(0, 0);
      return;
    }
    sendTeleop(lin, ang);
  }

  // On-screen keypad buttons
  document.querySelectorAll('.key-btn').forEach(btn => {
    const startAction = () => {
      const action = btn.dataset.action;
      handleTeleopDirection(action);
    };
    const stopAction = () => {
      sendTeleop(0, 0);
    };

    btn.addEventListener('mousedown', startAction);
    btn.addEventListener('mouseup', stopAction);
    btn.addEventListener('mouseleave', stopAction);
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); startAction(); });
    btn.addEventListener('touchend', (e) => { e.preventDefault(); stopAction(); });
  });

  // Global blur listener: emergency stop when window or tab loses focus
  window.addEventListener('blur', () => {
    state.teleop.activeKeys.clear();
    if (state.teleop.timer) {
      clearInterval(state.teleop.timer);
      state.teleop.timer = null;
    }
    sendTeleop(0, 0);
  });

  // Keyboard navigation — only active when teleop panel is focused
  const teleopArea = document.getElementById('teleop-area');
  if (teleopArea) {
    teleopArea.addEventListener('focus', () => {
      state.teleop.enabled = true;
    });
    teleopArea.addEventListener('blur', () => {
      state.teleop.enabled = false;
      // Stop robot when teleop loses focus
      state.teleop.activeKeys.clear();
      sendTeleop(0, 0);
    });
  }

  window.addEventListener('keydown', (e) => {
    if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
    if (!state.teleop.enabled) return;  // Must click teleop panel first
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd', ' ', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
      e.preventDefault();
      if (!state.teleop.activeKeys.has(key)) {
        state.teleop.activeKeys.add(key);
        processKeyTeleop();
      }
    }
  });

  window.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (state.teleop.activeKeys.has(key)) {
      state.teleop.activeKeys.delete(key);
      processKeyTeleop();
    }
  });

  function processKeyTeleop() {
    let lin = 0.0;
    let ang = 0.0;
    const maxLin = state.teleop.speed;
    const maxAng = 1.0;

    if (state.teleop.activeKeys.has('w') || state.teleop.activeKeys.has('arrowup')) lin += maxLin;
    if (state.teleop.activeKeys.has('s') || state.teleop.activeKeys.has('arrowdown')) lin -= maxLin;
    if (state.teleop.activeKeys.has('a') || state.teleop.activeKeys.has('arrowleft')) ang += maxAng;
    if (state.teleop.activeKeys.has('d') || state.teleop.activeKeys.has('arrowright')) ang -= maxAng;
    if (state.teleop.activeKeys.has(' ')) { lin = 0; ang = 0; }

    sendTeleop(lin, ang);

    if (state.teleop.activeKeys.size > 0) {
      if (!state.teleop.timer) {
        state.teleop.timer = setInterval(processKeyTeleop, 100);
      }
    } else {
      if (state.teleop.timer) {
        clearInterval(state.teleop.timer);
        state.teleop.timer = null;
        sendTeleop(0, 0);
      }
    }
  }

  // Layer Toggles
  if (toggleRooms) {
    toggleRooms.addEventListener('change', (e) => {
      state.options.showRooms = e.target.checked;
    });
  }
  document.getElementById('toggle-scan').addEventListener('change', (e) => {
    state.options.showScan = e.target.checked;
  });
  document.getElementById('toggle-path').addEventListener('change', (e) => {
    state.options.showPath = e.target.checked;
  });
  document.getElementById('toggle-trail').addEventListener('change', (e) => {
    state.options.showTrail = e.target.checked;
  });

  // --- Rendering Loop ---
  function render() {
    const width = container.clientWidth;
    const height = container.clientHeight;

    // Clear canvas
    ctx.clearRect(0, 0, width, height);

    // Draw subtle grid pattern
    drawBackgroundGrid(width, height);

    ctx.save();
    // Apply viewport transform (pan & zoom)
    ctx.translate(state.viewport.panX, state.viewport.panY);
    ctx.scale(state.viewport.scale, state.viewport.scale);

    // 1. Draw Occupancy Grid Map
    if (state.map.loaded && state.map.image) {
      ctx.drawImage(state.map.image, 0, 0, state.map.metadata.width, state.map.metadata.height);

      // Draw map boundary line
      ctx.strokeStyle = '#374151';
      ctx.lineWidth = 1 / state.viewport.scale;
      ctx.strokeRect(0, 0, state.map.metadata.width, state.map.metadata.height);
    }

    // 1.5. Draw House Room Zones & Labels
    if (state.options.showRooms && state.rooms && state.map.metadata) {
      drawRoomLabels();
    }

    // 2. Draw Robot Breadcrumb Trail
    if (state.options.showTrail && state.robot.trail.length > 1) {
      ctx.beginPath();
      const first = worldToGrid(state.robot.trail[0].x, state.robot.trail[0].y);
      ctx.moveTo(first.gx, first.gy);
      for (let i = 1; i < state.robot.trail.length; i++) {
        const pt = worldToGrid(state.robot.trail[i].x, state.robot.trail[i].y);
        ctx.lineTo(pt.gx, pt.gy);
      }
      ctx.strokeStyle = 'rgba(6, 182, 212, 0.35)';
      ctx.lineWidth = 2.0 / state.viewport.scale;
      ctx.stroke();
    }

    // 3. Draw Planned Nav2 Path
    if (state.options.showPath && state.plan.length > 1) {
      ctx.beginPath();
      const startPt = worldToGrid(state.plan[0].x, state.plan[0].y);
      ctx.moveTo(startPt.gx, startPt.gy);
      for (let i = 1; i < state.plan.length; i++) {
        const pt = worldToGrid(state.plan[i].x, state.plan[i].y);
        ctx.lineTo(pt.gx, pt.gy);
      }
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 3.0 / state.viewport.scale;
      ctx.shadowColor = '#0284c7';
      ctx.shadowBlur = 8;
      ctx.stroke();
      ctx.shadowBlur = 0; // reset
    }

    // 4. Draw LiDAR Scan (Obstacle Hits)
    if (state.options.showScan && state.scan.length > 0) {
      ctx.fillStyle = '#ef4444';
      const scanRadius = Math.max(1.0, 1.8 / state.viewport.scale);
      for (let i = 0; i < state.scan.length; i++) {
        const pt = worldToGrid(state.scan[i].x, state.scan[i].y);
        ctx.beginPath();
        ctx.arc(pt.gx, pt.gy, scanRadius, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // 5. Draw Navigation Target Marker
    if (state.target.active && state.map.metadata) {
      const { gx, gy } = worldToGrid(state.target.x, state.target.y);
      drawTargetMarker(gx, gy, state.target.yaw);
    }

    // 6. Draw Robot Footprint & Orientation
    if (state.map.metadata) {
      const { gx, gy } = worldToGrid(state.robot.x, state.robot.y);
      drawRobot(gx, gy, state.robot.yaw);
    }

    ctx.restore();

    requestAnimationFrame(render);
  }

  function drawBackgroundGrid(w, h) {
    const gridSize = 30;
    ctx.strokeStyle = '#111827';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = 0; x < w; x += gridSize) {
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
    }
    for (let y = 0; y < h; y += gridSize) {
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
    }
    ctx.stroke();
  }

  function drawTargetMarker(gx, gy, yaw) {
    ctx.save();
    ctx.translate(gx, gy);
    // Invert yaw because canvas Y is flipped relative to ROS Cartesian Y
    ctx.rotate(-yaw);

    const baseRadius = 8 / state.viewport.scale;

    // Glowing target circle
    ctx.beginPath();
    ctx.arc(0, 0, baseRadius * 1.8, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(16, 185, 129, 0.2)';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(0, 0, baseRadius, 0, Math.PI * 2);
    ctx.fillStyle = '#10b981';
    ctx.shadowColor = '#10b981';
    ctx.shadowBlur = 10;
    ctx.fill();
    ctx.shadowBlur = 0;

    // Target crosshair
    ctx.strokeStyle = '#022c22';
    ctx.lineWidth = 1.5 / state.viewport.scale;
    ctx.beginPath();
    ctx.moveTo(-baseRadius, 0);
    ctx.lineTo(baseRadius, 0);
    ctx.moveTo(0, -baseRadius);
    ctx.lineTo(0, baseRadius);
    ctx.stroke();

    // Directional Arrow
    const arrowLen = baseRadius * 2.8;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(arrowLen, 0);
    ctx.lineTo(arrowLen - baseRadius * 0.7, -baseRadius * 0.5);
    ctx.moveTo(arrowLen, 0);
    ctx.lineTo(arrowLen - baseRadius * 0.7, baseRadius * 0.5);
    ctx.strokeStyle = '#34d399';
    ctx.lineWidth = 2.5 / state.viewport.scale;
    ctx.stroke();

    ctx.restore();
  }

  function drawRoomLabels() {
    for (const [roomId, r] of Object.entries(state.rooms)) {
      const { gx, gy } = worldToGrid(r.x, r.y);
      const color = r.color || '#38bdf8';

      ctx.save();
      ctx.translate(gx, gy);

      // 1. Soft glowing room ambient circle
      const zoneRadius = 14.0 / state.viewport.scale;
      const grad = ctx.createRadialGradient(0, 0, 1, 0, 0, zoneRadius * 2);
      grad.addColorStop(0, color + '55');
      grad.addColorStop(0.6, color + '22');
      grad.addColorStop(1, 'transparent');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(0, 0, zoneRadius * 2, 0, Math.PI * 2);
      ctx.fill();

      // 2. Room Center Pin
      ctx.beginPath();
      ctx.arc(0, 0, 4 / state.viewport.scale, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.shadowColor = color;
      ctx.shadowBlur = 8;
      ctx.fill();
      ctx.shadowBlur = 0;

      // 3. Room Label Badge Pill
      const screenFontSize = 11.5;
      const gridFontSize = screenFontSize / state.viewport.scale;
      ctx.font = `600 ${gridFontSize}px "Plus Jakarta Sans", -apple-system, sans-serif`;

      const labelText = `${r.icon || '📍'} ${r.name}`;
      const textMetrics = ctx.measureText(labelText);
      const padX = 7 / state.viewport.scale;
      const padY = 3.5 / state.viewport.scale;
      const pillWidth = textMetrics.width + padX * 2;
      const pillHeight = gridFontSize + padY * 2;
      const pillY = -zoneRadius - pillHeight;

      // Pill Background
      ctx.fillStyle = 'rgba(11, 15, 25, 0.9)';
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.3 / state.viewport.scale;

      const rx = -pillWidth / 2;
      const ry = pillY;
      const radius = 4 / state.viewport.scale;

      ctx.beginPath();
      ctx.moveTo(rx + radius, ry);
      ctx.lineTo(rx + pillWidth - radius, ry);
      ctx.quadraticCurveTo(rx + pillWidth, ry, rx + pillWidth, ry + radius);
      ctx.lineTo(rx + pillWidth, ry + pillHeight - radius);
      ctx.quadraticCurveTo(rx + pillWidth, ry + pillHeight, rx + pillWidth - radius, ry + pillHeight);
      ctx.lineTo(rx + radius, ry + pillHeight);
      ctx.quadraticCurveTo(rx, ry + pillHeight, rx, ry + pillHeight - radius);
      ctx.lineTo(rx, ry + radius);
      ctx.quadraticCurveTo(rx, ry, rx + radius, ry);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Label text
      ctx.fillStyle = '#ffffff';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(labelText, 0, pillY + pillHeight / 2);

      ctx.restore();
    }
  }

  function drawRobot(gx, gy, yaw) {
    ctx.save();
    ctx.translate(gx, gy);
    // Invert yaw because canvas Y is flipped relative to ROS Cartesian Y
    ctx.rotate(-yaw);

    // Robot body radius in grid cells (approx 0.15m / 0.05m = 3 cells)
    const res = state.map.metadata.resolution;
    const robotRadiusCells = Math.max(0.16 / res, 4);

    // Robot base circle (chassis)
    ctx.beginPath();
    ctx.arc(0, 0, robotRadiusCells, 0, Math.PI * 2);
    ctx.fillStyle = '#0284c7';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2.0 / state.viewport.scale;
    ctx.shadowColor = '#06b6d4';
    ctx.shadowBlur = 12;
    ctx.fill();
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Left and right wheels
    const wheelW = robotRadiusCells * 0.5;
    const wheelH = robotRadiusCells * 0.22;
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 1.0 / state.viewport.scale;
    // Left wheel (+Y in robot frame is top in standard view, but flipped canvas means -Y)
    ctx.fillRect(-wheelW / 2, -robotRadiusCells - wheelH * 0.6, wheelW, wheelH);
    ctx.strokeRect(-wheelW / 2, -robotRadiusCells - wheelH * 0.6, wheelW, wheelH);
    // Right wheel
    ctx.fillRect(-wheelW / 2, robotRadiusCells - wheelH * 0.4, wheelW, wheelH);
    ctx.strokeRect(-wheelW / 2, robotRadiusCells - wheelH * 0.4, wheelW, wheelH);

    // LiDAR puck in center
    ctx.beginPath();
    ctx.arc(0, 0, robotRadiusCells * 0.35, 0, Math.PI * 2);
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 1.5 / state.viewport.scale;
    ctx.fill();
    ctx.stroke();

    // Prominent Heading Arrow pointing along +X
    ctx.beginPath();
    ctx.moveTo(robotRadiusCells * 0.35, 0);
    ctx.lineTo(robotRadiusCells * 1.5, 0);
    ctx.lineTo(robotRadiusCells * 1.1, -robotRadiusCells * 0.35);
    ctx.moveTo(robotRadiusCells * 1.5, 0);
    ctx.lineTo(robotRadiusCells * 1.1, robotRadiusCells * 0.35);
    ctx.strokeStyle = '#fef08a';
    ctx.lineWidth = 2.5 / state.viewport.scale;
    ctx.stroke();

    ctx.restore();
  }

  // --- Start App ---
  connectWebSocket();
  requestAnimationFrame(render);

})();
