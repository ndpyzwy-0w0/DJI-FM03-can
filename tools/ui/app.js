"use strict";
const byId = (id) => document.getElementById(id);
let token = "", busy = false, latestState = null, timer = null;
const count = (n) => Number(n || 0).toLocaleString("zh-CN");
const size = (n) => n < 1024 ? `${n} B` : n < 1048576 ? `${(n / 1024).toFixed(1)} KB` : `${(n / 1048576).toFixed(2)} MB`;
const duration = (seconds) => {const n = Math.floor(seconds);return `${String(Math.floor(n / 60)).padStart(2, "0")}:${String(n % 60).padStart(2, "0")}`;};
let audio = null, candidateKey = "", zeroEvents = null;

function drawCandidate(channels) {
  const canvas = byId("candidateChart"), ctx = canvas.getContext("2d");
  const width = canvas.clientWidth || 1000, scale = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * scale);canvas.height = Math.round(160 * scale);ctx.scale(scale,scale);
  ctx.clearRect(0, 0, width, 160);
  const all = channels.flatMap((c) => c.history || []);
  if (!all.length) return;
  const end = Math.max(...all.map((p) => p[0])), start = Math.max(0, end - 10);
  const bound = Math.max(100, ...all.map((p) => Math.abs(p[1])));
  const x = (t) => 55 + (t - start) / Math.max(1, end - start) * (width - 70);
  const y = (v) => 15 + (bound - v) / (2 * bound) * 110;
  const style = getComputedStyle(document.documentElement);
  ctx.font = "11px Segoe UI";
  for (const v of [-bound, 0, bound]) {
    ctx.strokeStyle = style.getPropertyValue("--line");ctx.setLineDash([]);ctx.beginPath();ctx.moveTo(55,y(v));ctx.lineTo(width-15,y(v));ctx.stroke();
    ctx.fillStyle = style.getPropertyValue("--muted");ctx.fillText(String(v),5,y(v)+4);
  }
  ctx.fillText(`${start.toFixed(1)} s`,55,148);ctx.fillText(`${end.toFixed(1)} s`,width-65,148);
  channels.forEach((channel,index) => {
    ctx.strokeStyle = style.getPropertyValue(index ? "--muted" : "--accent");ctx.setLineDash(index ? [4,3] : []);ctx.lineWidth = 1.5;ctx.beginPath();
    let first = true;
    for (const [t,v] of channel.history || []) {if (t < start) continue;if(first)ctx.moveTo(x(t),y(v));else ctx.lineTo(x(t),y(v));first = false;}
    ctx.stroke();
  });
}

function renderCandidate(state, active) {
  const channels = state?.candidate?.channels || [];
  const key = `${state?.started_at || ""}:${state?.candidate?.segment ?? ""}`;
  if (key !== candidateKey) {candidateKey = key;zeroEvents = null;}
  for (const suffix of ["503","201"]) {
    const channel = channels.find((c) => c.can_id === `0x${suffix}`);
    const fresh = active && channel?.value != null && channel.age_s <= 0.5;
    byId(`value${suffix}`).textContent = channel?.value == null || (active && !fresh) ? "—" : String(channel.value);
    byId(`candidate${suffix}`).className = `candidate-channel ${fresh && channel.value === 0 ? "zero" : !fresh ? "stale" : ""}`;
    byId(`state${suffix}`).textContent = channel?.value == null ? "等待有效数据" : !active ? "已停止 · 末次快照" : !fresh ? "数据超时 · 无法判断" : channel.value !== 0 ? "字段非零" : channel.last_zero_s != null ? "已归零 · 观察电机是否还在转" : "字段为零";
    byId(`zero${suffix}`).textContent = channel?.last_zero_s == null ? "尚未观察到非零 → 零" : `最近归零：本段 ${channel.last_zero_s.toFixed(3)} 秒 · ${channel.zero_events} 次`;
  }
  const main = channels.find((c) => c.can_id === "0x503");
  if (main) {
    if (zeroEvents !== null && main.zero_events > zeroEvents && active && main.value === 0 && main.age_s <= 0.5 && byId("zeroSound").checked && audio?.state === "running") {
      const oscillator = audio.createOscillator(), gain = audio.createGain();oscillator.frequency.value = 880;gain.gain.value = 0.08;
      oscillator.connect(gain);gain.connect(audio.destination);oscillator.start();oscillator.stop(audio.currentTime + 0.12);oscillator.onended = () => {oscillator.disconnect();gain.disconnect();};
    }
    zeroEvents = main.zero_events;
  }
  drawCandidate(channels);
  byId("candidateDetail").textContent = !state?.candidate ? "实时解析尚未连接，请重启上位机服务。" : `快速拨一下后停手，观察字段归零时电机是否还在转。提示略有延迟，字段含义和单位尚未确认。${state.candidate.segment ? " 检测到数据断点，曲线与归零计时已重新开始。" : ""}`;
}

async function api(path, body) {
  const options = body === undefined ? {} : {method:"POST", headers:{"Content-Type":"application/json", "X-FM03-Token":token}, body:JSON.stringify(body)};
  const response = await fetch(path, {...options, cache:"no-store", signal:AbortSignal.timeout(6000)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || "操作失败，请重试。");
  return result;
}

function notice(text, kind = "") {byId("notice").textContent = text;byId("notice").className = `notice ${kind}`;}
function download(id, name, available) {const link = byId(id);link.hidden = !available || !name;if(name) link.href = `/captures/${encodeURIComponent(name)}`;}

function renderFrames(rows) {
  const fragment = document.createDocumentFragment();
  for (const row of rows) {
    const tr = document.createElement("tr");
    const values = [(row.t_us / 1000000).toFixed(6), row.sequence, row.id, `${row.extended ? "扩展" : "标准"} · ${row.rtr ? "远程" : "数据"}`, row.dlc, row.rtr ? "—" : (row.data || "—")];
    values.forEach((value, index) => {const td = document.createElement("td");td.textContent = value;if(index === 2) td.className = "id";if(index === 3) td.className = "kind";tr.append(td);});
    fragment.append(tr);
  }
  byId("frameRows").replaceChildren(fragment);byId("frameEmpty").hidden = rows.length > 0;
}

function renderIds(rows, total) {
  const fragment = document.createDocumentFragment();
  for (const row of rows.slice(0, 60)) {
    const item = document.createElement("div");item.className = "id-item";
    const top = document.createElement("div");top.className = "id-top";
    const id = document.createElement("b");id.textContent = row.id;
    const amount = document.createElement("span");amount.textContent = `${count(row.count)} 帧`;top.append(id, amount);
    const data = document.createElement("div");data.className = "id-data";data.textContent = `${row.extended ? "扩展" : "标准"} · ${row.rtr ? "远程帧" : (row.data || "无数据字节")}`;
    const bar = document.createElement("div");bar.className = "id-bar";const fill = document.createElement("i");fill.style.width = `${total ? row.count / total * 100 : 0}%`;bar.append(fill);
    item.append(top, data, bar);fragment.append(item);
  }
  byId("idRows").replaceChildren(fragment);byId("idEmpty").hidden = rows.length > 0;byId("idCount").textContent = rows.length > 60 ? `${count(rows.length)} 个 · 显示前 60` : `${count(rows.length)} 个`;
}

function render(state) {
  latestState = state;if(state.token) token = state.token;
  const active = ["starting", "capturing", "stopping"].includes(state.phase);
  const labels = {idle:"待机", starting:"正在连接串口", capturing:"正在抓包", stopping:"正在停止并保存", stopped:"已停止 · 日志已保存", error:"抓包中断"};
  byId("phaseText").textContent = labels[state.phase] || state.phase;
  byId("phase").className = `badge ${state.phase === "capturing" ? "live" : state.phase === "error" ? "error" : ""}`;
  byId("start").disabled = busy || active || !byId("port").value || !token;
  byId("stop").disabled = busy || !["starting", "capturing"].includes(state.phase);
  byId("port").disabled = active || busy;byId("label").disabled = active || busy;byId("refresh").disabled = active || busy;
  byId("frames").textContent = count(state.frames);byId("fps").textContent = count(state.fps);
  byId("elapsed").textContent = duration(state.elapsed);byId("bytes").textContent = `已保存 ${size(state.bytes_saved)}`;
  byId("unique").textContent = count(state.unique_ids);byId("gaps").textContent = count(state.sequence_gaps);
  const status = state.status, delta = state.counter_deltas;
  const drops = delta.logger_dropped + delta.rx_missed + delta.rx_overrun;
  byId("drops").textContent = status ? count(drops) : "—";byId("errors").textContent = status ? count(delta.bus_errors) : "—";
  byId("drops").className = drops ? "bad" : "";byId("errors").className = delta.bus_errors ? "bad" : "";
  byId("bitrate").textContent = status?.bitrate ? `CAN ${(status.bitrate / 1000).toLocaleString()} kbit/s · 固件配置` : "CAN 速率待确认";
  const firmwareError = state.messages.findLast((line) => line.startsWith("# ERROR"));
  byId("firmware").textContent = firmwareError ? "固件报告错误" : !status ? "尚未读取" : active && state.status_age > 3 ? "状态更新超时" : active ? "状态已收到" : "末次状态已保存";
  byId("firmware").className = firmwareError || (active && status && state.status_age > 3) ? "bad" : "";
  byId("healthDetail").textContent = `本次增量以首条固件状态为基准。${status ? `输出队列丢帧 ${delta.logger_dropped} · 驱动丢帧 ${delta.rx_missed} · FIFO 溢出 ${delta.rx_overrun} · 当前积压 ${(status.pending || 0) + (status.logger_pending || 0)} 帧。` : "等待固件状态。"}${state.sequence_resets ? ` 检测到 ${state.sequence_resets} 次帧序号重置。` : ""}`;
  if (state.error) notice(`${state.error}${state.raw_file ? " 已收到的原始字节已保留在日志中。" : ""}`, "error");
  else if (firmwareError) notice(firmwareError, "error");
  else if (state.phase === "capturing" && !status && state.elapsed > 3) notice("串口已打开，但尚未收到监听固件状态。请检查所选串口与固件。", "warning");
  else if (state.phase === "capturing" && status && state.status_age > 3) notice("固件状态已超过 3 秒未更新，请检查开发板连接。原始日志仍在记录。", "warning");
  else if (drops || delta.bus_errors || state.sequence_gaps) notice("发现丢帧、序号缺口或总线错误。请保留日志，检查速率、接线和串口吞吐。", "warning");
  else if (state.phase === "capturing") notice(state.frames ? "正在接收 CAN 帧，原始字节实时写入日志。" : "固件正在监听，暂未收到 CAN 帧。请确认总线已连接、RS3 ↔ FM03 正常工作；当前速率仍需实测确认。");
  else if (state.phase === "stopping") notice("正在关闭串口并保存本次摘要，请稍候。");
  else if (state.phase === "starting") notice("正在连接所选串口…");
  else if (state.phase === "stopped") notice("本次抓包已结束，串口已释放。可以下载日志，或切换场景开始下一次抓包。");
  else notice("选择串口后点击开始。未接 CAN 总线时，可以先检查固件状态。");
  renderFrames(state.latest);renderIds(state.ids, state.frames);
  renderCandidate(state, state.phase === "capturing");
  byId("saveTitle").textContent = state.raw_file ? active ? "原始日志正在写入" : "本次日志" : "原始日志自动保存";
  byId("savePath").textContent = state.raw_file ? `${state.saved_directory}\\${state.raw_file}` : state.saved_directory;
  download("downloadLog", state.raw_file, !active);download("downloadSummary", state.summary_file, !active);
  byId("details").textContent = [status ? `当前固件累计计数（从开机起）：\n${JSON.stringify(status, null, 2)}` : "尚未收到固件状态。", `非帧行 ${state.non_frame_lines} · 末尾未完成行 ${state.trailing_bytes} 字节`, ...state.messages].join("\n");
}

async function refreshPorts() {
  const previous = byId("port").value, ports = await api("/api/ports");
  const choices = ports.filter((p) => p.usb).concat(ports.filter((p) => !p.usb));
  byId("port").replaceChildren();
  for (const port of choices) {const option = document.createElement("option");option.value = port.port;option.textContent = `${port.port} · ${port.description}`;byId("port").append(option);}
  if (!choices.length) {const option = document.createElement("option");option.value = "";option.textContent = "未发现串口，请连接 ESP32 后刷新";byId("port").append(option);}
  if (choices.some((p) => p.port === previous)) byId("port").value = previous;
  if (latestState) render(latestState);
}

async function action(path, body) {
  busy = true;if(latestState) render(latestState);
  try {render(await api(path, body));}
  catch (error) {notice(error.message, "error");}
  finally {busy = false;if(latestState) {byId("start").disabled = ["starting","capturing","stopping"].includes(latestState.phase) || !byId("port").value || !token;byId("stop").disabled = !["starting","capturing"].includes(latestState.phase);}}
}

byId("refresh").addEventListener("click", () => refreshPorts().catch((error) => notice(error.message,"error")));
byId("port").addEventListener("change", () => latestState && render(latestState));
byId("start").addEventListener("click", () => action("/api/start", {port:byId("port").value, label:byId("label").value}));
byId("stop").addEventListener("click", () => action("/api/stop", {}));
byId("zeroSound").addEventListener("change", async () => {
  if (!byId("zeroSound").checked) return;
  try {audio ||= new AudioContext();await audio.resume();}
  catch (_) {byId("zeroSound").checked = false;notice("浏览器未能启用提示音，仍可观察归零文字。","warning");}
});

async function poll() {
  try {if (!busy) render(await api("/api/state"));}
  catch (error) {byId("phaseText").textContent = "上位机服务未连接";byId("phase").className = "badge error";byId("start").disabled = true;byId("stop").disabled = true;token = "";renderCandidate(null,false);notice("无法连接本机上位机服务，请重新运行「启动上位机.cmd」。", "error");}
  timer = setTimeout(poll, latestState?.phase === "capturing" ? 250 : 500);
}
async function init() {await poll();try {await refreshPorts();}catch(error){notice(error.message,"error");}}
init();
