const states = ["Idle", "Following", "Turning", "Scanning", "Avoiding", "Recovering", "Finished", "Fault"];
const faults = ["Motor stall", "Encoder fault", "IMU fault", "No echo", "Wi-Fi loss"];
const directions = ["None", "Left", "Right", "Straight", "U-turn"];
const sensorIds = ["left", "centre", "right"];
const empty = "—";

function put(id, value) {
  const element = document.getElementById(id);
  if (element.textContent !== String(value)) element.textContent = value;
  return element;
}

function connection(label, tone) {
  put("connection", label).dataset.tone = tone;
}

function render(snapshot) {
  const data = snapshot.telemetry;
  const simulated = snapshot.demo;
  const faultBits = data?.faults ?? 0;
  const hasFault = bit => Boolean(faultBits & (1 << bit));
  const measure = (value, unit) => value == null ? empty : `${value} ${unit}`;
  const optionalMeasure = (value, unit) => simulated ? measure(value, unit) : "Not published";

  put("robot", simulated ? "Demo mode · Simulated robot" : snapshot.client_id);
  put("heartbeat", snapshot.heartbeat === null ? "Waiting / expired" : snapshot.heartbeat ? "Online" : "Offline");
  put("telemetry-topic", snapshot.telemetry_topic);
  put("status-topic", snapshot.status_topic);
  put("rejected", snapshot.rejected);
  if (simulated) connection("Demo feed", "good");
  else if (snapshot.heartbeat === false) connection("Robot offline", "warn");
  else if (snapshot.stale) connection("Telemetry stale", "warn");
  else if (data) connection("Receiving telemetry", "good");
  else connection("Waiting for telemetry", "");

  put("state", states[data?.state] ?? empty);
  put("faults", data ? faults.filter((name, bit) => hasFault(bit)).join(", ") || "None" : empty);
  put("received", snapshot.telemetry_at == null ? empty : new Date(snapshot.telemetry_at * 1000).toLocaleTimeString());
  put("distance", hasFault(1) ? "Unreliable" : measure(data?.distance_cm, "cm"));
  put("speed", !data ? empty : hasFault(1) ? "Unreliable" :
      data.speed_valid ? measure(data.speed_cms, "cm/s") : "Not published");
  sensorIds.forEach((id, bit) => {
    const value = data ? (data.line_sensors >> bit) & 1 : empty;
    put(id, value).dataset.active = value;
  });
  for (const [id, count] of [["barcode-black", data?.barcode_black],
                             ["barcode-white", data?.barcode_white]]) {
    put(id, !data ? empty : count == null ? "Not published" :
        count === 65535 ? "65,535+" : count);
  }
  const letter = data?.barcode_letter;
  put("barcode-letter", !data ? empty : letter == null ? "Not published" :
      letter === 0 ? "None yet" : String.fromCharCode(64 + letter));
  put("barcode", directions[data?.barcode] ?? empty);
  put("hump", data?.highest_hump_mm > 0 ? measure(data.highest_hump_mm, "mm") :
      hasFault(2) ? "Unreliable" : data ? "No measurement yet" : empty);
  put("obstacle", optionalMeasure(data?.obstacle_cm, "cm"));
  put("angle", optionalMeasure(data?.scan_deg, "°"));
  put("width", optionalMeasure(data?.obstacle_width_cm, "cm"));
}

const controls = document.querySelectorAll("[data-command]");
const barcodeForm = document.getElementById("barcode-actions");
const barcodeFields = document.getElementById("barcode-fields");
let liveControls = false;
let sendingCommand = false;
let sendingBarcode = false;

function updateControls() {
  for (const button of controls) button.disabled = !liveControls || sendingCommand;
  barcodeFields.disabled = !liveControls || sendingBarcode;
}

async function publishBarcodeActions(event) {
  event.preventDefault();
  if (!liveControls || sendingBarcode || !barcodeForm.reportValidity()) return;
  const params = new URLSearchParams();
  for (const [action, letter] of new FormData(barcodeForm)) {
    params.set(action, letter.toUpperCase());
  }
  if (new Set(params.values()).size !== 4) {
    put("barcode-action-status", "Use a different letter for each action.");
    return;
  }
  const query = params.toString();
  sendingBarcode = true;
  updateControls();
  put("barcode-action-status", "Sending barcode actions…");
  try {
    const response = await fetch("api/mapping?" + query, {
      method: "POST",
      headers: { "X-Dashboard-Command": "1" },
      signal: AbortSignal.timeout(5000),
    });
    if (!response.ok) throw new Error("Barcode publish failed");
    put("barcode-action-status", "Barcode actions sent to the broker.");
  } catch {
    put("barcode-action-status", "Publish unconfirmed. Check the broker and press Enter to retry.");
  } finally {
    sendingBarcode = false;
    updateControls();
  }
}

barcodeForm.addEventListener("change", publishBarcodeActions);
barcodeForm.addEventListener("submit", publishBarcodeActions);
barcodeForm.addEventListener("keydown", (event) => {
  if (event.key === "Enter") publishBarcodeActions(event);
});

for (const button of controls) {
  button.addEventListener("click", async () => {
    sendingCommand = true;
    updateControls();
    put("command-status", "Sending…");
    try {
      const response = await fetch("api/command/" + button.dataset.command, {
        method: "POST",
        headers: { "X-Dashboard-Command": "1" },
        signal: AbortSignal.timeout(5000),
      });
      if (!response.ok) throw new Error("Command request failed");
      put("command-status", button.textContent + " sent to broker");
    } catch {
      put("command-status", "Could not confirm publish. Check the broker connection.");
    } finally {
      sendingCommand = false;
      updateControls();
    }
  });
}

async function refresh() {
  try {
    const response = await fetch("api/telemetry", { signal: AbortSignal.timeout(4000) });
    if (!response.ok) throw new Error("Telemetry request failed");
    const packet = await response.json();
    render(packet);
    liveControls = !packet.demo;
    if (packet.demo) put("command-status", "Controls require live MQTT mode");
  } catch {
    liveControls = false;
    connection("Dashboard server unavailable", "warn");
  } finally {
    updateControls();
    setTimeout(refresh, 1000);
  }
}

refresh();
