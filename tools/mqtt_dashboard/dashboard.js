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
  put("hump", hasFault(2) ? "Unreliable" : measure(data?.highest_hump_mm, "mm"));
  put("obstacle", optionalMeasure(data?.obstacle_cm, "cm"));
  put("angle", optionalMeasure(data?.scan_deg, "°"));
  put("width", optionalMeasure(data?.obstacle_width_cm, "cm"));
}

async function refresh() {
  try {
    const response = await fetch("api/telemetry", { signal: AbortSignal.timeout(4000) });
    if (!response.ok) throw new Error("Telemetry request failed");
    render(await response.json());
  } catch {
    connection("Dashboard server unavailable", "warn");
  } finally {
    setTimeout(refresh, 1000);
  }
}

refresh();
