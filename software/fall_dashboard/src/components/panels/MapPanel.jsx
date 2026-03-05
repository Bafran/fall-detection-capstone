import { MapContainer, TileLayer, Circle, useMap, CircleMarker } from "react-leaflet"
import "./leafletFix.js"

function Recenter({ lat, lng }) {
  const map = useMap()
  if (lat != null && lng != null) {
    map.setView([lat, lng], map.getZoom(), { animate: true })
  }
  return null
}

export default function MapPanel({ loc, safeZone, setSafeZone }) {
  const lat = loc.coords?.lat
  const lng = loc.coords?.lng
  const accuracy = loc.coords?.accuracy

  const subtitle = (() => {
    if (loc.status === "unsupported") return "Geolocation not supported"
    if (loc.status === "denied") return "Location permission denied"
    if (loc.status === "loading") return "Getting location…"
    if (loc.status === "ok" && loc.coords) {
      return `lat ${lat.toFixed(5)}, lng ${lng.toFixed(5)} • ±${Math.round(accuracy)} m`
    }
    return "Click Center on user"
  })()

  const defaultCenter = [safeZone.center.lat, safeZone.center.lng]

  const setHomeZoneHere = () => {
    if (!loc.coords) return
    setSafeZone((z) => ({
      ...z,
      center: { lat: loc.coords.lat, lng: loc.coords.lng },
    }))
  }

  return (
    <section style={styles.panel}>
      <div style={styles.head}>
        <div>
          <div style={{ fontWeight: 700 }}>Location</div>
          <div style={styles.sub}>{subtitle}</div>
        </div>

        <div style={styles.actions}>
          <button style={styles.btn} onClick={loc.requestOnce}>Center on user</button>
          <button style={styles.btnTeal} onClick={setHomeZoneHere}>Set Home Zone</button>
        </div>
      </div>

      <div style={styles.mapWrap}>
        <MapContainer
          center={lat != null && lng != null ? [lat, lng] : defaultCenter}
          zoom={16}
          scrollWheelZoom={false}
          zoomControl={false}
          style={styles.map}
        >
          <TileLayer
            url="https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png"
          />

          {/* SAFE ZONE (teal) */}
          <Circle
            center={[safeZone.center.lat, safeZone.center.lng]}
            radius={safeZone.radiusM}
            pathOptions={{
              color: "#14b8a6",
              fillColor: "#14b8a6",
              fillOpacity: 0.14,
              weight: 2,
            }}
          />

          {/* CURRENT LOCATION (orange) */}
          {lat != null && lng != null && (
            <>
              <CircleMarker
                center={[lat, lng]}
                radius={9}
                pathOptions={{
                  color: "#ffffff",
                  weight: 2,
                  fillColor: "#fbbf24",
                  fillOpacity: 1,
                }}
              />

              {/* Accuracy bubble (light orange) */}
              <Circle
                center={[lat, lng]}
                radius={Math.min(Math.max((accuracy ?? 20) * 0.6, 12), 45)}
                pathOptions={{
                  color: "#fbbf24",
                  fillColor: "#fbbf24",
                  fillOpacity: 0.10,
                  weight: 2,
                }}
              />

              <Recenter lat={lat} lng={lng} />
            </>
          )}
        </MapContainer>
      </div>
    </section>
  )
}

const styles = {
  panel: {
    background: "var(--panel)",
    border: "1px solid var(--border)",
    borderRadius: "var(--radius)",
    boxShadow: "var(--shadow)",
    overflow: "hidden",
    gridColumn: "1 / 2",
  },
  head: {
    padding: "12px 14px",
    borderBottom: "1px solid var(--border)",
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    gap: 10,
  },
  sub: { fontSize: 12, color: "var(--muted)", marginTop: 2 },
  actions: { display: "flex", gap: 8, alignItems: "center" },
  btn: {
    border: "1px solid var(--border)",
    background: "white",
    borderRadius: 12,
    padding: "8px 10px",
    fontSize: 12,
    cursor: "pointer",
    whiteSpace: "nowrap",
  },
  btnTeal: {
    border: "1px solid rgba(20,184,166,.35)",
    background: "rgba(20,184,166,.12)",
    color: "#0f766e",
    borderRadius: 12,
    padding: "8px 10px",
    fontSize: 12,
    cursor: "pointer",
    whiteSpace: "nowrap",
    fontWeight: 700,
  },
  mapWrap: { height: 360 },
  map: { height: "100%", width: "100%" },
}