import { useState, useEffect } from "react"
import { MapContainer, TileLayer, Circle, CircleMarker, Tooltip, useMap } from "react-leaflet"
import { geocodeAddress } from "../../utils/geo.js"
import "./leafletFix.js"

function Recenter({ lat, lng }) {
  const map = useMap()
  if (lat != null && lng != null) {
    map.setView([lat, lng], map.getZoom(), { animate: true })
  }
  return null
}

/** One-time recenter to home zone center when trigger changes (e.g. after setting home). */
function RecenterToHome({ trigger, center }) {
  const map = useMap()
  useEffect(() => {
    if (!trigger || !center?.lat || !center?.lng) return
    map.setView([center.lat, center.lng], 16, { animate: true })
  }, [trigger, center?.lat, center?.lng, map])
  return null
}

export default function MapPanel({ loc, safeZone, setSafeZone }) {
  const lat = loc.coords?.lat
  const lng = loc.coords?.lng
  const accuracy = loc.coords?.accuracy

  const [homeModalOpen, setHomeModalOpen] = useState(false)
  const [homeAddress, setHomeAddress] = useState("")
  const [geocodeLoading, setGeocodeLoading] = useState(false)
  const [geocodeError, setGeocodeError] = useState(null)
  const [homeSetConfirm, setHomeSetConfirm] = useState(null)
  const [recenterHomeTrigger, setRecenterHomeTrigger] = useState(0)

  useEffect(() => {
    if (!homeSetConfirm) return
    const t = setTimeout(() => setHomeSetConfirm(null), 4500)
    return () => clearTimeout(t)
  }, [homeSetConfirm])

  const subtitle = (() => {
    if (loc.status === "unsupported") return "Geolocation not supported"
    if (loc.status === "denied") return "Location permission denied"
    if (loc.status === "loading") return "Getting location…"
    if (loc.status === "ok" && loc.coords) {
      return `lat ${lat.toFixed(5)}, lng ${lng.toFixed(5)} • ±${Math.round(accuracy)} m`
    }
    return "Click “Center on user” to see patient location"
  })()

  const defaultCenter = [safeZone.center.lat, safeZone.center.lng]

  const openSetHomeModal = () => {
    setHomeAddress("")
    setGeocodeError(null)
    setHomeModalOpen(true)
  }

  const setHomeFromAddress = async () => {
    const query = homeAddress.trim()
    if (!query) {
      setGeocodeError("Enter an address or place name.")
      return
    }
    setGeocodeLoading(true)
    setGeocodeError(null)
    try {
      const result = await geocodeAddress(query)
      if (!result) {
        setGeocodeError("Address not found. Try a different search.")
        return
      }
      setSafeZone((z) => ({
        ...z,
        name: "Home",
        center: { lat: result.lat, lng: result.lng },
      }))
      setHomeModalOpen(false)
      setHomeSetConfirm(result.displayName)
      setRecenterHomeTrigger((n) => n + 1)
    } catch {
      setGeocodeError("Lookup failed. Please try again.")
    } finally {
      setGeocodeLoading(false)
    }
  }

  const setHomeFromCurrentLocation = () => {
    if (!loc.coords) {
      setGeocodeError("Patient location unknown. Use “Center on user” first or enter an address.")
      return
    }
    setSafeZone((z) => ({
      ...z,
      name: "Home",
      center: { lat: loc.coords.lat, lng: loc.coords.lng },
    }))
    setHomeModalOpen(false)
    setHomeSetConfirm("Current location")
    setRecenterHomeTrigger((n) => n + 1)
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
          <button style={styles.btnTeal} onClick={openSetHomeModal}>Set Home Zone</button>
        </div>
      </div>

      {homeModalOpen && (
        <div style={styles.modalBackdrop} onClick={() => setHomeModalOpen(false)}>
          <div style={styles.modalBox} onClick={(e) => e.stopPropagation()}>
            <div style={styles.modalTitle}>Set patient’s Home</div>
            <p style={styles.modalHint}>
              When the patient leaves this area, Safe Zone will show “Outside” and you can use “Center on user” to see where they are.
            </p>
            <input
              type="text"
              placeholder="Address or place (e.g. 123 Main St, City)"
              value={homeAddress}
              onChange={(e) => setHomeAddress(e.target.value)}
              onKeyDown={(e) => e.key === "Enter" && setHomeFromAddress()}
              style={styles.modalInput}
              autoFocus
            />
            {geocodeError && <div style={styles.modalError}>{geocodeError}</div>}
            <div style={styles.modalActions}>
              <button style={styles.btn} onClick={() => setHomeModalOpen(false)}>Cancel</button>
              <button
                style={styles.btnTeal}
                onClick={setHomeFromAddress}
                disabled={geocodeLoading}
              >
                {geocodeLoading ? "Looking up…" : "Set Home"}
              </button>
            </div>
            <button style={styles.useCurrentLink} onClick={setHomeFromCurrentLocation}>
              Use current patient location as Home
            </button>
          </div>
        </div>
      )}

      {homeSetConfirm && (
        <div style={styles.confirmBanner}>
          <span style={styles.confirmIcon}>✓</span>
          Home set to {homeSetConfirm.length > 60 ? homeSetConfirm.slice(0, 57) + "…" : homeSetConfirm}
        </div>
      )}

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

          <RecenterToHome trigger={recenterHomeTrigger} center={safeZone.center} />

          {/* SAFE ZONE (teal) — labeled as Home */}
          <Circle
            center={[safeZone.center.lat, safeZone.center.lng]}
            radius={safeZone.radiusM}
            pathOptions={{
              color: "#14b8a6",
              fillColor: "#14b8a6",
              fillOpacity: 0.14,
              weight: 2,
            }}
          >
            <Tooltip permanent direction="center" opacity={0.95}>
              <strong>Home</strong>
            </Tooltip>
          </Circle>

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
  modalBackdrop: {
    position: "fixed",
    inset: 0,
    background: "rgba(0,0,0,0.4)",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    zIndex: 1000,
  },
  modalBox: {
    background: "var(--panel)",
    borderRadius: "var(--radius)",
    boxShadow: "var(--shadow)",
    padding: 20,
    minWidth: 320,
    maxWidth: 420,
  },
  modalTitle: { fontWeight: 700, fontSize: 16, marginBottom: 8 },
  modalHint: {
    fontSize: 13,
    color: "var(--muted)",
    margin: "0 0 14px 0",
    lineHeight: 1.4,
  },
  modalInput: {
    width: "100%",
    padding: "10px 12px",
    fontSize: 14,
    border: "1px solid var(--border)",
    borderRadius: 12,
    marginBottom: 8,
    boxSizing: "border-box",
  },
  modalError: {
    fontSize: 12,
    color: "var(--danger)",
    marginBottom: 10,
  },
  modalActions: {
    display: "flex",
    gap: 8,
    justifyContent: "flex-end",
    marginBottom: 12,
  },
  useCurrentLink: {
    border: "none",
    background: "none",
    color: "var(--accent)",
    fontSize: 12,
    cursor: "pointer",
    padding: 0,
    textDecoration: "underline",
  },
  confirmBanner: {
    padding: "10px 14px",
    background: "rgba(34, 197, 94, 0.12)",
    borderBottom: "1px solid rgba(34, 197, 94, 0.3)",
    color: "#15803d",
    fontSize: 13,
    display: "flex",
    alignItems: "center",
    gap: 8,
  },
  confirmIcon: {
    fontWeight: 700,
    color: "var(--ok)",
  },
}