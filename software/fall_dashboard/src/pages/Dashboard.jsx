import { useMemo, useState } from "react"
import { haversineMeters } from "../utils/geo.js"
import useBrowserLocation from "../hooks/useBrowserLocation.js"
import Sidebar from "../components/layout/Sidebar.jsx"
import Topbar from "../components/layout/Topbar.jsx"
import StatusCard from "../components/cards/StatusCard.jsx"
import MapPanel from "../components/panels/MapPanel.jsx"
import EventLog from "../components/panels/EventLog.jsx"
import PatientPanel from "../components/panels/PatientPanel.jsx"

export default function Dashboard() {
  const loc = useBrowserLocation()
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false)

  const [safeZone, setSafeZone] = useState({
    // default: "Home" near Waterloo (change anytime)
    name: "Home",
    center: { lat: 43.4723, lng: -80.5449 },
    radiusM: 60,
  })

  const insideSafeZone = useMemo(() => {
    if (!loc.coords) return null // unknown until we have a location
    const d = haversineMeters(
      { lat: loc.coords.lat, lng: loc.coords.lng },
      safeZone.center
    )
    return d <= safeZone.radiusM
  }, [loc.coords, safeZone])

  const safeZoneValue =
    insideSafeZone == null ? "Unknown" : insideSafeZone ? "Inside" : "Outside"

  const safeZoneTone =
    insideSafeZone == null ? "neutral" : insideSafeZone ? "ok" : "danger"

  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: sidebarCollapsed ? "92px 1fr" : "260px 1fr",
        minHeight: "100vh",
        transition: "grid-template-columns 220ms ease",
      }}
    >
      <Sidebar collapsed={sidebarCollapsed} onToggle={() => setSidebarCollapsed(v => !v)} />
      <main style={styles.main}>
        <Topbar />

        <section style={styles.cards}>
          <StatusCard
            title="Fall Detection"
            value="Safe"
            tone="ok"
            sub="Last check: 1 min ago"
            icon="shield"
          />

          <StatusCard
            title="Safe Zone"
            value={safeZoneValue}
            tone={safeZoneTone}
            sub={`${safeZone.name} zone`}
            icon="pin"
          />

          <StatusCard
            title="Heart Rate"
            value="67 bpm"
            tone="neutral"
            sub="Updated: 5 min ago"
            icon="heart"
          />

          <StatusCard
            title="Bracelet Battery"
            value="82%"
            tone="neutral"
            sub="Device: Bracelet"
            icon="battery"
            percent={82}
          />

          <StatusCard
            title="Pendant Battery"
            value="76%"
            tone="neutral"
            sub="Device: Pendant"
            icon="battery"
            percent={76}
          />
        </section>

        <section style={styles.grid}>
          <MapPanel
            loc={loc}
            safeZone={safeZone}
            setSafeZone={setSafeZone}
          />
          <PatientPanel
            safeZoneValue={safeZoneValue}
          />
          <EventLog />
        </section>
      </main>
    </div>
  )
}

const styles = {
  app: {
    display: "grid",
    gridTemplateColumns: "260px 1fr",
    minHeight: "100vh",
  },
  main: {
    padding: 18,
    display: "flex",
    flexDirection: "column",
    gap: 16,
  },
  cards: {
    display: "grid",
    gridTemplateColumns: "repeat(5, minmax(0, 1fr))",
    gap: 12,
  },
  grid: {
    display: "grid",
    gridTemplateColumns: "1.4fr .9fr",
    gap: 16,
    alignItems: "start",
  },
}