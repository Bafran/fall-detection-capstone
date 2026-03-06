import { useMemo, useState } from "react"
import { useParams, Navigate } from "react-router-dom"
import { haversineMeters } from "../utils/geo.js"
import { getPatientById } from "../data/patients.js"
import useBrowserLocation from "../hooks/useBrowserLocation.js"
import useFallState from "../hooks/useFallState.js"
import Sidebar from "../components/layout/Sidebar.jsx"
import Topbar from "../components/layout/Topbar.jsx"
import StatusCard from "../components/cards/StatusCard.jsx"
import MapPanel from "../components/panels/MapPanel.jsx"
import EventLog from "../components/panels/EventLog.jsx"
import PatientPanel from "../components/panels/PatientPanel.jsx"

export default function Dashboard() {
  const { patientId } = useParams()
  const patient = getPatientById(patientId ?? "")
  if (!patient) return <Navigate to="/patients" replace />

  const loc = useBrowserLocation()
  const fallState = useFallState()
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

  const fallEvent = fallState?.last_event
  const fallDetected = fallEvent === "fall_confirmed"
  const fallCardValue = fallDetected ? "Fall detected" : "Safe"
  const fallCardTone = fallDetected ? "danger" : "ok"

  let fallCardSub = "No recent falls"
  let fallEventLogEntry = null
  if (fallState?.updated_at) {
    const dt = new Date(fallState.updated_at * 1000)
    const timeStr = dt.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })
    if (fallDetected) {
      fallCardSub = `Last fall: ${timeStr}`
      fallEventLogEntry = {
        time: timeStr,
        msg: `Fall confirmed from ${fallState.last_device ?? "device"}`,
        tone: "danger",
      }
    } else {
      fallCardSub = `Last update: ${timeStr}`
    }
  } else if (!fallState) {
    fallCardSub = "Waiting for device data…"
  }

  const baseEvents = [
    { time: "11:45", msg: "Location refreshed", tone: "neutral" },
    { time: "11:32", msg: "Entered safe zone: Home", tone: "ok" },
    { time: "10:58", msg: "Device battery: 82%", tone: "neutral" },
  ]

  const events = fallEventLogEntry ? [fallEventLogEntry, ...baseEvents] : baseEvents

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
        <Topbar patient={patient} />

        <section style={styles.cards}>
          <StatusCard
            title="Fall Detection"
            value={fallCardValue}
            tone={fallCardTone}
            sub={fallCardSub}
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
            patient={patient}
            safeZoneValue={safeZoneValue}
          />
          <EventLog events={events} />
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
    gridTemplateColumns: "repeat(4, minmax(0, 1fr))",
    gap: 12,
  },
  grid: {
    display: "grid",
    gridTemplateColumns: "1.4fr .9fr",
    gap: 16,
    alignItems: "start",
  },
}