import { useMemo, useState } from "react"
import { useParams, Navigate } from "react-router-dom"
import { haversineMeters } from "../utils/geo.js"
import { getPatientById } from "../data/patients.js"
import useBrowserLocation from "../hooks/useBrowserLocation.js"
import useDeviceStatus from "../hooks/useDeviceStatus.js"
import useDeviceEvents from "../hooks/useDeviceEvents.js"
import Sidebar from "../components/layout/Sidebar.jsx"
import Topbar from "../components/layout/Topbar.jsx"
import StatusCard from "../components/cards/StatusCard.jsx"
import MapPanel from "../components/panels/MapPanel.jsx"
import EventLog from "../components/panels/EventLog.jsx"
import PatientPanel from "../components/panels/PatientPanel.jsx"

function batteryMvToPercent(mv) {
  if (mv == null) return 0
  const min = 3300
  const max = 4200
  const percent = ((mv - min) / (max - min)) * 100
  return Math.max(0, Math.min(100, Math.round(percent)))
}

function formatTimeFromSeconds(seconds) {
  if (!seconds) return "just now"
  const dt = new Date(seconds * 1000)
  return dt.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })
}

export default function Dashboard() {
  const { patientId } = useParams()
  const patient = getPatientById(patientId ?? "")
  if (!patient) return <Navigate to="/patients" replace />

  const loc = useBrowserLocation()
  const { status, loading: statusLoading, error: statusError } = useDeviceStatus()
  const { events: apiEvents, loading: eventsLoading, error: eventsError } = useDeviceEvents()
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false)

  const [safeZone, setSafeZone] = useState({
    name: "Home",
    center: { lat: 43.4723, lng: -80.5449 },
    radiusM: 60,
  })

  const insideSafeZone = useMemo(() => {
    if (!loc.coords) return null
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

  const latestEvent = apiEvents.length > 0 ? apiEvents[0] : null
  const latestEventType = latestEvent?.event_type ?? null

  const deviceFallState = status?.fall_state ?? null

  let fallCardValue = "Unknown"
  let fallCardTone = "neutral"
  let fallCardSub = "Waiting for device data…"

  if (latestEventType === "alert_cleared") {
    fallCardValue = "Safe"
    fallCardTone = "ok"
    fallCardSub = `Cleared at ${formatTimeFromSeconds(latestEvent?.received_time)}`
  } else if (latestEventType === "manual_trigger") {
    fallCardValue = "Manual trigger"
    fallCardTone = "danger"
    fallCardSub = `Triggered at ${formatTimeFromSeconds(latestEvent?.received_time)}`
  } else if (latestEventType === "fall_confirmed") {
    fallCardValue = "Fall detected"
    fallCardTone = "danger"
    fallCardSub = `Confirmed at ${formatTimeFromSeconds(latestEvent?.received_time)}`
  } else if (statusLoading) {
    fallCardValue = "Loading..."
    fallCardTone = "neutral"
    fallCardSub = "Loading device status…"
  } else if (statusError) {
    fallCardValue = "Unavailable"
    fallCardTone = "neutral"
    fallCardSub = "Unable to reach device API"
  } else if (deviceFallState === "IDLE") {
    fallCardValue = "Safe"
    fallCardTone = "ok"
    fallCardSub = `Last update: ${formatTimeFromSeconds(status?.last_udp_time)}`
  } else if (deviceFallState === "IMPACT_DETECTED") {
    fallCardValue = "Impact detected"
    fallCardTone = "danger"
    fallCardSub = `Detected at ${formatTimeFromSeconds(status?.last_udp_time)}`
  } else if (deviceFallState === "VERIFYING") {
    fallCardValue = "Verifying"
    fallCardTone = "danger"
    fallCardSub = `Checking at ${formatTimeFromSeconds(status?.last_udp_time)}`
  } else {
    fallCardValue = "Unknown"
    fallCardTone = "neutral"
    fallCardSub = "No current fall state"
  }

  const braceletBattery = batteryMvToPercent(status?.wrist_batt_mv)
  const pendantBattery = batteryMvToPercent(status?.chest_batt_mv)

  const baseEvents = [
    {
      time: formatTimeFromSeconds(status?.last_udp_time),
      msg: "Location refreshed",
      tone: "neutral",
    },
    {
      time: formatTimeFromSeconds(status?.last_udp_time),
      msg: safeZoneValue === "Inside" ? "Inside safe zone: Home" : "Outside safe zone: Home",
      tone: safeZoneValue === "Inside" ? "ok" : "danger",
    },
    {
      time: formatTimeFromSeconds(status?.last_udp_time),
      msg: `Bracelet battery: ${braceletBattery}% • Pendant battery: ${pendantBattery}%`,
      tone: "neutral",
    },
  ]

  const eventEntriesFromApi = apiEvents.map((evt) => {
    const time = formatTimeFromSeconds(evt.received_time)

    let msg = "Device event received"
    let tone = "neutral"

    if (evt.event_type === "manual_trigger") {
      msg = `Manual emergency trigger from ${evt.device_id ?? "device"}`
      tone = "danger"
    } else if (evt.event_type === "fall_confirmed") {
      msg = `Fall confirmed from ${evt.device_id ?? "device"}`
      tone = "danger"
    }

    return { time, msg, tone }
  })

  const events = [...eventEntriesFromApi, ...baseEvents]

  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: sidebarCollapsed ? "92px 1fr" : "260px 1fr",
        minHeight: "100vh",
        transition: "grid-template-columns 220ms ease",
      }}
    >
      <Sidebar
        collapsed={sidebarCollapsed}
        onToggle={() => setSidebarCollapsed((v) => !v)}
      />

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
            value={statusLoading ? "--" : `${braceletBattery}%`}
            tone="neutral"
            sub="Device: Bracelet"
            icon="battery"
            percent={braceletBattery}
          />

          <StatusCard
            title="Pendant Battery"
            value={statusLoading ? "--" : `${pendantBattery}%`}
            tone="neutral"
            sub="Device: Pendant"
            icon="battery"
            percent={pendantBattery}
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

        {(statusError || eventsError) && (
          <div style={styles.errorBanner}>
            API warning: {statusError ?? eventsError}
          </div>
        )}
      </main>
    </div>
  )
}

const styles = {
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
  errorBanner: {
    padding: "10px 12px",
    borderRadius: 12,
    background: "rgba(239,68,68,.12)",
    color: "#991b1b",
    fontSize: 13,
    border: "1px solid rgba(239,68,68,.18)",
  },
}