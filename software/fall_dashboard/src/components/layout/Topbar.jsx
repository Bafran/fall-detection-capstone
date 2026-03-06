export default function Topbar({ patient }) {
  const title = patient ? `${patient.name}'s Dashboard` : "Patients"
  const subtitle = patient ? "Location, safety, and vitals" : "Select a patient to view their dashboard"

  return (
    <header style={styles.topbar}>
      <div>
        <div style={{ fontSize: 16, fontWeight: 700 }}>{title}</div>
        <div style={{ fontSize: 12, color: "var(--muted)" }}>{subtitle}</div>
      </div>
      <div style={styles.user}>
        <div style={{ fontSize: 12, color: "var(--muted)" }}>Caregiver view</div>
        <div style={styles.badge}>PC</div>
      </div>
    </header>
  )
}

const styles = {
  topbar: {
    background: "var(--panel)",
    border: "1px solid var(--border)",
    borderRadius: "var(--radius)",
    boxShadow: "var(--shadow)",
    padding: "12px 14px",
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    gap: 12,
  },
  user: {
    display: "flex",
    alignItems: "center",
    gap: 10,
  },
  badge: {
    width: 34,
    height: 34,
    borderRadius: 999,
    background: "#111827",
    color: "white",
    display: "grid",
    placeItems: "center",
    fontSize: 12,
  },
}